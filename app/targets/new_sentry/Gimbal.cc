#include "Gimbal.hpp"
#include "gimbal-tool-suite/dynamics/dynamics.hpp"
#include <cstdio>

namespace {
struct GimbalState {
  f32 target_yaw{0.0f};
  f32 current_yaw{0.0f};
  f32 target_pitch{0.0f};
  f32 current_pitch{0.0f};
  f32 yaw_s{0.0f};
  f32 yaw_a{0.0f};
  f32 pitch_s{0.0f};
  f32 pitch_a{0.0f};
} gimbal_state;
}  // namespace

namespace {
constexpr size_t kIdentifyHarmonicCount = 5;
constexpr f32 kIdentifyBaseFreqHz = 0.1f;
constexpr f32 kRpmToRadPerSec = static_cast<f32>(M_PI) * 2.0f / 60.0f;
constexpr f32 kIdentifyPitchTopLimit = -2.0f;
constexpr f32 kIdentifyPitchBottomLimit = -3.14f;
constexpr f32 kIdentifyPitchCenter = (kIdentifyPitchTopLimit + kIdentifyPitchBottomLimit) * 0.5f;
constexpr f32 kIdentifyYawTopLimit = 2310.0f;
constexpr f32 kIdentifyYawBottomLimit = 410.0f;
constexpr f32 kIdentifyYawCenter = (kIdentifyYawTopLimit + kIdentifyYawBottomLimit) * 0.5f;
constexpr f32 kIdentifyYawAmp[kIdentifyHarmonicCount] = {600.0f, -300.0f, 180.0f, -100.0f, 50.0f};
constexpr f32 kIdentifyPitchAmp[kIdentifyHarmonicCount] = {0.24, -0.16f, 0.14f, -0.10f, 0.08f};
constexpr f32 kGm6020VoltageCmdLimit = 25000.0f;
constexpr f32 kGm6020BusVoltage = 24.0f;
constexpr f32 kGm6020TorqueConstant = 0.741f;
constexpr f32 kGm6020PhaseResistance = 1.8f;
constexpr f32 kGm6020SpeedConstantRpmPerVolt = 13.33f;
constexpr f32 kGm6020BackEmfConstant = 60.0f / (2.0f * static_cast<f32>(M_PI) * kGm6020SpeedConstantRpmPerVolt);
constexpr f32 kNormalFfMaxYawSpeed = 8.0f;
constexpr f32 kNormalFfMaxPitchSpeed = 4.0f;
constexpr f32 kNormalFfMaxYawAccel = 80.0f;
constexpr f32 kNormalFfMaxPitchAccel = 40.0f;

// 使用 gimbal-tool-suite 完整动力学模型（3D 重力补偿），参数来自 ident.ipynb 辨识结果
Gimbal2DofDynamics g_gimbal_dynamics;
void InitDynamicsTheta() {
  Eigen::Matrix<float, 9, 1> theta;
  theta << 0.0704193, 0.06767012, 0.0, 0.22035452, 0.12618492, 0.04860608, 0.47820438, 3.54705607, 0.16939606;

  g_gimbal_dynamics.SetTheta(theta);
}

struct IdentifyTrajectoryPoint {
  f32 q;
  f32 dq;
  f32 ddq;
};

IdentifyTrajectoryPoint EvaluateIdentifyTrajectory(f32 center, const f32 (&amplitudes)[kIdentifyHarmonicCount], f32 t) {
  IdentifyTrajectoryPoint point{center, 0.0f, 0.0f};
  const f32 wf = 2.0f * static_cast<f32>(M_PI) * kIdentifyBaseFreqHz;
  for (size_t i = 0; i < kIdentifyHarmonicCount; ++i) {
    const f32 k = static_cast<f32>(i + 1);
    const f32 kwf = k * wf;
    const f32 phase = kwf * t;
    point.q += amplitudes[i] * std::sin(phase);
    point.dq += amplitudes[i] * kwf * std::cos(phase);
    point.ddq -= amplitudes[i] * kwf * kwf * std::sin(phase);
  }
  return point;
}

f32 YawVoltageCmdToTorque(f32 voltage_cmd, f32 speed_rad_per_sec) {
  const f32 voltage = voltage_cmd / kGm6020VoltageCmdLimit * kGm6020BusVoltage;
  return kGm6020TorqueConstant / kGm6020PhaseResistance * (voltage - kGm6020BackEmfConstant * speed_rad_per_sec);
}

f32 YawTorqueToVoltageCmd(f32 torque_nm, f32 speed_rad_per_sec) {
  const f32 voltage =
      torque_nm * kGm6020PhaseResistance / kGm6020TorqueConstant + kGm6020BackEmfConstant * speed_rad_per_sec;
  return rm::modules::Clamp(voltage / kGm6020BusVoltage * kGm6020VoltageCmdLimit, -kGm6020VoltageCmdLimit,
                            kGm6020VoltageCmdLimit);
}

int AppendFloat(char *buffer, size_t size, f32 value) {
  if (size == 0) {
    return 0;
  }
  const bool negative = value < 0.0f;
  f32 abs_value = negative ? -value : value;
  unsigned long integer_part = static_cast<unsigned long>(abs_value);
  unsigned long fractional_part =
      static_cast<unsigned long>((abs_value - static_cast<f32>(integer_part)) * 1000000.0f + 0.5f);
  if (fractional_part >= 1000000UL) {
    ++integer_part;
    fractional_part -= 1000000UL;
  }
  return std::snprintf(buffer, size, "%s%lu.%06lu", negative ? "-" : "", integer_part, fractional_part);
}
}  // namespace

void Gimbal::GimbalInit() {
  InitDynamicsTheta();
  gimbal->gimbal_up_yaw_target_ = globals->hipnuc_imu->yaw();
  gimbal->gimbal_down_yaw_target_ = globals->ahrs.euler_angle().yaw;
  gimbal->gimbal_pitch_target_ = globals->hipnuc_imu->pitch();
  gimbal->last_yaw_target_ = gimbal->gimbal_up_yaw_target_;
  gimbal->last_pitch_target_ = gimbal->gimbal_pitch_target_;
  gimbal->up_yaw_move_limiter_.ResetAt(globals->hipnuc_imu->yaw());
  gimbal->down_yaw_move_limiter_.ResetAt(globals->ahrs.euler_angle().yaw);
  gimbal->pitch_torque_ = 0.0f;
  gimbal->identify_time_s_ = 0.0f;
  gimbal->identify_yaw_position_ = globals->up_yaw_motor->encoder();
  gimbal->identify_yaw_speed_ = static_cast<f32>(globals->up_yaw_motor->rpm()) * kRpmToRadPerSec;
  gimbal->identify_pitch_position_ = -globals->pitch_motor->pos();
  gimbal->identify_pitch_speed_ = -globals->pitch_motor->vel();
  gimbal->last_yaw_speed_ref_ = 0.0f;
  gimbal->last_pitch_speed_ref_ = 0.0f;
}

void Gimbal::GimbalTask() {
  gimbal->GimbalStateUpdate();
  gimbal->heat_limit_ = globals->referee_data->data().robot_status.shooter_barrel_heat_limit;
  gimbal->heat_current_ = globals->referee_data->data().power_heat_data.shooter_17mm_1_barrel_heat;
  gimbal_state.target_yaw = gimbal->gimbal_up_yaw_target_;
  // gimbal_state.current_yaw = globals->up_yaw_motor->encoder();
  gimbal_state.current_yaw = globals->hipnuc_imu->yaw();
  gimbal_state.target_pitch = gimbal->gimbal_pitch_target_;
  // gimbal_state.current_pitch = globals->pitch_motor->pos();
  gimbal_state.current_pitch = globals->hipnuc_imu->pitch();
  gimbal_state.yaw_s = globals->aimbot_communicator->yaw_vel();
  gimbal_state.pitch_s = globals->aimbot_communicator->pitch_vel();
  gimbal_state.yaw_a = globals->aimbot_communicator->yaw_acc();
  gimbal_state.pitch_a = globals->aimbot_communicator->pitch_acc();
}

void Gimbal::GimbalStateUpdate() {
  if (!globals->device_gimbal.all_device_ok()) {
    gimbal->GimbalDisableUpdate();  // 云台电机失能计算
  } else {
    switch (globals->StateMachine_) {
      case kNoForce:                    // 无力模式下，所有电机失能
        gimbal->GimbalDisableUpdate();  // 云台电机失能计算
        break;

      case kTest:                      // 测试模式下，发射系统与拨盘电机失能
        gimbal->GimbalEnableUpdate();  // 云台电机使能计算
        break;

      case kMatch:                    // 比赛模式下，所有电机正常工作
        gimbal->GimbalMatchUpdate();  // 云台电机使能计算
        break;

      default:                          // 错误状态，所有电机失能
        gimbal->GimbalDisableUpdate();  // 云台电机失能计算
        break;
    }
  }
  if (globals->referee_data->data().robot_status.power_management_shooter_output && !globals->last_shooter_power) {
    globals->shooter_init_time = 1500;
  }
  globals->last_shooter_power = globals->referee_data->data().robot_status.power_management_shooter_output;
  if (globals->shooter_init_time > 0) {
    globals->shooter_init_time--;
  }
  if (!globals->device_shoot.all_device_ok() || globals->shooter_init_time > 0 ||
      !globals->referee_data->data().robot_status.power_management_shooter_output) {
    gimbal->ShootDisableUpdate();  // 发射机构失能计算
  } else {
    switch (globals->StateMachine_) {
      case kMatch:                    // 比赛模式下，发射系统与拨盘电使能
        gimbal->ShootEnableUpdate();  // 发射机构使能计算
        break;
      case kTest:
        switch (gimbal->GimbalMove_) {
          case kGbAimbot:
            if (globals->wfly_et16s->switch_position(rc_ch::SF) == SwitchPosition::kDown) {
              gimbal->ShootEnableUpdate();  // 发射机构使能计算
            } else {
              gimbal->ShootDisableUpdate();  // 发射机构失能计算
            }
            break;
          case kGbRemote:
          default:
            gimbal->ShootDisableUpdate();  // 发射机构失能计算
            break;
        }
        break;
      case kNoForce:                   // 无力模式下，所有电机失能
      default:                         // 错误状态，所有电机失能
        gimbal->ShootDisableUpdate();  // 发射机构失能计算
        break;
    }
  }
}

void Gimbal::GimbalRCTargetUpdate() {
  gimbal->gimbal_up_yaw_target_ -= rm::modules::Map(globals->wfly_et16s->left_x(), -1, 1, -0.004f, 0.004f);
  gimbal->gimbal_down_yaw_target_ -= rm::modules::Map(globals->wfly_et16s->left_x(), -1, 1, -0.004f, 0.004f);
  gimbal->gimbal_pitch_target_ -= rm::modules::Map(globals->wfly_et16s->left_y(), -1, 1, -0.004f, 0.004f);
  gimbal->gimbal_up_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_up_yaw_target_, -static_cast<f32>(M_PI), M_PI);
  gimbal->gimbal_down_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_down_yaw_target_, -static_cast<f32>(M_PI), M_PI);
  gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_,  // pitch轴限位
                                                    gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);
  // 遥控模式：从位置目标差分得到速度/加速度
  const f32 yaw_delta = rm::modules::Wrap(gimbal->gimbal_up_yaw_target_ - gimbal->last_yaw_target_,
                                          -static_cast<f32>(M_PI), static_cast<f32>(M_PI));
  yaw_speed_ref = rm::modules::Clamp(yaw_delta / gimbal->Ts, -kNormalFfMaxYawSpeed, kNormalFfMaxYawSpeed);
  pitch_speed_ref = rm::modules::Clamp((gimbal->gimbal_pitch_target_ - gimbal->last_pitch_target_) / gimbal->Ts,
                                       -kNormalFfMaxPitchSpeed, kNormalFfMaxPitchSpeed);
  yaw_accel_ref = rm::modules::Clamp((yaw_speed_ref - gimbal->last_yaw_speed_ref_) / gimbal->Ts, -kNormalFfMaxYawAccel,
                                     kNormalFfMaxYawAccel);
  pitch_accel_ref = rm::modules::Clamp((pitch_speed_ref - gimbal->last_pitch_speed_ref_) / gimbal->Ts,
                                       -kNormalFfMaxPitchAccel, kNormalFfMaxPitchAccel);
  gimbal->yaw_speed_ff_ = gimbal->Kf * yaw_speed_ref;
  gimbal->last_yaw_target_ = gimbal->gimbal_up_yaw_target_;
  gimbal->last_pitch_target_ = gimbal->gimbal_pitch_target_;
  gimbal->last_yaw_speed_ref_ = yaw_speed_ref;
  gimbal->last_pitch_speed_ref_ = pitch_speed_ref;

  gimbal->up_yaw_move_limiter_.ResetAt(globals->hipnuc_imu->yaw());
  gimbal->down_yaw_move_limiter_.ResetAt(globals->ahrs.euler_angle().yaw);
}

void Gimbal::GimbalScanTargetUpdate() {
  // 上部yaw轴扫描
  if (globals->navigate_communicator->aimbot_mode() || globals->navigate_communicator->outpost_mode()) {
    gimbal->gimbal_up_yaw_target_ =
        globals->hipnuc_imu->yaw() -
        rm::modules::Map(static_cast<f32>(globals->up_yaw_motor->encoder() - gimbal->mid_up_yaw_pos_),  //
                         0, 8191, 0, 2 * static_cast<f32>(M_PI));
  } else {
    if (globals->up_yaw_motor->encoder() >= gimbal->max_up_yaw_pos_ && globals->up_yaw_motor->encoder() <= 5000) {
      gimbal->scan_yaw_flag_ = true;
    } else if (globals->up_yaw_motor->encoder() <= gimbal->min_up_yaw_pos_ ||
               globals->up_yaw_motor->encoder() >= 6000) {
      gimbal->scan_yaw_flag_ = false;
    }
    if (gimbal->scan_yaw_flag_) {
      gimbal->gimbal_up_yaw_target_ -= 0.0025f;
    } else {
      gimbal->gimbal_up_yaw_target_ += 0.0025f;
    }
  }
  // pitch轴扫描
  if (globals->navigate_communicator->aimbot_mode()) {
    gimbal->gimbal_pitch_target_ = -0.0f;
  } else if (globals->navigate_communicator->outpost_mode()) {
    gimbal->gimbal_pitch_target_ = -0.3f;
  } else {
    if (gimbal->gimbal_pitch_target_ <= gimbal->lowest_aimbot_pitch_angle_) {
      gimbal->scan_pitch_flag_ = false;
    } else if (gimbal->gimbal_pitch_target_ >= gimbal->highest_pitch_angle_) {
      gimbal->scan_pitch_flag_ = true;
    }
    if (gimbal->scan_pitch_flag_) {
      gimbal->gimbal_pitch_target_ -= 0.005f;
    } else {
      gimbal->gimbal_pitch_target_ += 0.005f;
    }
  }
  // 下部yaw轴扫描
  if (globals->navigate_communicator->scan_mode()) {
    gimbal->GimbalMove_ = kGbScan;
  } else {
    // gimbal->GimbalMove_ = kGbScan;
    gimbal->GimbalMove_ = kGbNavigate;
  }
  if (gimbal->perception_time_ > 0) {
    gimbal->perception_time_--;
  } else {
    if (gimbal->GimbalMove_ == kGbNavigate) {
      gimbal->gimbal_down_yaw_target_ +=
          rm::modules::Map(rm::modules::Clamp(globals->navigate_communicator->target_yaw_speed(), -1.0f, 1.0f), -1.0f,
                           1.0f, -0.01f, 0.01f);
    } else {
      gimbal->gimbal_down_yaw_target_ += 0.001f;
    }
  }
  // 基于下部yaw轴转速增减上部yaw轴转速
  if (gimbal->GimbalMove_ == kGbNavigate) {
    gimbal->gimbal_up_yaw_target_ +=
        rm::modules::Map(rm::modules::Clamp(globals->navigate_communicator->target_yaw_speed(), -1.0f, 1.0f), -1.0f,
                         1.0f, -0.01f, 0.01f);
  } else {
    gimbal->gimbal_up_yaw_target_ += 0.001f;
  }
  gimbal->gimbal_up_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_up_yaw_target_,  // 上部yaw轴周期限制
                                                    -static_cast<f32>(M_PI), M_PI);
  gimbal->gimbal_down_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_down_yaw_target_,  // 下部yaw轴周期限制
                                                      -static_cast<f32>(M_PI), M_PI);

  gimbal->up_yaw_move_limiter_.ResetAt(globals->hipnuc_imu->yaw());
  gimbal->down_yaw_move_limiter_.ResetAt(globals->ahrs.euler_angle().yaw);
}

void Gimbal::GimbalPerceptTargetUpdate() {
  if (gimbal->percept_move_complete_) {
    if (globals->navigate_communicator->perception_flag() >> 0 & 0x01) {
      gimbal->up_yaw_percept_target_ = globals->hipnuc_imu->yaw() + static_cast<f32>(M_PI) / 2.0f;
      gimbal->down_yaw_percept_target_ = globals->ahrs.euler_angle().yaw + static_cast<f32>(M_PI) / 2.0f;
    } else if (globals->navigate_communicator->perception_flag() >> 2 & 0x01) {
      gimbal->up_yaw_percept_target_ = globals->hipnuc_imu->yaw() - static_cast<f32>(M_PI) / 2.0f;
      gimbal->down_yaw_percept_target_ = globals->ahrs.euler_angle().yaw - static_cast<f32>(M_PI) / 2.0f;
    } else if (globals->navigate_communicator->perception_flag() >> 1 & 0x01) {
      gimbal->up_yaw_percept_target_ = globals->hipnuc_imu->yaw() + static_cast<f32>(M_PI);
      gimbal->down_yaw_percept_target_ = globals->ahrs.euler_angle().yaw + static_cast<f32>(M_PI);
    }
    gimbal->up_yaw_move_limiter_.SetTarget(gimbal->up_yaw_percept_target_);
    gimbal->down_yaw_move_limiter_.SetTarget(gimbal->down_yaw_percept_target_);
    gimbal->percept_move_complete_ = false;
  }
  gimbal->gimbal_up_yaw_target_ = gimbal->up_yaw_move_limiter_.Update(0.002f);
  gimbal->gimbal_down_yaw_target_ = gimbal->down_yaw_move_limiter_.Update(0.002f);
  gimbal->gimbal_up_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_up_yaw_target_,  // 上部yaw轴周期限制
                                                    -static_cast<f32>(M_PI), M_PI);
  gimbal->gimbal_down_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_down_yaw_target_,  // 下部yaw轴周期限制
                                                      -static_cast<f32>(M_PI), M_PI);
  gimbal->percept_move_complete_ = gimbal->up_yaw_move_limiter_.IsAtTarget(0.001f);
  if (gimbal->percept_move_complete_) {
    gimbal->up_yaw_move_limiter_.ResetAt(globals->hipnuc_imu->yaw());
    gimbal->down_yaw_move_limiter_.ResetAt(globals->ahrs.euler_angle().yaw);
    gimbal->perception_time_ = 1000;
  }
  // pitch轴扫描
  if (gimbal->gimbal_pitch_target_ <= gimbal->lowest_aimbot_pitch_angle_) {
    gimbal->scan_pitch_flag_ = false;
  } else if (gimbal->gimbal_pitch_target_ >= gimbal->highest_pitch_angle_) {
    gimbal->scan_pitch_flag_ = true;
  }
  if (gimbal->scan_pitch_flag_) {
    gimbal->gimbal_pitch_target_ -= 0.004f;
  } else {
    gimbal->gimbal_pitch_target_ += 0.004f;
  }
}

void Gimbal::GimbalAimbotTargetUpdate() {
  if ((globals->aimbot_communicator->aimbot_state() >> 0 & 0x01 && globals->StateMachine_ == kTest &&
       globals->wfly_et16s->switch_position(rc_ch::SC) != SwitchPosition::kDown) ||
      (globals->aimbot_communicator->aimbot_state() >> 0 & 0x01 && globals->StateMachine_ == kMatch)) {
    // 自瞄模式：直接使用 NUC 下发的目标速度/加速度
    yaw_speed_ref = globals->aimbot_communicator->yaw_vel();
    pitch_speed_ref = globals->aimbot_communicator->pitch_vel();
    yaw_accel_ref = globals->aimbot_communicator->yaw_acc();
    pitch_accel_ref = globals->aimbot_communicator->pitch_acc();
    if (globals->up_yaw_motor->encoder() >= gimbal->down_yaw_move_high_) {
      gimbal->gimbal_down_yaw_target_ += 0.001;
    } else if (globals->up_yaw_motor->encoder() <= gimbal->down_yaw_move_low_) {
      gimbal->gimbal_down_yaw_target_ -= 0.001;
    }
    auto aimbot_target_yaw = rm::modules::Map(rm::modules::Wrap(globals->aimbot_communicator->yaw(), -180.0f, 180.0f),
                                              0.0f, 360.0f, 0.0f, 2.0f * static_cast<f32>(M_PI));
    if ((globals->up_yaw_motor->encoder() >= gimbal->max_up_yaw_pos_ && globals->up_yaw_motor->encoder() <= 5000 &&
         aimbot_target_yaw >= globals->hipnuc_imu->yaw()) ||
        ((globals->up_yaw_motor->encoder() <= gimbal->min_up_yaw_pos_ || globals->up_yaw_motor->encoder() >= 6000) &&
         aimbot_target_yaw <= globals->hipnuc_imu->yaw())) {
      gimbal->gimbal_up_yaw_target_ = aimbot_target_yaw;
      gimbal->up_yaw_move_limiter_.SetTarget(aimbot_target_yaw);
      gimbal->gimbal_up_yaw_target_ = gimbal->up_yaw_move_limiter_.Update(0.002f);
      gimbal->gimbal_up_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_up_yaw_target_, -static_cast<f32>(M_PI), M_PI);
      if (gimbal->up_yaw_move_limiter_.IsAtTarget(0.001f)) {
        gimbal->up_yaw_move_limiter_.ResetAt(globals->hipnuc_imu->yaw());
      }
      gimbal->gimbal_down_yaw_target_ =
          aimbot_target_yaw - globals->hipnuc_imu->yaw() + globals->ahrs.euler_angle().yaw;
      gimbal->down_yaw_move_limiter_.SetTarget(gimbal->gimbal_down_yaw_target_);
      gimbal->gimbal_down_yaw_target_ = gimbal->down_yaw_move_limiter_.Update(0.002f);
      gimbal->gimbal_down_yaw_target_ =
          rm::modules::Wrap(gimbal->gimbal_down_yaw_target_, -static_cast<f32>(M_PI), M_PI);
      if (gimbal->down_yaw_move_limiter_.IsAtTarget(0.001f)) {
        gimbal->down_yaw_move_limiter_.ResetAt(globals->ahrs.euler_angle().yaw);
      }
    } else {
      gimbal->gimbal_up_yaw_target_ = aimbot_target_yaw;
      gimbal->up_yaw_move_limiter_.ResetAt(globals->hipnuc_imu->yaw());
      gimbal->down_yaw_move_limiter_.ResetAt(globals->ahrs.euler_angle().yaw);
    }
    gimbal->gimbal_pitch_target_ = rm::modules::Wrap(
        rm::modules::Map(globals->aimbot_communicator->pitch(), 0.0f, 360.0f, 0.0f, 2.0f * static_cast<f32>(M_PI)),
        -static_cast<f32>(M_PI), M_PI);
    gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_,  // pitch轴限位
                                                      gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);
    gimbal->aimbot_time_ = 100;
  } else if (gimbal->aimbot_time_ > 0) {
    gimbal->aimbot_time_--;
  } else {
    gimbal->GimbalRCTargetUpdate();
  }
}

void Gimbal::GimbalMovePIDUpdate() {
  globals->gimbal_controller.SetTarget(gimbal->gimbal_up_yaw_target_, gimbal->gimbal_down_yaw_target_,
                                       gimbal->gimbal_pitch_target_, gimbal->yaw_speed_ff_);
  globals->gimbal_controller.Update(globals->hipnuc_imu->yaw(), -globals->hipnuc_imu->gyro_z(),
                                    globals->ahrs.euler_angle().yaw, globals->imu->gyro_z(),
                                    globals->hipnuc_imu->pitch(), -globals->hipnuc_imu->gyro_x(), 2.0f);
  const Eigen::Vector3f g_stationary(0.0f, 0.0f, -9.81f);
  const auto ff =
      g_gimbal_dynamics.ComputeFf(gimbal->gimbal_up_yaw_target_, gimbal->gimbal_pitch_target_, yaw_speed_ref,
                                  pitch_speed_ref, yaw_accel_ref, pitch_accel_ref, g_stationary);
  gimbal->yaw_torque_ = ff.x();
  const f32 yaw_ff_voltage =
      YawTorqueToVoltageCmd(gimbal->yaw_torque_, static_cast<f32>(globals->up_yaw_motor->rpm()) * kRpmToRadPerSec);
  gimbal->up_yaw_current_ = globals->gimbal_controller.output().up_yaw +
                            static_cast<f32>(globals->up_yaw_motor->rpm()) * 100.f + yaw_ff_voltage;
  gimbal->up_yaw_current_ =
      rm::modules::Clamp(gimbal->up_yaw_current_, -kGm6020VoltageCmdLimit, kGm6020VoltageCmdLimit);
  gimbal->pitch_torque_ = globals->gimbal_controller.output().pitch + ff.y();
  gimbal->pitch_torque_ = rm::modules::Clamp(gimbal->pitch_torque_, -10.f, 10.f);
}

void Gimbal::ApplyNormalGimbalPID() {
  // 上部 Yaw PID 参数
  globals->gimbal_controller.pid().up_yaw_position.SetKp(20.f).SetKi(0).SetKd(120.f).SetMaxOut(20000.f).SetMaxIout(0);
  // pitch PID 参数
  globals->gimbal_controller.pid().pitch_position.SetKp(70.f).SetKi(0).SetKd(0.f).SetMaxOut(30.f).SetMaxIout(0);
}

void Gimbal::GimbalIdentifyUpdate() {
  // pid 更新
  globals->gimbal_controller.pid().up_yaw_position.SetKp(300.f).SetKi(0).SetKd(800.f).SetMaxOut(25000.f).SetMaxIout(0);
  globals->gimbal_controller.pid().pitch_position.SetKp(120.f).SetKi(0).SetKd(5000.f).SetMaxOut(10.f).SetMaxIout(0);
  globals->gimbal_controller.EnableSpeedPid(false);
  // 标定参数更新
  gimbal->identify_yaw_position_ = globals->up_yaw_motor->encoder();
  gimbal->identify_yaw_speed_ = static_cast<f32>(globals->up_yaw_motor->rpm()) * kRpmToRadPerSec;
  gimbal->identify_pitch_position_ = -globals->pitch_motor->pos();
  gimbal->identify_pitch_speed_ = -globals->pitch_motor->vel();
  // 目标值更新
  const auto yaw = EvaluateIdentifyTrajectory(kIdentifyYawCenter, kIdentifyYawAmp, gimbal->identify_time_s_);
  const auto pitch = EvaluateIdentifyTrajectory(kIdentifyPitchCenter, kIdentifyPitchAmp, gimbal->identify_time_s_);
  gimbal->gimbal_up_yaw_target_ = rm::modules::Clamp(yaw.q, kIdentifyYawBottomLimit, kIdentifyYawTopLimit);
  gimbal->gimbal_pitch_target_ = -rm::modules::Clamp(pitch.q, kIdentifyPitchBottomLimit, kIdentifyPitchTopLimit);
  // pid计算
  globals->gimbal_controller.SetTarget(gimbal->gimbal_up_yaw_target_, gimbal->gimbal_down_yaw_target_,
                                       gimbal->gimbal_pitch_target_);
  globals->gimbal_controller.Update(gimbal->identify_yaw_position_, 0.0f, gimbal->gimbal_down_yaw_target_,
                                    globals->imu->gyro_z(), gimbal->identify_pitch_position_, 0.0f);
  // 限幅及转化
  gimbal->up_yaw_current_ =
      rm::modules::Clamp(globals->gimbal_controller.output().up_yaw, -kGm6020VoltageCmdLimit, kGm6020VoltageCmdLimit);
  gimbal->yaw_torque_ = YawVoltageCmdToTorque(gimbal->up_yaw_current_, gimbal->identify_yaw_speed_);
  gimbal->pitch_torque_ = rm::modules::Clamp(globals->gimbal_controller.output().pitch, -10.0f, 10.0f);
  gimbal->identify_time_s_ += gimbal->Ts;
}

void Gimbal::GimbalFfVerifyUpdate() {
  // 重力补偿验证：dq/ddq 置零，只用实际 pitch 位置计算重力项
  const Eigen::Vector3f g_stationary(0.0f, 0.0f, -9.81f);
  const auto ff =
      g_gimbal_dynamics.ComputeFfDecomposed(0, globals->hipnuc_imu->pitch() + 0.38f, 0, 0, 0, 0, g_stationary);
  gimbal->yaw_torque_ = ff.yaw;
  gimbal->up_yaw_current_ = YawTorqueToVoltageCmd(gimbal->yaw_torque_, gimbal->identify_yaw_speed_);
  gimbal->pitch_torque_ = rm::modules::Clamp(ff.pitch, -10.0f, 10.0f);
}

void Gimbal::GimbalMatchUpdate() {
  if (globals->aimbot_communicator->aimbot_state() >> 0 & 0x01 || gimbal->aimbot_time_ > 0) {
    gimbal->GimbalMove_ = kGbAimbot;
    gimbal->percept_move_complete_ = true;
    gimbal->perception_time_ = 0;
  } else if ((globals->navigate_communicator->perception_flag() != 0x00 || !gimbal->percept_move_complete_) &&
             !globals->navigate_communicator->aimbot_mode() && !globals->navigate_communicator->outpost_mode() &&
             gimbal->perception_time_ <= 0) {
    gimbal->GimbalMove_ = kGbPercept;
  } else if (globals->navigate_communicator->scan_mode()) {
    gimbal->GimbalMove_ = kGbScan;
  } else {
    gimbal->GimbalMove_ = kGbNavigate;
  }
  gimbal->GimbalEnableUpdate();
}

void Gimbal::GimbalEnableUpdate() {
  globals->gimbal_controller.Enable(true);
  if (gimbal->GimbalMove_ != kGbIdentify) {
    gimbal->ApplyNormalGimbalPID();
    globals->gimbal_controller.EnableSpeedPid(true);
  }
  if (gimbal->GimbalMove_ == kGbRemote) {
    gimbal->GimbalRCTargetUpdate();
    gimbal->GimbalMovePIDUpdate();
  } else if (gimbal->GimbalMove_ == kGbAimbot) {
    gimbal->GimbalAimbotTargetUpdate();
    gimbal->GimbalMovePIDUpdate();
  } else if (gimbal->GimbalMove_ == kGbPercept) {
    gimbal->GimbalPerceptTargetUpdate();
    gimbal->GimbalMovePIDUpdate();
  } else if (gimbal->GimbalMove_ == kGbScan || gimbal->GimbalMove_ == kGbNavigate) {
    gimbal->GimbalScanTargetUpdate();
    gimbal->GimbalMovePIDUpdate();
  } else if (gimbal->GimbalMove_ == kGbIdentify) {
    gimbal->GimbalIdentifyUpdate();
  } else if (gimbal->GimbalMove_ == kGbFfVerify) {
    gimbal->GimbalFfVerifyUpdate();
  } else {
    globals->gimbal_controller.Enable(false);
    gimbal->up_yaw_current_ = 0.f;
    gimbal->yaw_torque_ = 0.f;
    gimbal->pitch_torque_ = 0.f;
  }
  if (globals->StateMachine_ == kMatch && globals->navigate_communicator->outpost_mode()) {
    globals->aim_mode = 0x04;
  } else if (globals->StateMachine_ == kMatch && globals->navigate_communicator->aimbot_mode()) {
    if (globals->referee_data->data().game_status.game_progress == 4 &&
        globals->referee_data->data().game_status.stage_remain_time <= 240) {
      globals->aim_mode = 0x03;
    } else {
      globals->aim_mode = 0x02;
    }
  } else if (globals->StateMachine_ == kTest) {
    if (globals->wfly_et16s->switch_position(rc_ch::SB) == SwitchPosition::kMid) {
      globals->aim_mode = 0x02;
    } else if (globals->wfly_et16s->switch_position(rc_ch::SB) == SwitchPosition::kUp) {
      globals->aim_mode = 0x03;
    } else if (globals->wfly_et16s->switch_position(rc_ch::SC) == SwitchPosition::kUp) {
      globals->aim_mode = 0x04;
    } else {
      globals->aim_mode = 0x01;
    }
  } else {
    globals->aim_mode = 0x01;
  }
  gimbal->DaMiaoMotorEnable();
  gimbal->SetMotorCurrent();
}

void Gimbal::GimbalDisableUpdate() {
  globals->gimbal_controller.Enable(false);
  globals->aim_mode = 0x01;
  gimbal->gimbal_up_yaw_target_ = globals->hipnuc_imu->yaw();
  gimbal->gimbal_down_yaw_target_ = globals->ahrs.euler_angle().yaw;
  gimbal->gimbal_pitch_target_ = globals->hipnuc_imu->pitch();
  gimbal->up_yaw_move_limiter_.ResetAt(globals->hipnuc_imu->yaw());
  gimbal->down_yaw_move_limiter_.ResetAt(globals->ahrs.euler_angle().yaw);
  gimbal->GimbalMovePIDUpdate();
  gimbal->DaMiaoMotorDisable();
  gimbal->up_yaw_current_ = 0.f;
  gimbal->yaw_torque_ = 0.f;
  gimbal->pitch_torque_ = 0.0f;
  gimbal->SetMotorCurrent();
}

void Gimbal::DaMiaoMotorEnable() {
  if (globals->down_yaw_motor->status() != 0x1F && globals->down_yaw_motor->status() != 0x0F) {
    globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  } else if (globals->pitch_motor->status() != 0x1F && globals->pitch_motor->status() != 0x0F) {
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  } else {
    if (globals->down_yaw_motor->status() == 0x0F) {
      globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    }
    if (globals->pitch_motor->status() == 0x0F) {
      globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    }
  }
}

void Gimbal::DaMiaoMotorDisable() {
  if (globals->down_yaw_motor->status() != 0x1F && globals->down_yaw_motor->status() != 0x0F) {
    globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  } else if (globals->pitch_motor->status() != 0x1F && globals->pitch_motor->status() != 0x0F) {
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  } else {
    if (globals->down_yaw_motor->status() == 0x1F) {
      globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    }
    if (globals->pitch_motor->status() == 0x1F) {
      globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    }
  }
}

void Gimbal::ShootEnableUpdate() {
  globals->shoot_controller.Enable(true);
  globals->shoot_controller.Arm(true);
  globals->shoot_controller.SetArmSpeed(gimbal->ammo_speed_);
  globals->dail_encoder_counter.Update(globals->dial_motor->encoder());
  if (globals->referee_data->data().shoot_data.initial_speed >= 22.0f ||
      (globals->referee_data->data().shoot_data.initial_speed >= 15.0f &&
       globals->referee_data->data().shoot_data.initial_speed <= 21.0f)) {
    gimbal->ammo_speed_ = 6200.0f * std::sqrt(22.0f / globals->referee_data->data().shoot_data.initial_speed);
  }
  if (((globals->wfly_et16s->wheel_position(rc_ch::LS) <= -650 &&
        globals->wfly_et16s->switch_position(rc_ch::SH) == SwitchPosition::kDown) ||  // 手动强制单发
       ((globals->wfly_et16s->switch_position(rc_ch::SB) == SwitchPosition::kMid ||
         globals->wfly_et16s->switch_position(rc_ch::SB) == SwitchPosition::kUp) &&
        globals->wfly_et16s->wheel_position(rc_ch::LS) >= 650 && globals->StateMachine_ == kTest &&
        globals->aimbot_communicator->aimbot_state() >> 1 & 0x01) ||  // 测试模式自动开火
       (globals->navigate_communicator->aimbot_mode() && globals->StateMachine_ == kMatch &&
        globals->aimbot_communicator->aimbot_state() >> 1 & 0x01)) &&
      heat_limit_ - heat_current_ > 30) {
    if (!single_shoot_flag_) {
      globals->shoot_controller.SetMode(Shoot3Fric::kSingleShot);
      globals->shoot_controller.Fire();
      single_shoot_flag_ = true;
      gimbal->single_shoot_time_ = 200;
    } else {
      globals->shoot_controller.SetShootFrequency(0.0f);
    }
  } else if (globals->wfly_et16s->switch_position(rc_ch::SH) == SwitchPosition::kDown ||  // 手动强制连发
             (globals->wfly_et16s->switch_position(rc_ch::SB) == SwitchPosition::kDown &&
              globals->wfly_et16s->wheel_position(rc_ch::LS) >= 650 && globals->StateMachine_ == kTest &&
              globals->aimbot_communicator->aimbot_state() >> 1 & 0x01) ||
             (!globals->navigate_communicator->aimbot_mode() && globals->StateMachine_ == kMatch &&
              globals->aimbot_communicator->aimbot_state() >> 1 & 0x01)) {
    globals->shoot_controller.SetMode(Shoot3Fric::kFullAuto);
    if (heat_limit_ - heat_current_ > 100) {
      globals->shoot_controller.SetShootFrequency(20.0f);
    } else if (heat_limit_ - heat_current_ < 20) {
      globals->shoot_controller.SetShootFrequency(0.0f);
    } else {
      globals->shoot_controller.SetShootFrequency(static_cast<f32>(heat_limit_ - heat_current_) / 6.0f + 5.0f);
    }
  } else {
    globals->shoot_controller.SetShootFrequency(0.0f);
  }
  if (gimbal->single_shoot_time_ > 0) {
    gimbal->single_shoot_time_--;
  } else if (gimbal->single_shoot_time_ == 0) {
    single_shoot_flag_ = false;
  }
  globals->shoot_controller.Update(globals->friction_left->rpm(), globals->friction_right->rpm(), 0,
                                   static_cast<f32>(globals->dail_encoder_counter.linear_ticks()),
                                   globals->dial_motor->rpm());
}

void Gimbal::ShootDisableUpdate() {
  globals->shoot_controller.SetMode(Shoot3Fric::kStop);
  if (!globals->referee_data->data().robot_status.power_management_shooter_output) {
    globals->shoot_controller.Enable(false);
    globals->shoot_controller.Arm(false);
  } else {
    globals->shoot_controller.Enable(true);
    globals->shoot_controller.Arm(true);
    globals->shoot_controller.SetArmSpeed(0.0f);
    globals->shoot_controller.SetShootFrequency(0.0f);
  }
  globals->dail_encoder_counter.Reset(0, globals->dial_motor->encoder());
  globals->shoot_controller.Update(globals->friction_left->rpm(), globals->friction_right->rpm(), 0,
                                   static_cast<f32>(globals->dail_encoder_counter.linear_ticks()),
                                   globals->dial_motor->rpm());
}

void Gimbal::GimbalIdentifyDataSend() {
  if (globals == nullptr || globals->ident_uart == nullptr || globals->StateMachine_ != kTest ||
      gimbal->GimbalMove_ != kGbIdentify) {
    return;
  }

  char tx_buf[128]{};
  int len =
      std::snprintf(tx_buf, sizeof(tx_buf), "%lu,", static_cast<unsigned long>(gimbal->identify_time_s_ * 1000.0f));
  len += AppendFloat(tx_buf + len, sizeof(tx_buf) - len, gimbal->yaw_torque_);
  len += std::snprintf(tx_buf + len, sizeof(tx_buf) - len, ",");
  len += AppendFloat(tx_buf + len, sizeof(tx_buf) - len, gimbal->identify_yaw_position_);
  len += std::snprintf(tx_buf + len, sizeof(tx_buf) - len, ",");
  len += AppendFloat(tx_buf + len, sizeof(tx_buf) - len, gimbal->identify_yaw_speed_);
  len += std::snprintf(tx_buf + len, sizeof(tx_buf) - len, ",");
  len += AppendFloat(tx_buf + len, sizeof(tx_buf) - len, -gimbal->pitch_torque_);
  len += std::snprintf(tx_buf + len, sizeof(tx_buf) - len, ",");
  len += AppendFloat(tx_buf + len, sizeof(tx_buf) - len, gimbal->identify_pitch_position_);
  len += std::snprintf(tx_buf + len, sizeof(tx_buf) - len, ",");
  len += AppendFloat(tx_buf + len, sizeof(tx_buf) - len, gimbal->identify_pitch_speed_);
  len += std::snprintf(tx_buf + len, sizeof(tx_buf) - len, "\r\n");
  if (len <= 0) {
    return;
  }
  if (static_cast<size_t>(len) >= sizeof(tx_buf)) {
    len = sizeof(tx_buf) - 1;
  }
  globals->ident_uart->Write(reinterpret_cast<const u8 *>(tx_buf), static_cast<usize>(len));
}

void Gimbal::SetMotorCurrent() {
  globals->up_yaw_motor->SetCurrent(static_cast<i16>(gimbal->up_yaw_current_));
  // globals->up_yaw_motor->SetCurrent(0);
  globals->friction_left->SetCurrent(static_cast<i16>(globals->shoot_controller.output().fric_1));
  globals->friction_right->SetCurrent(static_cast<i16>(globals->shoot_controller.output().fric_2));
  globals->dial_motor->SetCurrent(static_cast<i16>(globals->shoot_controller.output().loader));
}