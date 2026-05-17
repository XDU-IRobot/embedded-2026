#include "Gimbal.hpp"

#include <cstdio>

#include "gimbal-tool-suite/dynamics/dynamics.hpp"

f32 a, b, c, d;

extern "C" {
volatile f32 fm_ident_yaw_target = 0.0f;
volatile f32 fm_ident_pitch_target = 0.0f;
volatile f32 fm_ident_yaw_position = 0.0f;
volatile f32 fm_ident_pitch_position = 0.0f;
volatile f32 fm_ident_yaw_current = 0.0f;
volatile f32 fm_ident_pitch_torque = 0.0f;
volatile f32 fm_aimbot_state = 0.0f;
volatile f32 fm_aimbot_target = 0.0f;
volatile f32 fm_aimbot_yaw = 0.0f;
volatile f32 fm_aimbot_pitch = 0.0f;
volatile f32 fm_aimbot_nuc_start_flag = 0.0f;
volatile f32 fm_aimbot_yaw_vel = 0.0f;
volatile f32 fm_aimbot_pitch_vel = 0.0f;
volatile f32 fm_aimbot_yaw_acc = 0.0f;
volatile f32 fm_aimbot_pitch_acc = 0.0f;
volatile f32 fm_gimbal_yaw = 0.0f;
volatile f32 fm_gimbal_pitch = 0.0f;
volatile f32 fm_ff_yaw_torque = 0.0f;
volatile f32 fm_ff_pitch_torque = 0.0f;
volatile f32 fm_pid_yaw = 0.0f;
volatile f32 fm_pid_pitch = 0.0f;
volatile f32 fm_ff_yaw_voltage = 0.0f;
}

namespace {
constexpr size_t kIdentifyHarmonicCount = 5;
constexpr f32 kIdentifyBaseFreqHz = 0.1f;
constexpr f32 kEncoderTicksPerRev = 8192.0f;
constexpr f32 kRpmToRadPerSec = static_cast<f32>(M_PI) * 2.0f / 60.0f;
constexpr f32 kIdentifyPitchTopLimit = -1.4521f;
constexpr f32 kIdentifyPitchBottomLimit = -0.3057f;
constexpr f32 kIdentifyPitchCenter = (kIdentifyPitchTopLimit + kIdentifyPitchBottomLimit) * 0.5f;
constexpr f32 kIdentifyYawAmp[kIdentifyHarmonicCount] = {3.5f, -2.0f, 1.2f, -0.8f, 0.5f};
constexpr f32 kIdentifyPitchAmp[kIdentifyHarmonicCount] = {0.34f, -0.18f, 0.11f, -0.07f, 0.04f};
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
bool InitDynamicsTheta() {
  Eigen::Matrix<float, 9, 1> theta;
  theta << 0.11313911f, 0.12711330f, 0.02701958f, 0.07856400f, 0.03824676f, 0.00151766f, 0.70682046f, 0.35594090f,
      0.03705741f;
  g_gimbal_dynamics.SetTheta(theta);
  return true;
}
const bool g_dynamics_initialized = InitDynamicsTheta();

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

f32 PitchRawToIdentifyModel(f32 raw_pitch) { return raw_pitch - kIdentifyPitchCenter; }

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
  gimbal->gimbal_yaw_target_ = globals->ahrs.euler_angle().yaw;
  gimbal->gimbal_pitch_target_ = globals->ahrs.euler_angle().pitch;
  gimbal->last_yaw_target = gimbal->gimbal_yaw_target_;
  gimbal->last_pitch_target_ = gimbal->gimbal_pitch_target_;
  globals->dail_encoder_counter.Reset(0, globals->dial_motor->encoder());
}

void Gimbal::GimbalTask() {
  gimbal->GimbalStateUpdate();
  a = gimbal->gimbal_yaw_target_;
  b = gimbal->gimbal_pitch_target_;
  c = globals->ahrs.euler_angle().yaw;
  d = globals->ahrs.euler_angle().pitch;
}

void Gimbal::GimbalStateUpdate() {
  if (!globals->device_gimbal.all_device_ok() || !globals->chassis_communicator->gimbal_power_state()) {
    globals->StateMachine_ = kUnable;  // 如果云台设备离线或云台供电异常，进入无力模式
    gimbal->GimbalDisableUpdate();     // 云台电机失能计算
  } else {
    switch (globals->StateMachine_) {
      case kNoForce:                    // 无力模式下，所有电机失能
        gimbal->GimbalDisableUpdate();  // 云台电机失能计算
        break;

      case kTest:                      // 测试模式下，发射系统与拨盘电机失能
        gimbal->GimbalEnableUpdate();  // 云台电机使能计算
        break;

      case kMatch:
        gimbal->GimbalMatchUpdate();
        break;

      default:                          // 错误状态，所有电机失能
        gimbal->GimbalDisableUpdate();  // 云台电机失能计算
        break;
    }
  }
  if (!globals->device_shoot.all_device_ok() || !globals->chassis_communicator->ammo_power_state()) {
    gimbal->ShootDisableUpdate();  // 发射机构失能计算
  } else {
    switch (globals->StateMachine_) {
      case kMatch:
        gimbal->ShootEnableUpdate();  // 发射机构使能计算
        break;
      case kTest:  // 测试模式下，发射系统与拨盘电机失能
        switch (gimbal->GimbalMove_) {
          case kGbAimbot:
          case kGbAimbotFu:
            gimbal->ShootEnableUpdate();  // 发射机构使能计算
            break;
          case kGbIdentify:
            gimbal->ShootIdentifyUpdate();
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
  gimbal->gimbal_yaw_target_ -= rm::modules::Map(
      static_cast<f32>(globals->rc->left_x()) +
          30.0f * static_cast<f32>(globals->image_update_flag ? globals->image_data->data().mouse_x
                                                              : globals->rc->mouse_x()),  // 上部yaw轴目标值
      -660, 660, -gimbal->sensitivity_yaw_, gimbal->sensitivity_yaw_);
  gimbal->gimbal_pitch_target_ -= rm::modules::Map(
      static_cast<f32>(globals->rc->left_y()) +
          30.0f * static_cast<f32>(globals->image_update_flag ? globals->image_data->data().mouse_y
                                                              : globals->rc->mouse_y()),  // pitch轴目标值
      -660, 660, -gimbal->sensitivity_pitch_, gimbal->sensitivity_pitch_);
  gimbal->gimbal_yaw_target_ =
      rm::modules::Wrap(gimbal->gimbal_yaw_target_, -static_cast<f32>(M_PI), M_PI);  // yaw轴限位
  gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_,    // pitch轴限位
                                                    gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);
}

void Gimbal::GimbalAimbotTargetUpdate() {
  if ((globals->StateMachine_ == kTest && globals->aimbot_communicator->aimbot_state() >> 0 & 0x01) ||
      (globals->StateMachine_ == kMatch && (globals->image_update_flag ? globals->image_data->data().mouse_button_right
                                                                       : globals->rc->mouse_button_right()))) {
    gimbal->gimbal_yaw_target_ = globals->aimbot_communicator->yaw();
    gimbal->gimbal_pitch_target_ = globals->aimbot_communicator->pitch();
  } else {
    gimbal->GimbalRCTargetUpdate();
  }
  gimbal->gimbal_yaw_target_ =
      rm::modules::Wrap(gimbal->gimbal_yaw_target_, -static_cast<f32>(M_PI), M_PI);  // yaw轴限位
  gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_,    // pitch轴限位
                                                    gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);
}

void Gimbal::GimbalMovePIDUpdate() {
  if (!gimbal->move_ff_initialized_) {
    gimbal->last_yaw_target = gimbal->gimbal_yaw_target_;
    gimbal->last_pitch_target_ = gimbal->gimbal_pitch_target_;
    gimbal->last_yaw_speed_ref_ = 0.0f;
    gimbal->last_pitch_speed_ref_ = 0.0f;
    gimbal->move_ff_initialized_ = true;
  }

  fm_aimbot_state = static_cast<f32>(globals->aimbot_communicator->aimbot_state());
  fm_aimbot_target = static_cast<f32>(globals->aimbot_communicator->aimbot_target());
  fm_aimbot_yaw = globals->aimbot_communicator->yaw();
  fm_aimbot_pitch = globals->aimbot_communicator->pitch();
  fm_aimbot_nuc_start_flag = static_cast<f32>(globals->aimbot_communicator->nuc_start_flag());

  fm_gimbal_yaw = globals->ahrs.euler_angle().yaw;
  fm_gimbal_pitch = globals->ahrs.euler_angle().pitch;

  f32 yaw_speed_ref;
  f32 pitch_speed_ref;
  f32 yaw_accel_ref;
  f32 pitch_accel_ref;

  if ((gimbal->GimbalMove_ == kGbAimbot || gimbal->GimbalMove_ == kGbAimbotFu) &&
      globals->aimbot_communicator->aimbot_state() >> 0 & 0x01) {
    // 自瞄模式：直接使用 NUC 下发的目标速度/加速度
    yaw_speed_ref = globals->aimbot_communicator->yaw_vel();
    pitch_speed_ref = globals->aimbot_communicator->pitch_vel();
    yaw_accel_ref = globals->aimbot_communicator->yaw_acc();
    pitch_accel_ref = globals->aimbot_communicator->pitch_acc();
    fm_aimbot_yaw_vel = yaw_speed_ref;
    fm_aimbot_pitch_vel = pitch_speed_ref;
    fm_aimbot_yaw_acc = yaw_accel_ref;
    fm_aimbot_pitch_acc = pitch_accel_ref;
  } else {
    // 遥控模式：从位置目标差分得到速度/加速度
    const f32 yaw_delta = rm::modules::Wrap(gimbal->gimbal_yaw_target_ - gimbal->last_yaw_target,
                                            -static_cast<f32>(M_PI), static_cast<f32>(M_PI));
    yaw_speed_ref = rm::modules::Clamp(yaw_delta / gimbal->Ts, -kNormalFfMaxYawSpeed, kNormalFfMaxYawSpeed);
    pitch_speed_ref = rm::modules::Clamp((gimbal->gimbal_pitch_target_ - gimbal->last_pitch_target_) / gimbal->Ts,
                                         -kNormalFfMaxPitchSpeed, kNormalFfMaxPitchSpeed);
    yaw_accel_ref = rm::modules::Clamp((yaw_speed_ref - gimbal->last_yaw_speed_ref_) / gimbal->Ts,
                                       -kNormalFfMaxYawAccel, kNormalFfMaxYawAccel);
    pitch_accel_ref = rm::modules::Clamp((pitch_speed_ref - gimbal->last_pitch_speed_ref_) / gimbal->Ts,
                                         -kNormalFfMaxPitchAccel, kNormalFfMaxPitchAccel);
  }

  gimbal->yaw_speed_ff = gimbal->Kf * yaw_speed_ref;
  gimbal->last_yaw_target = gimbal->gimbal_yaw_target_;
  gimbal->last_pitch_target_ = gimbal->gimbal_pitch_target_;
  gimbal->last_yaw_speed_ref_ = yaw_speed_ref;
  gimbal->last_pitch_speed_ref_ = pitch_speed_ref;

  globals->gimbal_controller.SetTarget(gimbal->gimbal_yaw_target_, gimbal->gimbal_pitch_target_, gimbal->yaw_speed_ff);
  globals->gimbal_controller.Update(globals->ahrs.euler_angle().yaw, globals->imu->gyro_z(),
                                    globals->ahrs.euler_angle().pitch, globals->imu->gyro_x());
  const Eigen::Vector3f g_stationary(0.0f, 0.0f, -9.81f);
  const auto ff = g_gimbal_dynamics.ComputeFf(gimbal->gimbal_yaw_target_, gimbal->gimbal_pitch_target_, yaw_speed_ref,
                                              pitch_speed_ref, yaw_accel_ref, pitch_accel_ref, g_stationary);
  gimbal->yaw_torque_ = ff.x();
  fm_ff_yaw_torque = ff.x();
  fm_ff_pitch_torque = ff.y();
  fm_pid_yaw = globals->gimbal_controller.output().yaw;
  fm_pid_pitch = globals->gimbal_controller.output().pitch;
  const f32 yaw_ff_voltage =
      YawTorqueToVoltageCmd(gimbal->yaw_torque_, static_cast<f32>(globals->yaw_motor->rpm()) * kRpmToRadPerSec);
  fm_ff_yaw_voltage = yaw_ff_voltage;
  gimbal->yaw_current_ =
      globals->gimbal_controller.output().yaw + static_cast<f32>(globals->yaw_motor->rpm()) * 100.f + yaw_ff_voltage;
  gimbal->yaw_current_ = rm::modules::Clamp(gimbal->yaw_current_, -kGm6020VoltageCmdLimit, kGm6020VoltageCmdLimit);
  gimbal->pitch_torque_ = globals->gimbal_controller.output().pitch + ff.y();
  gimbal->pitch_torque_ = rm::modules::Clamp(gimbal->pitch_torque_, -10.f, 10.f);
}

void Gimbal::ApplyNormalGimbalPID() {
  globals->gimbal_controller.pid()
      .yaw_position.SetKp(400.0f)
      .SetKi(0.0f)
      .SetKd(10000.0f)
      .SetMaxOut(30000.0f)
      .SetMaxIout(0.0f);
  globals->gimbal_controller.pid().pitch_position.SetKp(45.0f).SetKi(0.0f).SetKd(800.0f).SetMaxOut(10000.0f).SetMaxIout(
      0.0f);
  // globals->gimbal_controller.pid().yaw_position.SetKp(400.0f).SetKi(0.0f).SetKd(10000.0f).SetMaxOut(0.0f).SetMaxIout(0.0f);
  // globals->gimbal_controller.pid().pitch_position.SetKp(20.0f).SetKi(0.0f).SetKd(500.0f).SetMaxOut(0.0f).SetMaxIout(0.0f);
}

void Gimbal::ApplyIdentifyGimbalPID() {
  globals->gimbal_controller.pid()
      .yaw_position.SetKp(400000.0f)
      .SetKi(0.0f)
      .SetKd(100000.0f)
      .SetMaxOut(30000.0f)
      .SetMaxIout(0.0f);
  globals->gimbal_controller.pid().pitch_position.SetKp(20.0f).SetKi(0.0f).SetKd(50.f).SetMaxOut(10.0f).SetMaxIout(
      0.0f);
}

void Gimbal::GimbalIdentifyUpdate() {
  gimbal->ApplyIdentifyGimbalPID();
  globals->gimbal_controller.EnableSpeedPid(false);
  if (!gimbal->identify_active_) {
    gimbal->identify_yaw_encoder_counter_.Reset(0, globals->yaw_motor->encoder());
    gimbal->identify_yaw_encoder_counter_.Update(globals->yaw_motor->encoder());
    gimbal->identify_active_ = true;
    gimbal->identify_time_s_ = 0.0f;
    gimbal->identify_yaw_center_ = 0.0f;
    gimbal->identify_pitch_center_ = kIdentifyPitchCenter;
    gimbal->identify_yaw_position_ = 0.0f;
    gimbal->identify_yaw_speed_ = 0.0f;
    gimbal->identify_pitch_position_ = globals->pitch_motor->pos();
    gimbal->identify_pitch_speed_ = globals->pitch_motor->vel();
  }

  gimbal->identify_yaw_encoder_counter_.Update(globals->yaw_motor->encoder());
  gimbal->identify_yaw_position_ = static_cast<f32>(gimbal->identify_yaw_encoder_counter_.linear_ticks()) /
                                   kEncoderTicksPerRev * 2.0f * static_cast<f32>(M_PI);
  gimbal->identify_yaw_speed_ = static_cast<f32>(globals->yaw_motor->rpm()) * kRpmToRadPerSec;
  gimbal->identify_pitch_position_ = globals->pitch_motor->pos();
  gimbal->identify_pitch_speed_ = globals->pitch_motor->vel();

  gimbal->GimbalIdentifyTargetUpdate();
  gimbal->GimbalIdentifyPIDUpdate();
}

void Gimbal::GimbalIdentifyTargetUpdate() {
  const auto yaw = EvaluateIdentifyTrajectory(gimbal->identify_yaw_center_, kIdentifyYawAmp, gimbal->identify_time_s_);
  const auto pitch =
      EvaluateIdentifyTrajectory(gimbal->identify_pitch_center_, kIdentifyPitchAmp, gimbal->identify_time_s_);

  gimbal->gimbal_yaw_target_ = yaw.q;
  gimbal->gimbal_pitch_target_ = rm::modules::Clamp(pitch.q, kIdentifyPitchTopLimit, kIdentifyPitchBottomLimit);
  gimbal->identify_time_s_ += gimbal->Ts;
}

void Gimbal::GimbalIdentifyPIDUpdate() {
  fm_ident_yaw_target = gimbal->gimbal_yaw_target_;
  fm_ident_pitch_target = PitchRawToIdentifyModel(gimbal->gimbal_pitch_target_);
  fm_ident_yaw_position = gimbal->identify_yaw_position_;
  fm_ident_pitch_position = PitchRawToIdentifyModel(gimbal->identify_pitch_position_);

  globals->gimbal_controller.SetTarget(gimbal->gimbal_yaw_target_, gimbal->gimbal_pitch_target_);
  globals->gimbal_controller.Update(gimbal->identify_yaw_position_, 0.0f, gimbal->identify_pitch_position_, 0.0f);
  gimbal->yaw_current_ =
      rm::modules::Clamp(globals->gimbal_controller.output().yaw, -kGm6020VoltageCmdLimit, kGm6020VoltageCmdLimit);
  gimbal->yaw_torque_ = YawVoltageCmdToTorque(gimbal->yaw_current_, gimbal->identify_yaw_speed_);
  gimbal->pitch_torque_ = rm::modules::Clamp(globals->gimbal_controller.output().pitch, -10.0f, 10.0f);
  fm_ident_yaw_current = gimbal->yaw_current_;
  fm_ident_pitch_torque = gimbal->pitch_torque_;
}

void Gimbal::GimbalFfVerifyUpdate() {
  globals->gimbal_controller.Enable(false);
  globals->gimbal_controller.EnableSpeedPid(false);

  if (!gimbal->ff_verify_active_) {
    gimbal->identify_yaw_encoder_counter_.Reset(0, globals->yaw_motor->encoder());
    gimbal->identify_yaw_encoder_counter_.Update(globals->yaw_motor->encoder());
    gimbal->ff_verify_active_ = true;
    gimbal->ff_verify_time_s_ = 0.0f;
    gimbal->identify_yaw_position_ = 0.0f;
    gimbal->identify_yaw_speed_ = 0.0f;
    gimbal->identify_pitch_position_ = globals->pitch_motor->pos();
    gimbal->identify_pitch_speed_ = globals->pitch_motor->vel();
  }

  gimbal->identify_yaw_encoder_counter_.Update(globals->yaw_motor->encoder());
  gimbal->identify_yaw_position_ = static_cast<f32>(gimbal->identify_yaw_encoder_counter_.linear_ticks()) /
                                   kEncoderTicksPerRev * 2.0f * static_cast<f32>(M_PI);
  gimbal->identify_yaw_speed_ = static_cast<f32>(globals->yaw_motor->rpm()) * kRpmToRadPerSec;
  gimbal->identify_pitch_position_ = globals->pitch_motor->pos();
  gimbal->identify_pitch_speed_ = globals->pitch_motor->vel();

  const auto yaw = EvaluateIdentifyTrajectory(0.0f, kIdentifyYawAmp, gimbal->ff_verify_time_s_);
  const auto pitch = EvaluateIdentifyTrajectory(0.0f, kIdentifyPitchAmp, gimbal->ff_verify_time_s_);
  gimbal->gimbal_yaw_target_ = yaw.q;
  gimbal->gimbal_pitch_target_ =
      rm::modules::Clamp(pitch.q + kIdentifyPitchCenter, kIdentifyPitchTopLimit, kIdentifyPitchBottomLimit);

  // 重力补偿验证：dq/ddq 置零，只用实际 pitch 位置计算重力项
  const Eigen::Vector3f g_stationary(0.0f, 0.0f, -9.81f);
  const auto ff =
      g_gimbal_dynamics.ComputeFfDecomposed(yaw.q, pitch.q, yaw.dq, pitch.dq, yaw.ddq, pitch.ddq, g_stationary);

  gimbal->yaw_torque_ = ff.yaw;
  gimbal->yaw_current_ = YawTorqueToVoltageCmd(gimbal->yaw_torque_, gimbal->identify_yaw_speed_);
  gimbal->pitch_torque_ = rm::modules::Clamp(ff.pitch, -10.0f, 10.0f);

  fm_ident_yaw_target = yaw.q;
  fm_ident_pitch_target = pitch.q;
  fm_ident_yaw_position = gimbal->identify_yaw_position_;
  fm_ident_pitch_position = PitchRawToIdentifyModel(gimbal->identify_pitch_position_);
  fm_ident_yaw_current = gimbal->yaw_current_;
  fm_ident_pitch_torque = gimbal->pitch_torque_;

  gimbal->ff_verify_time_s_ += gimbal->Ts;
}

void Gimbal::GimbalMatchUpdate() {
  if (globals->aimbot_communicator->aimbot_state() >> 0 & 0x01) {
    gimbal->GimbalMove_ = kGbAimbot;
  } else {
    gimbal->GimbalMove_ = kGbRemote;
  }
  gimbal->GimbalEnableUpdate();
}

void Gimbal::GimbalEnableUpdate() {
  gimbal->DaMiaoMotorEnable();
  globals->gimbal_controller.Enable(true);
  if (gimbal->GimbalMove_ != kGbIdentify) {
    gimbal->identify_active_ = false;
    gimbal->ApplyNormalGimbalPID();
    globals->gimbal_controller.EnableSpeedPid(true);
  }
  if (gimbal->GimbalMove_ != kGbFfVerify) {
    gimbal->ff_verify_active_ = false;
  }
  if (gimbal->GimbalMove_ == kGbRemote) {
    gimbal->GimbalRCTargetUpdate();
    gimbal->GimbalMovePIDUpdate();
  } else if (gimbal->GimbalMove_ == kGbAimbot) {
    gimbal->GimbalAimbotTargetUpdate();
    gimbal->GimbalMovePIDUpdate();
  } else if (gimbal->GimbalMove_ == kGbAimbotFu) {
    globals->aim_mode = 0x02;
    if (globals->rc->dial() >= 650 && !globals->aim_mood_change_flag) {
      globals->aim_mode ^= static_cast<u8>(1 << 1);
      globals->aim_mode ^= static_cast<u8>(1 << 2);
      globals->aim_mood_change_flag = true;
    } else if (globals->rc->dial() <= 0) {
      globals->aim_mood_change_flag = false;
    }
    gimbal->GimbalAimbotTargetUpdate();
    gimbal->GimbalMovePIDUpdate();
  } else if (gimbal->GimbalMove_ == kGbIdentify) {
    gimbal->move_ff_initialized_ = false;
    gimbal->GimbalIdentifyUpdate();
  } else if (gimbal->GimbalMove_ == kGbFfVerify) {
    gimbal->move_ff_initialized_ = false;
    gimbal->GimbalFfVerifyUpdate();
  } else {
    globals->gimbal_controller.Enable(false);
    gimbal->yaw_current_ = 0.f;
    gimbal->yaw_torque_ = 0.f;
    gimbal->pitch_torque_ = 0.f;
  }
  gimbal->SetMotorCurrent();
}

void Gimbal::GimbalDisableUpdate() {
  gimbal->DaMiaoMotorDisable();
  gimbal->identify_active_ = false;
  gimbal->ff_verify_active_ = false;
  globals->gimbal_controller.EnableSpeedPid(true);
  globals->gimbal_controller.Enable(false);
  gimbal->gimbal_yaw_target_ = globals->ahrs.euler_angle().yaw;
  gimbal->gimbal_pitch_target_ = globals->ahrs.euler_angle().pitch;
  gimbal->GimbalMovePIDUpdate();
  gimbal->yaw_current_ = 0.f;
  gimbal->yaw_torque_ = 0.f;
  gimbal->pitch_torque_ = 0.f;
  gimbal->SetMotorCurrent();
}

void Gimbal::DaMiaoMotorEnable() {
  if (globals->pitch_motor->status() != 0x1F && globals->pitch_motor->status() != 0x0F) {
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  } else if (globals->pitch_motor->status() == 0x0F) {
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
  }
}

void Gimbal::DaMiaoMotorDisable() {
  if (globals->pitch_motor->status() != 0x1F && globals->pitch_motor->status() != 0x0F) {
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  } else if (globals->pitch_motor->status() == 0x1F) {
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
  }
}

void Gimbal::ShootEnableUpdate() {
  globals->shoot_controller.Enable(true);
  globals->shoot_controller.Arm(true);
  globals->shoot_controller.SetArmSpeed(gimbal->ammo_speed_ - static_cast<f32>(globals->aim_speed_change) * 100.0f);
  globals->dail_encoder_counter.Update(globals->dial_motor->encoder());
  if (globals->rc->dial() <= -650 ||
      (gimbal->GimbalMove_ == kGbAimbotFu && globals->aimbot_communicator->aimbot_state() >> 1 & 0x01) ||
      (globals->chassis_communicator->heat_limit() - globals->chassis_communicator->heat_real() > 30 &&
       (globals->df_state || globals->xf_state) &&
       (globals->image_update_flag ? globals->image_data->data().mouse_button_right
                                   : globals->rc->mouse_button_right()))) {
    if (!gimbal->single_shoot_flag_) {
      globals->shoot_controller.SetMode(Shoot3Fric::kSingleShot);
      globals->shoot_controller.Fire();
      gimbal->single_shoot_flag_ = true;
      gimbal->single_shoot_time_ = 200;
    } else if (gimbal->single_shoot_time_ > 0) {
      gimbal->single_shoot_time_--;
    } else if (gimbal->single_shoot_time_ == 0) {
      gimbal->single_shoot_flag_ = false;
    }
  } else if ((globals->StateMachine_ == kTest &&
              ((globals->rc->dial() >= 10 && globals->aimbot_communicator->aimbot_state() >> 0 & 0x01 &&
                globals->aimbot_communicator->aimbot_state() >> 1 & 0x01) ||
               globals->rc->dial() >= 650)) ||
             (globals->StateMachine_ == kMatch &&
              (((globals->image_update_flag ? globals->image_data->data().mouse_button_left
                                            : globals->rc->mouse_button_left()) &&  // 左键按下
                (globals->image_update_flag ? !globals->image_data->data().mouse_button_right
                                            : !globals->rc->mouse_button_right())) ||  // 右键未按下
               ((globals->image_update_flag ? globals->image_data->data().mouse_button_right
                                            : globals->rc->mouse_button_right()) &&  // 右键按下且瞄到目标
                globals->aimbot_communicator->aimbot_state() >> 0 & 0x01 &&
                globals->aimbot_communicator->aimbot_state() >> 1 & 0x01)))) {
    globals->shoot_controller.SetMode(Shoot3Fric::kFullAuto);
    if (globals->chassis_communicator->heat_limit() - globals->chassis_communicator->heat_real() > 60) {
      globals->shoot_controller.SetShootFrequency(20.0f);
    } else if (globals->chassis_communicator->heat_limit() - globals->chassis_communicator->heat_real() < 20) {
      globals->shoot_controller.SetShootFrequency(0.0f);
    } else {
      globals->shoot_controller.SetShootFrequency(
          static_cast<f32>(globals->chassis_communicator->heat_limit() - globals->chassis_communicator->heat_real()) /
          3.0f);
    }
  } else {
    globals->shoot_controller.SetShootFrequency(0.0f);
    gimbal->single_shoot_flag_ = false;
  }
  globals->shoot_controller.Update(globals->friction_left->rpm(), globals->friction_right->rpm(), 0,
                                   static_cast<f32>(globals->dail_encoder_counter.linear_ticks()),
                                   globals->dial_motor->rpm());
}

void Gimbal::ShootIdentifyUpdate() {
  globals->shoot_controller.Enable(true);
  globals->shoot_controller.Arm(true);
  globals->shoot_controller.SetMode(Shoot3Fric::kFullAuto);
  globals->shoot_controller.SetArmSpeed(gimbal->ammo_speed_);
  globals->shoot_controller.SetShootFrequency(0.0f);
  gimbal->single_shoot_flag_ = false;
  globals->dail_encoder_counter.Update(globals->dial_motor->encoder());
  globals->shoot_controller.Update(globals->friction_left->rpm(), globals->friction_right->rpm(), 0,
                                   static_cast<f32>(globals->dail_encoder_counter.linear_ticks()),
                                   globals->dial_motor->rpm());
}

void Gimbal::ShootDisableUpdate() {
  globals->shoot_controller.SetMode(Shoot3Fric::kStop);
  if (globals->StateMachine_ == kUnable) {
    globals->shoot_controller.Enable(false);
    globals->shoot_controller.Arm(false);
  } else {
    globals->shoot_controller.Enable(true);
    globals->shoot_controller.Arm(true);
    globals->shoot_controller.SetArmSpeed(0.f);
    globals->shoot_controller.SetShootFrequency(0.0f);
  }
  globals->shoot_controller.Update(globals->friction_left->rpm(), globals->friction_right->rpm(), 0,
                                   static_cast<f32>(globals->dail_encoder_counter.linear_ticks()),
                                   globals->dial_motor->rpm());
}

void Gimbal::GimbalIdentifyDataSend() {
  if (globals == nullptr || globals->ident_uart == nullptr || globals->StateMachine_ != kTest ||
      gimbal->GimbalMove_ != kGbIdentify || !gimbal->identify_active_) {
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
  len += AppendFloat(tx_buf + len, sizeof(tx_buf) - len, gimbal->pitch_torque_);
  len += std::snprintf(tx_buf + len, sizeof(tx_buf) - len, ",");
  len += AppendFloat(tx_buf + len, sizeof(tx_buf) - len, PitchRawToIdentifyModel(gimbal->identify_pitch_position_));
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
  globals->yaw_motor->SetCurrent(static_cast<i16>(gimbal->yaw_current_));
  globals->friction_left->SetCurrent(static_cast<i16>(globals->shoot_controller.output().fric_1));
  globals->friction_right->SetCurrent(static_cast<i16>(globals->shoot_controller.output().fric_2));
  globals->dial_motor->SetCurrent(static_cast<i16>(globals->shoot_controller.output().loader));

  // globals->yaw_motor->SetCurrent(0);
  // globals->friction_left->SetCurrent(0);
  // globals->friction_right->SetCurrent(0);
  // globals->dial_motor->SetCurrent(0);
}
