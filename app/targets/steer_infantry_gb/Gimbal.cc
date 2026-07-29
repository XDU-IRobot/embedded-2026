#include "Gimbal.hpp"

#include <cstdio>

#include "gimbal-tool-suite/dynamics/dynamics.hpp"

f32 a, b, c, d;

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
  theta << 0.11603785, 0.13510521, 0.06316841, 0.08590306, 0.0388916, 0.01889259, 0.56985536, 0.33167135, 0.03996353;
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
  gimbal->last_yaw_speed_ref_ = 0.0f;
  gimbal->last_pitch_speed_ref_ = 0.0f;
  gimbal->ff_verify_time_s_ = 0.0f;
  gimbal->identify_time_s_ = 0.0f;
  gimbal->identify_yaw_position_ = 0.0f;
  gimbal->identify_yaw_speed_ = 0.0f;
  gimbal->identify_pitch_position_ = globals->pitch_motor->pos();
  gimbal->identify_pitch_speed_ = globals->pitch_motor->vel();
  gimbal->identify_yaw_center_ = 0.0f;
  gimbal->identify_pitch_center_ = kIdentifyPitchCenter;
  gimbal->identify_yaw_encoder_counter_.Reset(0, globals->yaw_motor->encoder());
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
      globals->remote_input.left_x * 660.0f + 30.0f * static_cast<f32>(globals->remote_input.mouse_x),
      -660, 660, -gimbal->sensitivity_yaw_, gimbal->sensitivity_yaw_);
  gimbal->gimbal_pitch_target_ -= rm::modules::Map(
      globals->remote_input.left_y * 660.0f + 30.0f * static_cast<f32>(globals->remote_input.mouse_y),
      -660, 660, -gimbal->sensitivity_pitch_, gimbal->sensitivity_pitch_);
  gimbal->gimbal_yaw_target_ =
      rm::modules::Wrap(gimbal->gimbal_yaw_target_, -static_cast<f32>(M_PI), M_PI);  // yaw轴限位
  gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_,    // pitch轴限位
                                                    gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);
  // 遥控模式：从位置目标差分得到速度/加速度
  const f32 yaw_delta = rm::modules::Wrap(gimbal->gimbal_yaw_target_ - gimbal->last_yaw_target, -static_cast<f32>(M_PI),
                                          static_cast<f32>(M_PI));
  yaw_speed_ref = rm::modules::Clamp(yaw_delta / gimbal->Ts, -kNormalFfMaxYawSpeed, kNormalFfMaxYawSpeed);
  pitch_speed_ref = rm::modules::Clamp((gimbal->gimbal_pitch_target_ - gimbal->last_pitch_target_) / gimbal->Ts,
                                       -kNormalFfMaxPitchSpeed, kNormalFfMaxPitchSpeed);
  yaw_accel_ref = rm::modules::Clamp((yaw_speed_ref - gimbal->last_yaw_speed_ref_) / gimbal->Ts, -kNormalFfMaxYawAccel,
                                     kNormalFfMaxYawAccel);
  pitch_accel_ref = rm::modules::Clamp((pitch_speed_ref - gimbal->last_pitch_speed_ref_) / gimbal->Ts,
                                       -kNormalFfMaxPitchAccel, kNormalFfMaxPitchAccel);
}

void Gimbal::GimbalAimbotTargetUpdate() {
  if ((globals->StateMachine_ == kTest && globals->aimbot_communicator->aimbot_state() >> 0 & 0x01) ||
      (globals->StateMachine_ == kMatch && globals->remote_input.mouse_right)) {
    gimbal->gimbal_yaw_target_ = globals->aimbot_communicator->yaw();
    gimbal->gimbal_pitch_target_ = globals->aimbot_communicator->pitch();
    gimbal->gimbal_yaw_target_ =
        rm::modules::Wrap(gimbal->gimbal_yaw_target_, -static_cast<f32>(M_PI), M_PI);  // yaw轴限位
    gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_,    // pitch轴限位
                                                      gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);
    // 自瞄模式：直接使用 NUC 下发的目标速度/加速度
    yaw_speed_ref = globals->aimbot_communicator->yaw_vel();
    pitch_speed_ref = globals->aimbot_communicator->pitch_vel();
    yaw_accel_ref = globals->aimbot_communicator->yaw_acc();
    pitch_accel_ref = globals->aimbot_communicator->pitch_acc();
  } else {
    gimbal->GimbalRCTargetUpdate();
  }
}

void Gimbal::GimbalMovePIDUpdate() {
  gimbal->yaw_speed_ff = gimbal->Kf * yaw_speed_ref;
  gimbal->last_yaw_target = gimbal->gimbal_yaw_target_;
  gimbal->last_pitch_target_ = gimbal->gimbal_pitch_target_;
  gimbal->last_yaw_speed_ref_ = yaw_speed_ref;
  gimbal->last_pitch_speed_ref_ = pitch_speed_ref;
  gimbal->identify_yaw_encoder_counter_.Update(globals->yaw_motor->encoder());
  const auto yaw_position_ = static_cast<f32>(gimbal->identify_yaw_encoder_counter_.linear_ticks()) /
                             kEncoderTicksPerRev * 2.0f * static_cast<f32>(M_PI);

  globals->gimbal_controller.SetTarget(gimbal->gimbal_yaw_target_, gimbal->gimbal_pitch_target_, gimbal->yaw_speed_ff);
  globals->gimbal_controller.Update(globals->ahrs.euler_angle().yaw, globals->imu->gyro_z(),
                                    globals->ahrs.euler_angle().pitch, globals->imu->gyro_x());
  const Eigen::Vector3f g_stationary(0.0f, 0.0f, -9.81f);
  const auto ff =
      g_gimbal_dynamics.ComputeFf(yaw_position_, -globals->pitch_motor->pos() - 1.0f, 0, 0, 0, 0, g_stationary);
  gimbal->yaw_torque_ = ff.x();
  const f32 yaw_ff_voltage =
      YawTorqueToVoltageCmd(gimbal->yaw_torque_, static_cast<f32>(globals->yaw_motor->rpm()) * kRpmToRadPerSec);
  gimbal->yaw_current_ =
      globals->gimbal_controller.output().yaw + static_cast<f32>(globals->yaw_motor->rpm()) * 100.f + yaw_ff_voltage;
  gimbal->yaw_current_ = rm::modules::Clamp(gimbal->yaw_current_, -kGm6020VoltageCmdLimit, kGm6020VoltageCmdLimit);
  gimbal->pitch_torque_ = globals->gimbal_controller.output().pitch + ff.y()-0.001;
  gimbal->pitch_torque_ = rm::modules::Clamp(gimbal->pitch_torque_, -10.f, 10.f);
}

void Gimbal::ApplyNormalGimbalPID() {
  globals->gimbal_controller.pid().yaw_position.SetKp(380.0f).SetKi(0).SetKd(9000.0f).SetMaxOut(30000.0f).SetMaxIout(0);
  globals->gimbal_controller.pid().yaw_speed.SetKp(580.0f).SetKi(0).SetKd(0.0f).SetMaxOut(30000.0f).SetMaxIout(0);
  // pitch PID 参数
  globals->gimbal_controller.pid().pitch_position.SetKp(50.0f).SetKi(0).SetKd(600.0f).SetMaxOut(10000.0f).SetMaxIout(0);
  globals->gimbal_controller.pid().pitch_speed.SetKp(0.5f).SetKi(0).SetKd(0.0f).SetMaxOut(10.0f).SetMaxIout(0);
}

void Gimbal::ApplyIdentifyGimbalPID() {
  globals->gimbal_controller.pid().yaw_position.SetKp(75000).SetKi(0).SetKd(10000).SetMaxOut(30000).SetMaxIout(0);
  globals->gimbal_controller.pid().pitch_position.SetKp(45).SetKi(0).SetKd(65).SetMaxOut(10).SetMaxIout(0);
}

void Gimbal::GimbalIdentifyUpdate() {
  gimbal->ApplyIdentifyGimbalPID();
  globals->gimbal_controller.EnableSpeedPid(false);

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
  globals->gimbal_controller.SetTarget(gimbal->gimbal_yaw_target_, gimbal->gimbal_pitch_target_);
  globals->gimbal_controller.Update(gimbal->identify_yaw_position_, 0.0f, gimbal->identify_pitch_position_, 0.0f);
  gimbal->yaw_current_ =
      rm::modules::Clamp(globals->gimbal_controller.output().yaw, -kGm6020VoltageCmdLimit, kGm6020VoltageCmdLimit);
  gimbal->yaw_torque_ = YawVoltageCmdToTorque(gimbal->yaw_current_, gimbal->identify_yaw_speed_);
  gimbal->pitch_torque_ = rm::modules::Clamp(globals->gimbal_controller.output().pitch, -10.0f, 10.0f);
}

void Gimbal::GimbalFfVerifyUpdate() {
  globals->gimbal_controller.Enable(false);
  globals->gimbal_controller.EnableSpeedPid(false);
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
    gimbal->ApplyNormalGimbalPID();
    globals->gimbal_controller.EnableSpeedPid(true);
  }
  if (gimbal->GimbalMove_ == kGbRemote) {
    globals->aim_mode = 0x01;
    gimbal->GimbalRCTargetUpdate();
    gimbal->GimbalMovePIDUpdate();
  } else if (gimbal->GimbalMove_ == kGbAimbot) {
    globals->aim_mode = 0x01;
    gimbal->GimbalAimbotTargetUpdate();
    gimbal->GimbalMovePIDUpdate();
  } else if (gimbal->GimbalMove_ == kGbAimbotFu) {
    if (globals->aim_mode != 0x02 && globals->aim_mode != 0x03) {
      globals->aim_mode = 0x02;
    }
    if (globals->remote_input.dial <= -0.015f && !globals->aim_mood_change_flag) {
      if (globals->aim_mode == 0x02) {
        globals->aim_mode = 0x03;
      } else if (globals->aim_mode == 0x03) {
        globals->aim_mode = 0x02;
      }
      globals->aim_mood_change_flag = true;
    } else if (globals->remote_input.dial >= 0.0f) {
      globals->aim_mood_change_flag = false;
    }
    gimbal->GimbalAimbotTargetUpdate();
    gimbal->GimbalMovePIDUpdate();
  } else if (gimbal->GimbalMove_ == kGbIdentify) {
    gimbal->GimbalIdentifyUpdate();
  } else if (gimbal->GimbalMove_ == kGbFfVerify) {
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
  globals->aim_mode = 0x01;
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
  if (globals->pitch_motor->status() != 0x01 && globals->pitch_motor->status() != 0x00) {
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  } else if (globals->pitch_motor->status() == 0x00) {
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
  }
}

void Gimbal::DaMiaoMotorDisable() {
  if (globals->pitch_motor->status() != 0x01 && globals->pitch_motor->status() != 0x00) {
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  } else if (globals->pitch_motor->status() == 0x01) {
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
  }
}

void Gimbal::ShootEnableUpdate() {
  globals->shoot_controller.Enable(true);
  globals->shoot_controller.Arm(true);
  globals->shoot_controller.SetArmSpeed(gimbal->ammo_speed_ - static_cast<f32>(globals->aim_speed_change) * 100.0f);
  globals->dail_encoder_counter.Update(globals->dial_motor->encoder());
  if (globals->remote_input.dial <= -0.98f || globals->remote_input.trigger ||
      (gimbal->GimbalMove_ == kGbAimbotFu && globals->aimbot_communicator->aimbot_state() >> 1 & 0x01) ||
      (globals->chassis_communicator->heat_limit() - globals->chassis_communicator->heat_real() > 30 &&
       (globals->df_state || globals->xf_state) && globals->remote_input.mouse_right)) {
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
  } else if ((globals->StateMachine_ == kTest && ((globals->aimbot_communicator->aimbot_state() >> 0 & 0x01 &&
                                                   globals->aimbot_communicator->aimbot_state() >> 1 & 0x01) ||
                                                  globals->remote_input.dial >= 0.98f)) ||
             (globals->StateMachine_ == kMatch &&
               ((globals->remote_input.mouse_left && !globals->remote_input.mouse_right) ||
                (globals->remote_input.mouse_right &&
                 globals->aimbot_communicator->aimbot_state() >> 0 & 0x01 &&
                 globals->aimbot_communicator->aimbot_state() >> 1 & 0x01)))) {
    globals->shoot_controller.SetMode(Shoot3Fric::kFullAuto);
    if (globals->chassis_communicator->heat_limit() - globals->chassis_communicator->heat_real() > 100) {
      globals->shoot_controller.SetShootFrequency(20.0f);
    } else if (globals->chassis_communicator->heat_limit() - globals->chassis_communicator->heat_real() < 30) {
      globals->shoot_controller.SetShootFrequency(0.0f);
    } else {
      globals->shoot_controller.SetShootFrequency(
          static_cast<f32>(globals->chassis_communicator->heat_limit() - globals->chassis_communicator->heat_real()) /
          5.0f);
    }
  } else {
    globals->shoot_controller.SetShootFrequency(0.0f);
    gimbal->single_shoot_flag_ = false;
  }
  globals->shoot_controller.Update(globals->friction_left->rpm(), globals->friction_right->rpm(), 0,
                                   static_cast<f32>(globals->dail_encoder_counter.linear_ticks()),
                                   globals->dial_motor->rpm());

  // --- 开火延迟测量 (500Hz, 每tick=2ms) ---
  fd_tick_++;
  float fric_rpm = std::fabs(static_cast<float>(globals->friction_left->rpm()));

  bool is_firing;
  if (gimbal->single_shoot_flag_) {
    is_firing = !globals->shoot_controller.shoot_flag();
  } else {
    is_firing = (globals->shoot_controller.target().loader_speed != 0.0f);
  }

  switch (fd_state_) {
    case kFdIdle:
      if (is_firing) {
        fd_peak_rpm_ = fric_rpm;
        fd_arm_tick_ = fd_tick_;
        fd_state_ = kFdArmed;
      }
      break;
    case kFdArmed:
      if (fric_rpm > fd_peak_rpm_) fd_peak_rpm_ = fric_rpm;
      if (fric_rpm <= fd_peak_rpm_ - 100.0f) {
        float delay_ms = static_cast<float>(fd_tick_ - fd_arm_tick_) * 2.0f;
        if (delay_ms > 0.0f && delay_ms < 500.0f) {
          fd_sum_ms_ += delay_ms;
          fd_count_++;
        }
        fd_state_ = kFdDropped;
      } else if ((fd_tick_ - fd_arm_tick_) > 250) {
        fd_state_ = kFdIdle;
      }
      break;
    case kFdDropped:
      if (fric_rpm >= fd_peak_rpm_ - 50.0f) {
        if (is_firing) {
          fd_peak_rpm_ = fric_rpm;
          fd_arm_tick_ = fd_tick_;
          fd_state_ = kFdArmed;
        } else {
          fd_state_ = kFdIdle;
        }
      } else if (!is_firing && (fd_tick_ - fd_arm_tick_) > 500) {
        fd_state_ = kFdIdle;
      }
      break;
  }
}

void Gimbal::ShootDisableUpdate() {
  fd_state_ = kFdIdle;  // 重置开火延迟状态机
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
  globals->ident_uart->Write(reinterpret_cast<const u8 *>(tx_buf), static_cast<usize>(len), 500);
}

void Gimbal::SetMotorCurrent() {
  globals->yaw_motor->SetCurrent(static_cast<i16>(gimbal->yaw_current_));
  globals->friction_left->SetCurrent(static_cast<i16>(globals->shoot_controller.output().fric_1));
  globals->friction_right->SetCurrent(static_cast<i16>(globals->shoot_controller.output().fric_2));
  globals->dial_motor->SetCurrent(static_cast<i16>(globals->shoot_controller.output().loader));
}
