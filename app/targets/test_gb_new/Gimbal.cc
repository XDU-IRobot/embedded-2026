#include "Gimbal.hpp"

#include <cstdio>

#include "gimbal-tool-suite/dynamics/dynamics.hpp"

namespace {
constexpr size_t kIdentifyHarmonicCount = 5;
constexpr f32 kIdentifyBaseFreqHz = 0.1f;
constexpr f32 kIdentifyPitchTopLimit = -1.4521f;
constexpr f32 kIdentifyPitchBottomLimit = -0.3057f;
constexpr f32 kIdentifyPitchCenter = (kIdentifyPitchTopLimit + kIdentifyPitchBottomLimit) * 0.5f;
constexpr f32 kIdentifyYawAmp[kIdentifyHarmonicCount] = {3.5f, -2.0f, 1.2f, -0.8f, 0.5f};
constexpr f32 kIdentifyPitchAmp[kIdentifyHarmonicCount] = {0.34f, -0.18f, 0.11f, -0.07f, 0.04f};
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
  gimbal->identify_yaw_position_ = globals->yaw_motor->pos();
  gimbal->identify_yaw_speed_ = globals->yaw_motor->vel();
  gimbal->identify_pitch_position_ = globals->pitch_motor->pos();
  gimbal->identify_pitch_speed_ = globals->pitch_motor->vel();
  gimbal->identify_yaw_center_ = 0.0f;
  gimbal->identify_pitch_center_ = kIdentifyPitchCenter;
  gimbal->pitch_sampler_.Start();
}

void Gimbal::GimbalTask() { gimbal->GimbalStateUpdate(); }

void Gimbal::GimbalStateUpdate() {
  if (!globals->device_gimbal.all_device_ok()) {
    globals->StateMachine_ = kUnable;  // 如果云台设备离线，进入无力模式
    gimbal->GimbalDisableUpdate();     // 云台电机失能计算
  } else {
    switch (globals->StateMachine_) {
      case kNoForce:                    // 无力模式下，所有电机失能
        gimbal->GimbalDisableUpdate();  // 云台电机失能计算
        break;

      case kTest:                      // 云台测试模式
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
}

void Gimbal::GimbalRCTargetUpdate() {
  gimbal->gimbal_yaw_target_ -=
      rm::modules::Map(globals->remote_input.left_x * 660.0f + 30.0f * static_cast<f32>(globals->remote_input.mouse_x),
                       -660, 660, -gimbal->sensitivity_yaw_, gimbal->sensitivity_yaw_);
  gimbal->gimbal_pitch_target_ -=
      rm::modules::Map(globals->remote_input.left_y * 660.0f + 30.0f * static_cast<f32>(globals->remote_input.mouse_y),
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

void Gimbal::ApplyNormalGimbalPID() {
  globals->gimbal_controller.motor()
      .yaw.pid()
      .position.SetKp(380.0f)
      .SetKi(0)
      .SetKd(9000.0f)
      .SetMaxOut(30000.0f)
      .SetMaxIout(0);
  globals->gimbal_controller.motor().yaw.pid().speed.SetKp(580.0f).SetKi(0).SetKd(0.0f).SetMaxOut(30000.0f).SetMaxIout(
      0);
  // pitch PID 参数
  globals->gimbal_controller.motor()
      .pitch.pid()
      .position.SetKp(42.0f)
      .SetKi(0)
      .SetKd(600.0f)
      .SetMaxOut(10000.0f)
      .SetMaxIout(0);
  globals->gimbal_controller.motor().pitch.pid().speed.SetKp(0.5f).SetKi(0).SetKd(0.0f).SetMaxOut(10.0f).SetMaxIout(0);
}

void Gimbal::ApplyIdentifyGimbalPID() {
  globals->gimbal_controller.motor().yaw.pid().position.SetKp(75000).SetKi(0).SetKd(10000).SetMaxOut(30000).SetMaxIout(
      0);
  globals->gimbal_controller.motor().pitch.pid().position.SetKp(45).SetKi(0).SetKd(65).SetMaxOut(10).SetMaxIout(0);
}

void Gimbal::GimbalIdentifyUpdate() {
  gimbal->ApplyIdentifyGimbalPID();
  // globals->gimbal_controller.EnableSpeedPid(false);  // 设置为纯位置pid模式
  gimbal->identify_yaw_position_ = globals->yaw_motor->pos();
  gimbal->identify_yaw_speed_ = globals->yaw_motor->vel();
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
  globals->gimbal_controller.Update(gimbal->identify_yaw_position_, gimbal->identify_yaw_speed_,
                                    gimbal->identify_pitch_position_, gimbal->identify_pitch_speed_, Ts);
  gimbal->yaw_torque_ = rm::modules::Clamp(globals->gimbal_controller.output().yaw, -10.0f, 10.0f);
  gimbal->pitch_torque_ = rm::modules::Clamp(globals->gimbal_controller.output().pitch, -10.0f, 10.0f);
}

void Gimbal::GimbalFfVerifyUpdate() {
  globals->gimbal_controller.Enable(false);
  // globals->gimbal_controller.EnableSpeedPid(false);  // 设置为纯位置pid模式
  gimbal->identify_yaw_position_ = globals->yaw_motor->pos();
  gimbal->identify_yaw_speed_ = globals->yaw_motor->vel();
  gimbal->identify_pitch_position_ = globals->pitch_motor->pos();
  gimbal->identify_pitch_speed_ = globals->pitch_motor->vel();

  const auto yaw = EvaluateIdentifyTrajectory(0.0f, kIdentifyYawAmp, gimbal->ff_verify_time_s_);
  const auto pitch = EvaluateIdentifyTrajectory(0.0f, kIdentifyPitchAmp, gimbal->ff_verify_time_s_);
  gimbal->gimbal_yaw_target_ = yaw.q;
  gimbal->gimbal_pitch_target_ =
      rm::modules::Clamp(pitch.q + kIdentifyPitchCenter, kIdentifyPitchTopLimit, kIdentifyPitchBottomLimit);

  // 按参考轨迹计算完整动力学前馈力矩
  const Eigen::Vector3f g_stationary(0.0f, 0.0f, -9.81f);
  const auto ff =
      g_gimbal_dynamics.ComputeFfDecomposed(yaw.q, pitch.q, yaw.dq, pitch.dq, yaw.ddq, pitch.ddq, g_stationary);

  gimbal->yaw_torque_ = rm::modules::Clamp(ff.yaw, -10.0f, 10.0f);
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
    // globals->gimbal_controller.EnableSpeedPid(true);
  }
  if (gimbal->GimbalMove_ == kGbRemote) {
    globals->aim_mode = 0x01;
    gimbal->GimbalRCTargetUpdate();
    gimbal->GimbalMoveUpdate();
  } else if (gimbal->GimbalMove_ == kGbAimbot) {
    globals->aim_mode = 0x01;
    gimbal->GimbalAimbotTargetUpdate();
    gimbal->GimbalMoveUpdate();
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
    gimbal->GimbalMoveUpdate();
  } else if (gimbal->GimbalMove_ == kGbIdentify) {
    gimbal->GimbalIdentifyUpdate();
  } else if (gimbal->GimbalMove_ == kGbFfVerify) {
    gimbal->GimbalFfVerifyUpdate();
  } else {
    globals->gimbal_controller.Enable(false);
    gimbal->yaw_torque_ = 0.f;
    gimbal->pitch_torque_ = 0.f;
  }
}

void Gimbal::GimbalDisableUpdate() {
  gimbal->DaMiaoMotorDisable();
  globals->aim_mode = 0x01;
  // globals->gimbal_controller.EnableSpeedPid(true);
  globals->gimbal_controller.Enable(false);
  gimbal->gimbal_yaw_target_ = globals->ahrs.euler_angle().yaw;
  gimbal->gimbal_pitch_target_ = globals->ahrs.euler_angle().pitch;
  // // TODO
  // gimbal->gimbal_yaw_target_ = globals->yaw_motor->pos();
  // gimbal->gimbal_pitch_target_ = globals->pitch_motor->pos();

  gimbal->GimbalMoveUpdate();
  gimbal->yaw_torque_ = 0.f;
  gimbal->pitch_torque_ = 0.f;
}

void Gimbal::DaMiaoMotorEnable() {
  for (auto *motor : {globals->yaw_motor, globals->pitch_motor}) {
    if (motor->status() != 0x01 && motor->status() != 0x00) {
      motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
    } else if (motor->status() == 0x00) {
      motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    }
  }
}

void Gimbal::DaMiaoMotorDisable() {
  for (auto *motor : {globals->yaw_motor, globals->pitch_motor}) {
    if (motor->status() != 0x01 && motor->status() != 0x00) {
      motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
    } else if (motor->status() == 0x01) {
      motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    }
  }
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

void Gimbal::GimbalMoveUpdate() {
  gimbal->last_yaw_target = gimbal->gimbal_yaw_target_;
  gimbal->last_pitch_target_ = gimbal->gimbal_pitch_target_;
  gimbal->last_yaw_speed_ref_ = yaw_speed_ref;
  gimbal->last_pitch_speed_ref_ = pitch_speed_ref;

  f32 yaw_target_for_controller = gimbal->gimbal_yaw_target_;
  f32 pitch_target_for_controller = gimbal->gimbal_pitch_target_;

  // 生成波形叠加测试
  if (gimbal->GimbalMove_ == kGbAimbot) {
    yaw_target_for_controller = rm::modules::Wrap(yaw_target_for_controller + globals->yaw_wave.GetValue(),
                                                  -static_cast<f32>(M_PI), static_cast<f32>(M_PI));
    // pitch_target_for_controller = rm::modules::Wrap(pitch_target_for_controller + globals->pitch_wave.GetValue(),
    //                                                 -static_cast<f32>(M_PI), static_cast<f32>(M_PI));
  }

  // const f32 pitch_ff_nm = PitchGravity::Model::Calc(0.83432, 2.92575, globals->ahrs.euler_angle().pitch) + 0.01f;
  const f32 pitch_ff_nm = 0.0f;
  const f32 yaw_ff_cmd = 0.0f;
  globals->gimbal_controller.SetTarget(yaw_target_for_controller, pitch_target_for_controller);
  globals->gimbal_controller.SetYawFeedforward(0, yaw_ff_cmd);
  globals->gimbal_controller.SetPitchFeedforward(0, pitch_ff_nm);
  // globals->gimbal_controller.Update(globals->yaw_motor->pos(), globals->imu->gyro_z(),
  //                                   globals->pitch_motor->pos(), globals->imu->gyro_x(), Ts);
  globals->gimbal_controller.UpdateWithImu(globals->ahrs.euler_angle().yaw, globals->yaw_motor->pos(),
                                           globals->imu->gyro_z(), globals->ahrs.euler_angle().pitch,
                                           globals->pitch_motor->pos(), globals->imu->gyro_x(), Ts);
  // globals->gimbal_controller.UpdateWithImu(globals->ahrs.euler_angle().yaw, globals->yaw_motor->pos(),
  // globals->yaw_motor->vel(),
  //   globals->ahrs.euler_angle().pitch, globals->pitch_motor->pos(), globals->pitch_motor->vel(), Ts);

  gimbal->yaw_torque_ = globals->gimbal_controller.output().yaw;
  gimbal->pitch_torque_ = globals->gimbal_controller.output().pitch;
}

void Gimbal::GimbalPitchGravityCalc() {
  static bool success_announced = false;

  gimbal_pitch_target_ =
      pitch_sampler_.Update(globals->ahrs.euler_angle().pitch, globals->imu->gyro_x(), pitch_torque_, Ts);
  if (pitch_sampler_.IsSampling()) {
    success_announced = false;
  }

  if (!pitch_sampler_.IsSampling() && pitch_sampler_.GetIndex() == 42 && !success_announced) {
    globals->buzzer_controller.Play<modules::buzzer_melody::Success>();
    success_announced = true;
  }
  GimbalMoveUpdate();
}
