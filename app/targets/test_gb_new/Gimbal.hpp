#pragma once

#include <librm.hpp>

#include "main.hpp"

using namespace rm;

inline class Gimbal {
 public:
  StateMachineType GimbalMove_ = {kNoForce};              // 云台运动状态
  PitchGravity::Sampler<21> pitch_sampler_{-0.4f, 0.2f};  // 重力补偿采样器

  f32 yaw_torque_ = 0.0f;    // yaw 轴力矩数据
  f32 pitch_torque_ = 0.0f;  // pitch 轴力矩数据

 private:
  f32 gimbal_yaw_target_ = 0.0f;    // 云台上部yaw轴目标数据
  f32 gimbal_pitch_target_ = 0.0f;  // 云台pitch轴目标数据

  f32 Ts = 0.002f;
  f32 yaw_feedforward_torque_ = 0.0f;
  f32 last_yaw_target = 0.0f;
  f32 last_pitch_target_ = 0.0f;
  f32 last_yaw_speed_ref_ = 0.0f;
  f32 last_pitch_speed_ref_ = 0.0f;

  f32 ff_verify_time_s_ = 0.0f;
  f32 identify_time_s_ = 0.0f;
  f32 identify_yaw_center_ = 0.0f;
  f32 identify_pitch_center_ = 0.0f;
  f32 identify_yaw_position_ = 0.0f;
  f32 identify_yaw_speed_ = 0.0f;
  f32 identify_pitch_position_ = 0.0f;
  f32 identify_pitch_speed_ = 0.0f;

  f32 yaw_speed_ref = 0.0f;
  f32 pitch_speed_ref = 0.0f;
  f32 yaw_accel_ref = 0.0f;
  f32 pitch_accel_ref = 0.0f;

  const f32 sensitivity_yaw_ = 0.01f;     // 云台上部yaw轴灵敏度
  const f32 sensitivity_pitch_ = 0.01f;   // 云台pitch轴灵敏度
  const f32 highest_pitch_angle_ = 0.4f;  // 云台pitch轴最高（弧度制）
  const f32 lowest_pitch_angle_ = -0.6f;  // 云台pitch轴最低（弧度制）

 public:
  void GimbalInit();

  void GimbalTask();

  void GimbalIdentifyDataSend();

  [[nodiscard]] f32 yaw_target() const { return gimbal_yaw_target_; }
  [[nodiscard]] f32 pitch_target() const { return gimbal_pitch_target_; }
  [[nodiscard]] f32 yaw_speed_reference() const { return yaw_speed_ref; }
  [[nodiscard]] f32 pitch_speed_reference() const { return pitch_speed_ref; }
  [[nodiscard]] f32 yaw_accel_reference() const { return yaw_accel_ref; }
  [[nodiscard]] f32 pitch_accel_reference() const { return pitch_accel_ref; }
  [[nodiscard]] f32 yaw_feedforward_torque() const { return yaw_feedforward_torque_; }
  [[nodiscard]] f32 yaw_command() const { return yaw_torque_; }
  [[nodiscard]] f32 pitch_command() const { return pitch_torque_; }

 private:
  void GimbalStateUpdate();

  void GimbalRCTargetUpdate();

  void GimbalAimbotTargetUpdate();

  void ApplyNormalGimbalPID();

  void ApplyIdentifyGimbalPID();

  void GimbalIdentifyUpdate();

  void GimbalIdentifyTargetUpdate();

  void GimbalIdentifyPIDUpdate();

  void GimbalFfVerifyUpdate();

  void GimbalMatchUpdate();

  void GimbalEnableUpdate();

  void GimbalDisableUpdate();

  void DaMiaoMotorEnable();

  void DaMiaoMotorDisable();

  void GimbalMoveUpdate();

  void GimbalMoveSMCUpdate();

  void GimbalMovePIDUpdate();

  void GimbalPitchGravityCalc();
} *gimbal;
