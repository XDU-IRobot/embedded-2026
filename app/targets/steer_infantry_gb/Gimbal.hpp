#ifndef GIMBAL_HPP
#define GIMBAL_HPP

#include <librm.hpp>

#include "main.hpp"

using namespace rm;

inline class Gimbal {
 public:
  StateMachineType GimbalMove_ = {kNoForce};  // 云台运动状态

  f32 yaw_current_ = 0.0f;   // yaw轴力矩数据
  f32 pitch_torque_ = 0.0f;  // pitch轴力矩数据

 private:
  f32 gimbal_yaw_target_ = 0.0f;    // 云台上部yaw轴目标数据
  f32 gimbal_pitch_target_ = 0.0f;  // 云台pitch轴目标数据

  f32 ammo_speed_ = -6200.0f;  // 摩擦轮速度

  f32 yaw_speed_ff = 0.0f;
  f32 last_yaw_target = 0.0f;
  f32 Ts = 0.002f;
  f32 Kf = 1.0f;

  u16 heat_limit_ = 0;    // 热量上限值
  u16 heat_current_ = 0;  // 热量实时值

  u16 single_shoot_time_ = 0;  // 单发时长

  bool single_shoot_flag_ = false;  // 单发标志

  bool DM_enable_flag_ = false;  // 4310电机使能标志

  bool scan_yaw_flag_ = false;    // 扫描yaw轴方向标识位
  bool scan_pitch_flag_ = false;  // 扫描pitch轴方向标识位

  bool DF_flag_ = false;   // 大符标志
  bool XF_flag_ = false;   // 小符标志
  bool DF_state_ = false;  // 大符状态
  bool XF_state_ = false;  // 小符状态

  const f32 sensitivity_yaw_ = 0.01f;     // 云台上部yaw轴灵敏度
  const f32 sensitivity_pitch_ = 0.01f;   // 云台pitch轴灵敏度
  const f32 highest_pitch_angle_ = 0.4f;  // 云台pitch轴最高（弧度制）
  const f32 lowest_pitch_angle_ = -0.6f;  // 云台pitch轴最低（弧度制）

 public:
  void GimbalInit();

  void GimbalTask();

  void GimbalIdentifyDataSend();

 private:
  void GimbalStateUpdate();

  void GimbalRCTargetUpdate();

  void GimbalScanTargetUpdate();

  void GimbalAimbotTargetUpdate();

  void GimbalDownYawFollow();

  void GimbalMovePIDUpdate();

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

  void ShootEnableUpdate();

  void ShootIdentifyUpdate();

  void ShootDisableUpdate();

  void SetMotorCurrent();

  void EulerToQuaternion(f32 yaw, f32 pitch, f32 roll);

  EncoderCounter identify_yaw_encoder_counter_;
  bool identify_active_ = false;
  f32 identify_time_s_ = 0.0f;
  f32 identify_yaw_center_ = 0.0f;
  f32 identify_pitch_center_ = 0.0f;
  f32 identify_yaw_position_ = 0.0f;
  f32 identify_yaw_speed_ = 0.0f;
  f32 identify_pitch_position_ = 0.0f;
  f32 identify_pitch_speed_ = 0.0f;
  bool ff_verify_active_ = false;
  f32 ff_verify_time_s_ = 0.0f;
  f32 yaw_torque_ = 0.0f;
  bool move_ff_initialized_ = false;
  f32 last_pitch_target_ = 0.0f;
  f32 last_yaw_speed_ref_ = 0.0f;
  f32 last_pitch_speed_ref_ = 0.0f;
} *gimbal;

#endif  // GIMBAL_HPP
