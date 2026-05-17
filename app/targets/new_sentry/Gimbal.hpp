#ifndef GIMBAL_HPP
#define GIMBAL_HPP

#include <librm.hpp>

#include "main.hpp"

using namespace rm;

inline class Gimbal {
 public:
  StateMachineType GimbalMove_ = {kNoForce};  // 云台运动状态

  f32 pitch_torque_ = 0.0f;     // pitch轴力矩数据
  f32 down_yaw_torque_ = 0.0f;  // 下部yaw轴力矩数据
 private:
  rm::modules::TrajectoryLimiter up_yaw_move_limiter_{4.0f, 16.0f};
  rm::modules::TrajectoryLimiter down_yaw_move_limiter_{4.0f, 16.0f};

  f32 gimbal_up_yaw_target_ = 0.0f;    // 云台上部yaw轴目标数据（编码器控制，弧度制，左正右负）
  f32 gimbal_down_yaw_target_ = 0.0f;  // 云台下部yaw轴目标数据（陀螺仪控制，弧度制，左正右负）
  f32 gimbal_pitch_target_ = 0.0f;     // 云台pitch轴目标数据（编码器控制，弧度制，下正上负）

  f32 ammo_speed_ = 6800.0f;  // 摩擦轮速度初速度

  u16 aimbot_time_ = 0;        // 自瞄运动时间
  u16 perception_time_ = 0;    // 全向感知运动时间
  u16 single_shoot_time_ = 0;  // 单发时间

  f32 up_yaw_percept_target_ = 0.0f;   // 云台上部yaw轴感知目标数据（编码器控制，弧度制，左正右负）
  f32 down_yaw_percept_target_ = 0.f;  // 云台下部yaw轴感知目标数据（陀螺仪控制，弧度制，左正右负）

  f32 shoot_frequency_ = 0.0f;

  u16 heat_limit_ = 0;    // 热量上限值
  u16 heat_current_ = 0;  // 热量实时值

  bool single_shoot_flag_ = false;  // 单发标志

  bool down_yaw_enable_flag_ = false;  // 4310电机使能标志
  bool pitch_enable_flag_ = false;     // 4310电机使能标志

  bool percept_move_complete_ = true;  // 全向感知运动完成标志

  bool scan_yaw_flag_ = false;    // 扫描yaw轴方向标识位
  bool scan_pitch_flag_ = false;  // 扫描pitch轴方向标识位

  bool DF_flag_ = false;   // 大符标志
  bool XF_flag_ = false;   // 小符标志
  bool DF_state_ = false;  // 大符状态
  bool XF_state_ = false;  // 小符状态

  const f32 lowest_aimbot_pitch_angle_ = -0.3f;  // 云台自瞄扫描pitch轴最大（弧度制）
  const f32 highest_pitch_angle_ = 0.6f;         // 云台pitch轴最高（弧度制）
  const f32 lowest_pitch_angle_ = -0.7f;         // 云台pitch轴最低（弧度制）
  const u16 mid_up_yaw_pos_ = 1360;              // 云台上部yaw轴最大（编码器值）
  const u16 max_up_yaw_pos_ = 2310;              // 云台上部yaw轴最大（编码器值）
  const u16 min_up_yaw_pos_ = 410;               // 云台上部yaw轴最小（编码器值）
  const u16 down_yaw_move_high_ = 1910;          // 云台下部yaw轴运动高阈值（编码器值）
  const u16 down_yaw_move_low_ = 810;            // 云台下部yaw轴运动低阈值（编码器值）

 public:
  void GimbalInit();

  void GimbalTask();

  void GimbalIdentifyDataSend();

 private:
  void GimbalStateUpdate();

  void GimbalRCTargetUpdate();

  void GimbalScanTargetUpdate();

  void GimbalPerceptTargetUpdate();

  void GimbalAimbotTargetUpdate();

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

  void ShootDisableUpdate();

  void SetMotorCurrent();

  void EulerToQuaternion(f32 yaw, f32 pitch, f32 roll);

  EncoderCounter identify_yaw_encoder_counter_;
  f32 Kf = 2.0f;
  f32 Ts = 0.002f;
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
  f32 yaw_speed_ff_ = 0.0f;
  f32 up_yaw_current_ = 0.0f;
  f32 last_yaw_target_ = 0.0f;
  f32 last_pitch_target_ = 0.0f;
  f32 last_yaw_speed_ref_ = 0.0f;
  f32 last_pitch_speed_ref_ = 0.0f;
} *gimbal;

#endif  // GIMBAL_HPP
