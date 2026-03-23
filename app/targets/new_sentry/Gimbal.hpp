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

  f32 ammo_speed_ = 7200.0f;  // 摩擦轮速度初速度

  u16 perception_time_ = 0;  // 全向感知运动时间

  f32 up_yaw_percept_target_ = 0.0f;  // 云台上部yaw轴感知目标数据（编码器控制，弧度制，左正右负）
  f32 down_yaw_percept_target_ = 0.f;  // 云台下部yaw轴感知目标数据（陀螺仪控制，弧度制，左正右负）

  f32 shoot_frequency_ = 0.0f;

  u16 heat_limit_ = 0;    // 热量上限值
  u16 heat_current_ = 0;  // 热量实时值

  bool shoot_flag_ = false;         // 开火标志
  bool single_shoot_flag_ = false;  // 单发标志

  bool ammo_speed_change_flag_ = false;  // 摩擦轮速度改变标志

  bool down_yaw_enable_flag_ = false;  // 4310电机使能标志
  bool pitch_enable_flag_ = false;     // 4310电机使能标志
  bool max_angle_flag_ = false;        // 云台上部yaw轴最大角度标志
  bool min_angle_flag_ = false;        // 云台上部yaw轴最小角度标志

  bool percept_move_complete_ = true;  // 全向感知运动完成标志

  bool scan_yaw_flag_ = false;    // 扫描yaw轴方向标识位
  bool scan_pitch_flag_ = false;  // 扫描pitch轴方向标识位

  bool DF_flag_ = false;   // 大符标志
  bool XF_flag_ = false;   // 小符标志
  bool DF_state_ = false;  // 大符状态
  bool XF_state_ = false;  // 小符状态

  const f32 highest_aimbot_pitch_angle_ = -0.3f;  // 云台上部yaw轴最大（弧度制）
  const f32 highest_pitch_angle_ = 0.6f;          // 云台pitch轴最高（弧度制）
  const f32 lowest_pitch_angle_ = -0.7f;          // 云台pitch轴最低（弧度制）
  const u16 max_up_yaw_pos_ = 5500;               // 云台上部yaw轴最大（编码器值）
  const u16 min_up_yaw_pos_ = 2600;               // 云台上部yaw轴最小（编码器值）
  const u16 down_yaw_move_high_ = 5200;           // 云台下部yaw轴运动高阈值（编码器值）
  const u16 down_yaw_move_low_ = 2900;            // 云台下部yaw轴运动低阈值（编码器值）

 public:
  void GimbalInit();

  void GimbalTask();

 private:
  void GimbalStateUpdate();

  void GimbalRCTargetUpdate();

  void GimbalScanTargetUpdate();

  void GimbalPerceptTargetUpdate();

  void GimbalAimbotTargetUpdate();

  void GimbalMovePIDUpdate();

  void GimbalMatchUpdate();

  void GimbalEnableUpdate();

  void GimbalDisableUpdate();

  void DaMiaoMotorEnable();

  void DaMiaoMotorDisable();

  void ShootEnableUpdate();

  void ShootDisableUpdate();

  void SetMotorCurrent();

  void EulerToQuaternion(f32 yaw, f32 pitch, f32 roll);
} *gimbal;

#endif  // GIMBAL_HPP
