#ifndef GIMBAL_HPP
#define GIMBAL_HPP

#include <librm.hpp>

#include "main.hpp"

using namespace rm;

inline class Gimbal {
 public:
  StateMachineType GimbalMove_ = {kNoForce};  // 云台运动状态

  f32 pitch_torque_ = 0.0f;  // pitch轴力矩数据

 private:
  f32 gimbal_yaw_target_ = 0.0f;  // 云台上部yaw轴目标数据（编码器控制，编码器制，1500.0~3500.0f，左正右负）
  f32 gimbal_pitch_target_ = 0.0f;  // 云台pitch轴目标数据（编码器控制，角度制，30.0~-35.0f，下正上负）

  f32 gravity_compensation_ = 0.0f;    // 重力补偿值
  f32 k_gravity_compensation_ = -0.8f;  // 重力补偿系数

  f32 ammo_speed_ = 8000.0f;  // 摩擦轮速度 9000.0f -> 25m/s 初速度

  u8 shoot_num_ = 0;                        // 开火次数
  u16 last_remain_bullet_ = 0;              // 上一次剩余子弹数
  f32 shoot_initial_speed_[10] = {};        // 子弹初速度
  f32 shoot_initial_average_speed_ = 0.0f;  // 子弹初速度平均值
  f32 target_shoot_initial_speed_ = 30.0f;  // 目标子弹初速度

  f32 shoot_frequency_ = 0.0f;

  u16 heat_limit_ = 0;    // 热量上限值
  u16 heat_current_ = 0;  // 热量实时值

  bool shoot_flag_ = false;         // 开火标志
  bool single_shoot_flag_ = false;  // 单发标志

  bool DM_enable_flag_ = false;  // 4310电机使能标志

  bool down_yaw_target_refresh_flag_ = false;  // 云台下部yaw轴目标数据刷新标志

  bool scan_yaw_flag_ = false;    // 扫描yaw轴方向标识位
  bool scan_pitch_flag_ = false;  // 扫描pitch轴方向标识位

  bool DF_flag_ = false;   // 大符标志
  bool XF_flag_ = false;   // 小符标志
  bool DF_state_ = false;  // 大符状态
  bool XF_state_ = false;  // 小符状态

  const f32 sensitivity_yaw_ = 0.01f;      // 云台上部yaw轴灵敏度
  const f32 sensitivity_pitch_ = 0.01f;     // 云台pitch轴灵敏度
  const f32 highest_pitch_angle_ = 0.4f;     // 云台pitch轴最高（弧度制）
  const f32 lowest_pitch_angle_ = -0.6f;    // 云台pitch轴最低（弧度制）

 public:
  void GimbalInit();

  void GimbalTask();

 private:
  void GimbalStateUpdate();

  void GimbalRCTargetUpdate();

  void GimbalScanTargetUpdate();

  void GimbalAimbotTargetUpdate();

  void GimbalDownYawFollow();

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
