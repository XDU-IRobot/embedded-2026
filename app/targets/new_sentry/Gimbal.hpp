#ifndef GIMBAL_HPP
#define GIMBAL_HPP

#include <librm.hpp>

#include "main.hpp"

using namespace rm;

inline class Gimbal {
 public:
  StateMachineType GimbalMove_ = {kNoForce};  // 云台运动状态

  f32 pitch_torque_ = 0.0f;  // pitch轴力矩数据

  const f32 mid_up_yaw_angle_ = 2840.0f;  // 云台上部yaw轴中值 2840.0f（编码器值）

 private:
  f32 gimbal_up_yaw_target_ = 0.0f;  // 云台上部yaw轴目标数据（编码器控制，编码器制，1500.0~3500.0f，左正右负）
  f32 gimbal_down_yaw_target_ = 0.0f;  // 云台下部yaw轴目标数据（陀螺仪控制，弧度制，0~2pi，左正右负）
  f32 gimbal_pitch_target_ = 0.0f;  // 云台pitch轴目标数据（编码器控制，角度制，30.0~-35.0f，下正上负）

  f32 gravity_compensation_ = 0.0f;    // 重力补偿值
  f32 k_gravity_compensation_ = 1.0f;  // 重力补偿系数

  f32 ammo_speed_ = 7800.0f;  // 摩擦轮速度 9000.0f -> 25m/s 初速度

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

  const f32 ammo_init_speed_ = 7800.0f;      // 摩擦轮初始速度 6000.0f
  const f32 sensitivity_up_yaw_ = 3.0f;      // 云台上部yaw轴灵敏度 3.0f
  const f32 sensitivity_down_yaw_ = 0.004f;  // 云台下部yaw轴灵敏度 0.3f
  const f32 sensitivity_pitch_ = 0.002f;     // 云台pitch轴灵敏度 0.3f
  const f32 highest_pitch_angle_ = 0.2f;     // 云台pitch轴最高 0.54f（弧度制）
  const f32 lowest_pitch_angle_ = -0.38f;    // 云台pitch轴最低 0.38f（弧度制）
  const f32 max_up_yaw_angle_ = 4050.0f;     // 云台上部yaw轴最大 4050.0f（编码器值）
  const f32 min_up_yaw_angle_ = 1650.0f;     // 云台上部yaw轴最小 1650.0f（编码器值）
  const f32 down_yaw_move_high_ = 3700.0f;   // 云台上部yaw轴大于高值，云台下部yaw轴随动
  const f32 down_yaw_move_low_ = 2000.0f;    // 云台上部yaw轴小于低值，云台下部yaw轴随动

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
