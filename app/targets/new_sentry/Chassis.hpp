#ifndef CHASSIS_HPP
#define CHASSIS_HPP

#include <librm.hpp>

#include "main.hpp"

using namespace rm;

inline class Chassis {
 public:
  StateMachineType ChassisMove_ = {kNoForce};  // 底盘运动状态
 private:
  rm::modules::PID chassis_follow_pid_;

  f32 chassis_receive_x_ = 0.0f;  // 底盘x轴接收值
  f32 chassis_receive_y_ = 0.0f;  // 底盘y轴接收值
  f32 chassis_target_x_ = 0.0f;   // 底盘x轴目标值
  f32 chassis_target_y_ = 0.0f;   // 底盘y轴目标值
  f32 chassis_target_w_ = 0.0f;   // 底盘转动目标值

  f32 down_yaw_delta_ = 0.0f;  // 下部yaw轴随动差值

  f32 k_speed_power_limit_ = 1.0f;  // 底盘速度限制系数

  f32 chassis_move_delta_angle_ = 0.5f;

  bool rotate_flag_ = false;  // 小陀螺模式标识位

  const f32 front_down_yaw_angle_ = -0.835f;  // 前方下部yaw轴角度

  const f32 chassis_sensitivity_xy_ = 5000.0f;  // 底盘x、y轴灵敏度
  const f32 chassis_max_speed_xy_ = 10000.0f;    // 底盘x、y轴最大速度
  const f32 chassis_max_speed_w_ = 15000.0f;     // 底盘转动最大速度
  const f32 chassis_max_navigate_xyw_ = 1.0f;     // 底盘x、y、z轴导航最速度

 public:
  void ChassisInit();

  void ChassisTask();

 private:
  void ChassisStateUpdate();

  void ChassisRCDataUpdate();

  void ChassisNavigateDataUpdate();

  void ChassisMovePIDUpdate();

  void ChassisMatchUpdate();

  void ChassisEnableUpdate();

  void ChassisDisableUpdate();

  void PowerLimitLoop();

  void SetMotorCurrent();
} *chassis;

#endif  // CHASSIS_HPP
