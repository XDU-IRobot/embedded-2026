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

  bool DM_enable_flag_ = false;  // 4310电机使能标志

  const f32 sensitivity_ = 0.002f;     // 云台pitch轴灵敏度 0.3f
  const f32 highest_pitch_angle_ = 0.54f;    // 云台pitch轴最高 0.54f（弧度制）
  const f32 lowest_pitch_angle_ = -0.38f;    // 云台pitch轴最低 0.38f（弧度制）

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

  void AmmoSpeedUpdate();

  void SetMotorCurrent();
} *gimbal;

#endif  // GIMBAL_HPP
