#ifndef GIMBAL_HPP
#define GIMBAL_HPP

#include <librm.hpp>

#include "main.hpp"

using namespace rm;

inline class Gimbal {
 public:
  StateMachineType GimbalMove_ = {kNoForce};  // 云台运动状态

 private:
  f32 gimbal_yaw_target_ = 0.0f;  // 云台yaw轴目标数据（编码器控制，弧度制，-pi~pi，左正右负）
  f32 gimbal_pitch_target_ = 0.0f;  // 云台pitch轴目标数据（编码器控制，弧度制，-0.675f.0~0.615f，下正上负）

  bool DM_enable_flag_ = false;  // 4310电机使能标志

  const f32 sensitivity_ = 0.004f;          // 云台灵敏度 0.004f
  const f32 highest_pitch_angle_ = 0.615f;  // 云台pitch轴最高 0.615f（弧度制）
  const f32 lowest_pitch_angle_ = -0.675f;  // 云台pitch轴最低 -0.675f（弧度制）

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
