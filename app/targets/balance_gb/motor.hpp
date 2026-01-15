#pragma once

#include <librm.hpp>

#include "controllers/gimbal_2dof.hpp"
#include "controllers/shoot_2firc.hpp"

using namespace rm;
using namespace rm::device;

class Motor {
 public:
  rm::hal::Can *can1{nullptr}, *can2{nullptr};  ///< CAN 总线接口

  DmMotor<DmMotorControlMode::kMit> *yaw_motor{nullptr};
  DmMotor<DmMotorControlMode::kMit> *pitch_motor{nullptr};
  M2006 *dial_motor{nullptr};
  M3508 *ammo_left{nullptr};  ///< 左侧摩擦轮电机
  M3508 *ammo_right{nullptr};

  Gimbal2Dof gimbal_controller;           ///< 二轴双 Yaw 云台控制器
  Shoot2Fric shoot_controller{8, 36.0f};  ///< 摩擦轮
 public:
  void MotorInit();

  void DMEnable();
  void DMDisable();
  void ShootEnable(bool enable);

  void DMInit();
  void DMControl();

  void ShootControl();

 private:
  f32 rc_request_pitch = 0.f;
  f32 rc_request_yaw = 0.f;

  f32 pitch_init = 0.f;
  f32 yaw_init = 1.6f;
  f32 reset_yaw = 0.f;

  i16 init_count = 0;

  bool reset_yaw_flag = false;

  bool DMEnable_ = false;
  bool dm_enabled_{false};
  bool shoot_enabled_{false};

  void MotorPidInit();
};