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
    M3508 *dial_motor{nullptr};
    M3508 *ammo_left{nullptr};                                           ///< 左侧摩擦轮电机
    M3508 *ammo_right{nullptr};

    Gimbal2Dof gimbal_controller;                      ///< 二轴双 Yaw 云台控制器
    Shoot2Fric shoot_controller{8, 36.0f};                  ///< 摩擦轮
  public:
    void MotorInit();   ///< 电机初始化

    void DMEnable();    ///< 达妙使能
    void DMDisable();   ///< 达妙失能
    void ShootEnable(); ///< 发射机构使能
    void ShootDisable();///< 发射机构失能

    void DMInitControl();      ///< 达妙电机初始化控制
    void DMControl();          ///< 达妙电机正常控制更新

    void ShootControl();       ///< 发射机构正常控制更新

    void SendDMCommand();      ///<  发送达妙电机控制量
    void SendDjiCommand();     ///<  发送大疆电机控制量

  void MotorPidInit();

  private:
    f32 rc_request_pitch = 0.f;
    f32 rc_request_yaw = 0.f;

    f32 pitch_init = 0.f;
    f32 yaw_init = 1.6f;
    f32 reset_yaw = 0.f;

    i16 init_count = 0;

    bool reset_yaw_flag = false;

    bool DMEnable_ = true;
    bool dm_enabled_{false};
    bool shoot_enabled_{false};





};