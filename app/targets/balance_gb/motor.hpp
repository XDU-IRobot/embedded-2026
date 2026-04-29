#pragma once

#include <librm.hpp>
#include "aimbot_comm_can.hpp"
#include "controllers/gimbal_2dof.hpp"
#include "controllers/shoot_3fric.hpp"
#include "SineSweep.hpp"
#include "yaw_speed_feedforward.hpp"
#include "encoder_counter.hpp"

using namespace rm;
using namespace rm::device;

class Motor {
 public:
  rm::hal::Can *can1{nullptr}, *can2{nullptr};  ///< CAN 总线接口

  DmMotor<DmMotorControlMode::kMit> *yaw_motor{nullptr};
  DmMotor<DmMotorControlMode::kMit> *pitch_motor{nullptr};
  M3508 *dial_motor{nullptr};
  M3508 *ammo_left{nullptr};  ///< 左侧摩擦轮电机
  M3508 *ammo_right{nullptr};
  AimbotCanCommunicator *aimbot_comm{nullptr};
  Gimbal2Dof gimbal_controller;                  ///< 二轴双 Yaw 云台控制器
  Shoot3Fric shoot_controller{9, 42.75, false};  ///< 摩擦轮
  YawSpeedFeedforward *yaw_feedforward{nullptr};
  SineSweep *sweep_controller{nullptr};
  EncoderCounter dail_encoder_counter;
  DeviceManager<2> device_aimbot;  // 自瞄设备
  // modules::VofaPlotter *vofa_plotter{nullptr};

  enum class InitFlag {
    kNormal,   // 正常模式
    kOpposite  // 倒地自启时头向不会撞枪管的一面初始化
  };

  // InitFlag init_mode{InitFlag::kNormal};

 public:
  f32 rc_request_pitch = 0.f;
  f32 rc_request_yaw = 0.f;
  f32 rc_request_yaw_temp = 0.f;

  f32 yaw_motor_pos = 0.f;

  f32 gravity_compensation_ = 0.f, yaw_compensation_ = 0.f;

  int single_shoot_time_ = 200, single_shoot_temp = 0;

  bool reset_yaw_flag = false;
  bool single_flag = false;  // 单发标志

  bool change_yaw_init_flag = false;

  void MotorInit();  ///< 电机初始化

  void DMEnable();      ///< 达妙使能
  void DMDisable();     ///< 达妙失能
  void ShootEnable();   ///< 发射机构使能
  void ShootDisable();  ///< 发射机构失能

  void DMInitControl();       ///< 达妙电机初始化控制
  void DMAutoControl();       ///< 自瞄云台电机跟随
  void DMAimControl();        ///< 自瞄自动跟随
  void ShootNormalControl();  ///< 发射机构正常控制更新
  void ShootAutoControl();    ///< 发射机构自瞄控制更新
  void ShootAutoFuControl();  ///< 发射机构打符控制更新
  void ShooterCounter();      ///< 弹丸计数
  void FricSpeedUpdate();     ///< 摩擦轮转速更新
  void HeatUpdate();          ///< 热量闭环的热量更新

  void SendDMCommand();   ///<  发送达妙电机控制量
  void SendDjiCommand();  ///<  发送大疆电机控制量

  void MotorPidInit();

  void CalcYawPos(f32 pos);

  void Transit_initmode(bool keyboard_e);

  f32 yaw_init = 0.f;

  bool DMEnable_ = true;

 private:
  int shoot_number = 0;  // 发射的子弹总数

  f32 left_set_speed = 0.f, right_set_speed = 0.f;
  f32 left_fric_speed_max = 7000.f, left_fric_speed_min = 6000.f;
  f32 right_fric_speed_max = 7000.f, right_fric_speed_min = 6000.f;

  f32 pitch_init = -18.f;
  f32 reset_yaw = 0.f;

  f32 shoot_frequency = 0.f;

  f32 yaw_pos_kp = 0.f, yaw_pos_ki = 0.f, yaw_pos_kd = 0.f;
  f32 yaw_vel_kp = 0.f, yaw_vel_ki = 0.f, yaw_vel_kd = 0.f;
  f32 pitch_pos_kp = 0.f, pitch_pos_ki = 0.f, pitch_pos_kd = 0.f;
  f32 pitch_vel_kp = 0.f, pitch_vel_ki = 0.f, pitch_vel_kd = 0.f;

  u16 heat_ultimate;     // 计算得出的最终热量
  float heat_ultimate_;  // 计算得出的最终热量

  i16 o1 = 0;
  i16 o2 = 0;
  i16 o3 = 0;

  i16 last_right_rpm = 0;  // 右摩擦轮上次转速

  bool dm_enabled_{false};
  bool shoot_enabled_{false};
  bool single_shoot_flag_{false};
  bool fric_on_flag_{false};      // 摩擦轮启动且达到目标转速
  bool fric_reduce_flag_{false};  // 摩擦轮降速标志
  bool shoot_one_flag_{false};    // 打弹标志
  bool last_is_three = false;     // 上一周期是否处于“发三”状态
  int shoot_cycle_counter_ = 0;   // 距离上次发射的周期数（仅在持续3状态时累加）
};