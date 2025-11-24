#ifndef BOARDC_MAIN_HPP
#define BOARDC_MAIN_HPP

#include <librm.hpp>

#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "device_manager.hpp"
#include "can.h"
#include "usart.h"
#include "spi.h"
#include "timer_task.hpp"
#include "controllers/gimbal_2dof.hpp"
#include "controllers/shoot_2firc.hpp"

// freeMaster调试变量
double Ayaw;
double Apitch;
double Aroll;
double Aoutputyaw;
double Aoutputpitch;
double Arcyawdata;
double Arcpitchdata;

class Gimbal {
 public:
  // 状态机
  typedef enum {
    kNoForce,  // 无力
    kManual,   // 手动
    kAuto,     // 自瞄

    kStop,   // 无力
    kReady,  // 准备开火
    kFire    // 开火
  } StateMachineType;

  Buzzer *buzzer{nullptr};  ///< 蜂鸣器
  rm::modules::BuzzerController<rm::modules::buzzer_melody::Silent, rm::modules::buzzer_melody::Startup,
                                rm::modules::buzzer_melody::Success, rm::modules::buzzer_melody::Error,
                                rm::modules::buzzer_melody::SuperMario, rm::modules::buzzer_melody::SeeUAgain,
                                rm::modules::buzzer_melody::TheLick>
      buzzer_controller;  ///< 蜂鸣器控制器
  LED *led{nullptr};      ///< RGB LED灯
  rm::modules::RgbLedController<rm::modules::led_pattern::Off, rm::modules::led_pattern::RedFlash,
                                rm::modules::led_pattern::GreenBreath,
                                rm::modules::led_pattern::RgbFlow>
      led_controller;  ///< RGB LED控制器

  rm::hal::Can *can1{nullptr};     ///< CAN 总线接口
  rm::hal::Serial *dbus{nullptr};  ///< 遥控器串口接口

  DeviceManager<1> device_rc;  ///< 设备管理器，维护所有设备在线状态
  DeviceManager<2> device_gimbal;
  DeviceManager<3> device_shoot;

  rm::device::BMI088 *imu{nullptr};  ///< IMU
  rm::modules::MahonyAhrs ahrs{1000.0f};

  rm::device::DR16 *rc{nullptr};  ///< 遥控器

  rm::device::GM6020 *yaw_motor{nullptr};                                           ///< 云台 Yaw 上电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr};  ///< 云台 Pitch 电机
  rm::device::M3508 *friction_left{nullptr};                                        ///< 左侧摩擦轮电机
  rm::device::M3508 *friction_right{nullptr};                                       ///< 右侧摩擦轮电机
  rm::device::M2006 *dial_motor{nullptr};                                           ///< 拨盘电机

  StateMachineType AmmoState_ = {kNoForce};  // 当前状态
  StateMachineType GimbalState_ = {kStop};   // 云台运动状态

  Gimbal2Dof gimbal_controller;           ///< 二轴双 Yaw 云台控制器
  Shoot2Fric shoot_controller{8, 36.0f};  ///< 三摩擦轮发射机构控制器，8发拨盘
  bool DM_is_enable = false;
  int time_ = 0;  // 系统心跳

  double rc_yaw_data = 0;
  double rc_pitch_data = 0;

  double yaw = 0;
  double roll = 0;
  double pitch = 0;

  // 结构体初始化
  void GimbalInit() {
    buzzer = new Buzzer;
    led = new LED;

    can1 = new rm::hal::Can{hcan1};
    dbus = new rm::hal::Serial{huart3, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};

    imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};

    rc = new rm::device::DR16{*dbus};
    yaw_motor = new rm::device::GM6020{*can1, 2};
    pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
        *can1, {0x01, 0x07, 20.0f, 0.2f, 2.0f, std::make_pair(0.0f, 5.0f), std::make_pair(0.0f, 5.0f)}};
    friction_left = new rm::device::M3508{*can1, 4};
    friction_right = new rm::device::M3508{*can1, 3};
    dial_motor = new rm::device::M2006{*can1, 1};

    device_rc << rc;                                                // 遥控器
    device_gimbal << yaw_motor << pitch_motor;                      // 云台电机
    device_shoot << friction_left << friction_right << dial_motor;  // 发射机构电机

    can1->SetFilter(0, 0);
    can1->Begin();
    rc->Begin();
    buzzer->Init();
    led->Init();

    led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
    buzzer_controller.Play<rm::modules::buzzer_melody::TheLick>();

    time_ = 0;

    GimbalPIDInit();

    gimbal_controller.Enable(false);
    pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
  }

  // 云台pid初始化
  void GimbalPIDInit() {
    gimbal_controller.pid().yaw_position.SetKp(10.0f);  // 位置环 0.25f 0.0f 0.025f
    gimbal_controller.pid().yaw_position.SetKi(0.0f);
    gimbal_controller.pid().yaw_position.SetKd(0.0f);
    gimbal_controller.pid().yaw_position.SetMaxOut(20000.0f);
    gimbal_controller.pid().yaw_position.SetMaxIout(1000.0f);
    gimbal_controller.pid().yaw_speed.SetKp(1000.0f);  // 速度环 520.0f 0.0f 50.0f
    gimbal_controller.pid().yaw_speed.SetKi(0.0f);
    gimbal_controller.pid().yaw_speed.SetKd(0.0f);
    gimbal_controller.pid().yaw_speed.SetMaxOut(20000.0f);
    gimbal_controller.pid().yaw_speed.SetMaxIout(1000.0f);

    gimbal_controller.pid().pitch_position.SetKp(0.5f);  // 位置环 15.0f 0.0f 1.0f
    gimbal_controller.pid().pitch_position.SetKi(0.0f);
    gimbal_controller.pid().pitch_position.SetKd(0.0f);
    gimbal_controller.pid().pitch_position.SetMaxOut(20.0f);
    gimbal_controller.pid().pitch_position.SetMaxIout(10.0f);
    gimbal_controller.pid().pitch_speed.SetKp(2.0f);  // 速度环 1.6f 0.0f 4.5f
    gimbal_controller.pid().pitch_speed.SetKi(0.0f);
    gimbal_controller.pid().pitch_speed.SetKd(0.0f);
    gimbal_controller.pid().pitch_speed.SetMaxOut(20.0f);
    gimbal_controller.pid().pitch_speed.SetMaxIout(10.0f);
  }

  // 发射机构pid初始化
  void AmmoPIDInit() {
    shoot_controller.pid().fric_1_speed.SetKp(8.0f);  // 速度环 8.0f 0.0f 0.0f
    shoot_controller.pid().fric_1_speed.SetKi(0.0f);
    shoot_controller.pid().fric_1_speed.SetKd(0.0f);
    shoot_controller.pid().fric_1_speed.SetMaxOut(2000.0f);
    shoot_controller.pid().fric_1_speed.SetMaxIout(1000.0f);
    shoot_controller.pid().fric_2_speed.SetKp(8.0f);  // 速度环 8.0f 0.0f 0.0f
    shoot_controller.pid().fric_2_speed.SetKi(0.0f);
    shoot_controller.pid().fric_2_speed.SetKd(0.0f);
    shoot_controller.pid().fric_2_speed.SetMaxOut(2000.0f);
    shoot_controller.pid().fric_2_speed.SetMaxIout(1000.0f);
    shoot_controller.pid().loader_position.SetKp(500.0f);  // 位置环 0.0f 0.0f 0.0f
    shoot_controller.pid().loader_position.SetKi(0.0f);
    shoot_controller.pid().loader_position.SetKd(10.0f);
    shoot_controller.pid().loader_position.SetMaxOut(10000.0f);
    shoot_controller.pid().loader_position.SetMaxIout(0.0f);
    shoot_controller.pid().loader_speed.SetKp(5.0f);  // 速度环 5.0f 0.0f 1.0f
    shoot_controller.pid().loader_speed.SetKi(0.0f);
    shoot_controller.pid().loader_speed.SetKd(0.0f);
    shoot_controller.pid().loader_speed.SetMaxOut(10000.0f);
    shoot_controller.pid().loader_speed.SetMaxIout(2000.0f);
  }

  // 遥控器状态更新
  void RCStateUpdate() {
    if (!device_rc.all_device_ok()) {
      GimbalState_ = kNoForce;
      AmmoState_ = kStop;
    } else {
      switch (rc->switch_r()) {
        case rm::device::DR16::SwitchPosition::kUp:
          AmmoState_ = kFire;
          break;
        case rm::device::DR16::SwitchPosition::kMid:
          AmmoState_ = kReady;
          break;
        default:
          AmmoState_ = kStop;
          break;
      }
      switch (rc->switch_l()) {
        case rm::device::DR16::SwitchPosition::kUp:
          GimbalState_ = kAuto;
          break;
        case rm::device::DR16::SwitchPosition::kMid:
          GimbalState_ = kManual;
          break;
        default:
          GimbalState_ = kNoForce;
          break;
      }
    }
  }

  // 云台控制
  void GimbalControl() {
    // 手动控制
    if (GimbalState_ == kManual) {
      if (DM_is_enable == false) {  // 使达妙电机使能
        pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
        DM_is_enable = true;
        rc_yaw_data = yaw;
        rc_pitch_data = pitch;
        gimbal_controller.Enable(true);
      }
      rc_yaw_data += rm::modules::Map(rc->left_x(), -660, 660, -0.3f, 0.3f);  // 遥控器映射yaw轴
      rc_yaw_data = rm::modules::Wrap(rc_yaw_data, 0, 360);                   // 遥控器周期限制

      rc_pitch_data += rm::modules::Map(rc->left_y(), -660, 660, -0.3f, 0.3f);  // 遥控器映射yaw轴
      rc_pitch_data = rm::modules::Clamp(rc_pitch_data, 150, 190);              // 遥控器周期限制

      gimbal_controller.SetTarget(rc_yaw_data * M_PI / 180.0f, rc_pitch_data * M_PI / 180.0f);
      gimbal_controller.Update(yaw * M_PI / 180.0f, yaw_motor->rpm(), pitch * M_PI / 180.0f, pitch_motor->vel());

      yaw_motor->SetCurrent(gimbal_controller.output().yaw);
    }

    // 自瞄控制
    else if (GimbalState_ == kAuto) {
      if (DM_is_enable == false) {  // 使达妙电机使能
        pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
        DM_is_enable = true;
        rc_yaw_data = yaw;
        rc_pitch_data = pitch;
        gimbal_controller.Enable(true);
      }
    }

    // 无力或遥控器无信号
    else {
      if (DM_is_enable == true) {  // 使达妙电机使能
        pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
        DM_is_enable = false;
        gimbal_controller.Enable(false);
      }
    }
  }

  // 发射机构控制
  void AmmoControl() {
    if (AmmoState_ == kReady) {
    } else if (AmmoState_ == kManual) {
    } else {
    }
  }

  void FreeMasterDebug() {
    Arcyawdata = rc_yaw_data;
    Arcpitchdata = rc_pitch_data;
    Ayaw = yaw;
    Apitch = pitch;
    Aroll = roll;
    Aoutputyaw = gimbal_controller.output().yaw;
    Aoutputpitch = gimbal_controller.output().pitch;
  }

  // 遥控器和imu数据解算+DjiMotor发信息
  void SubLoop500Hz() {
    RCStateUpdate();  // 遥控器更新
    GimbalControl();  // 云台控制更新
    AmmoControl();    // 发射机构数据更新
    rm::device::DjiMotor<>::SendCommand(*can1);

    imu->Update();
    ahrs.Update(rm::modules::ImuData6Dof{-imu->gyro_x(), -imu->gyro_y(), imu->gyro_z(), -imu->accel_x(),
                                         -imu->accel_y(), imu->accel_z()});
    roll = -57.3 * ahrs.euler_angle().roll + 180;  // 云台正向靠近Yaw的0°和360°边界，顺时针从0°增大到360°
    yaw = -57.3 * ahrs.euler_angle().yaw + 180;  // 云台始终处于Roll的0°和360°边界，顺时针从0°增大到360°(不限位)
    pitch = -57.3 * ahrs.euler_angle().pitch + 180;  // 云台仰起Pitch增大，低头Pitch减小(有限位))
  }

  // DmMotor电机发信息
  void SubLoop250Hz() {
    if (time_ % 2 == 0) {
      pitch_motor->SetPosition(0, 0, -gimbal_controller.output().pitch, 0, 0);
    }
    FreeMasterDebug();
  }

  // usb收发数据
  void SubLoop100Hz() {
    if (time_ % 5 == 0) {
    }
  }

  // 裁判系统+UI绘制
  void SubLoop50Hz() {
    if (time_ % 10 == 0) {
    }
  }

  // 总循环
  void SubLoop10Hz() {
    if (time_ % 50 == 0) {
      time_ = 0;
    }
  }
};

#endif  // BOARDC_MAIN_HPP