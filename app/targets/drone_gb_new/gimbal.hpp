#ifndef BOARDC_GIMBAL_HPP
#define BOARDC_GIMBAL_HPP
// 最简控制单元测试
#include <librm.hpp>
#include "can.h"
#include "usart.h"
#include "spi.h"
#include "timer_task.hpp"
#include "ControllerPidGimbal.hpp"
#include "ControllerPidAmmo.hpp"

class Gimbal {
 public:
  double yaw = 0;              // imu yaw数据
  double roll = 0;             // imu roll数据
  double pitch = 0;            // imu pitch数据
  double rc_yaw_data = 0;      // 遥控器yaw数据
  double rc_pitch_data = 0;    // 遥控器pitch数据
  bool DM_is_enable = false;   // 达秒使能标志位
  float pitch_min_pos = 2.94;  // pitch电机最小限位
  float pitch_max_pos = 4.15;  // pitch电机最大限位

  rm::hal::Can *can1{nullptr};     // CAN 总线接口
  rm::hal::Serial *dbus{nullptr};  // 遥控器串口接口

  rm::device::DeviceManager<1> device_rc;      // 遥控管理器，维护所有设备在线状态
  rm::device::DeviceManager<2> device_gimbal;  // 云台管理器
  rm::device::DeviceManager<3> device_shoot;   // 发射管理器

  int time_ = 0;  // 系统心跳

  rm::device::BMI088 *imu{nullptr};      // IMU
  rm::modules::MahonyAhrs ahrs{500.0f};  // TODO Mahony滤波控制频率

  rm::device::GM6020 *yaw_motor{nullptr};                                           // 云台 Yaw 上电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr};  // 云台 Pitch 电机
  rm::device::M3508 *friction_left{nullptr};                                        // 左侧摩擦轮电机
  rm::device::M3508 *friction_right{nullptr};                                       // 右侧摩擦轮电机
  rm::device::M2006 *dial_motor{nullptr};                                           // 拨盘电机

  rm::device::DR16 *rc{nullptr};  // 遥控器

  typedef enum {
    kNoForce,          // 云台无力
    kManual,           // 云台手动
    kAuto,             // 云台自瞄
    kStop,             // 发射机构无力
    kReady,            // 发射机构准备开火
    kFire              // 发射机构开火
  } StateMachineType;  // 遥控器状态机

  StateMachineType AmmoState_ = {kNoForce};  // 初始化发射机构状态
  StateMachineType GimbalState_ = {kStop};   // 初始化云台运动状态

  Gimbal2Dof gimbal_controller;  // 二轴云台PID控制器
  Shoot2Fric shoot_controller;   // 双摩擦轮发射机构控制器

  void GimbalInit() {
    time_ = 0;  // 系统心跳置0
    can1 = new rm::hal::Can{hcan1};
    dbus = new rm::hal::Serial{huart3, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};

    imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
    rc = new rm::device::DR16{*dbus};

    yaw_motor = new rm::device::GM6020{*can1, 2};
    pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
        *can1, {0x00, 0x05, 10.0f, 20.0f, 10.0f, {0.0f, 10.0f}, {0.0f, 5.0f}}};

    friction_left = new rm::device::M3508{*can1, 4};
    friction_right = new rm::device::M3508{*can1, 3};
    dial_motor = new rm::device::M2006{*can1, 1};

    device_rc << rc;                                                // 遥控器
    device_gimbal << yaw_motor << pitch_motor;                      // 云台电机
    device_shoot << friction_left << friction_right << dial_motor;  // 发射机构电机

    can1->SetFilter(0, 0);//设置滤波？
    can1->Begin();
    rc->Begin();

    GimbalPIDInit();
    AmmoPIDInit();

    gimbal_controller.Enable(false);
    pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);

    shoot_controller.Enable(false);                   // 开启控制器
    shoot_controller.Arm(false);                      // 摩擦轮武装（允许转动）
    shoot_controller.SetMode(Shoot2Fric::kFullAuto);  // 连发模式
    shoot_controller.SetLoaderSpeed(0.0f);            // 拨盘目标线速度
    shoot_controller.SetArmSpeed(0.0f);               // 摩擦轮目标线速度

    // pitch_cmd_notch.initNotch(250.0, 3.15, 15);//控制频率 陷频频率 品质因数
    // pitch_chirp.init(250, 0.5, 100.0, 20.0, 0.01, ChirpType::kLog);// 控制频率 扫频区间f0-f1 周期 幅度
  }
};

#endif  // BOARDC_GIMBAL_HPP
