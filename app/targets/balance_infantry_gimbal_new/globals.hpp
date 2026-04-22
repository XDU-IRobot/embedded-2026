#pragma once

#include <librm.hpp>

#include "can.h"
#include "usart.h"
#include "spi.h"

#include "buzzer.hpp"
#include "rgb_led.hpp"
#include "controllers/gimbal_2dof.hpp"
#include "aimbot_comm_can.hpp"
#include "dynamics.hpp"
#include "parameter_config.hpp"
#include "communiate.hpp"
/**
 * @brief   全局数据仓库
 */
struct Globals {
  int loop_divisor{0};
  rm::hal::Can can1{hcan1}, can2{hcan2};
  rm::hal::Serial sbus_serial{huart3, 18, rm::hal::stm32::UartMode::kNormal,
                              rm::hal::stm32::UartMode::kDma};  ///< 用于接收遥控器信号的串口
  //  rm::hal::Serial imu_serial{huart1, 518, rm::hal::stm32::UartMode::kNormal,
  //                             rm::hal::stm32::UartMode::kDma};  ///< 用于接收遥控器信号的串口
  //
  //  rm::device::HipnucImu imu{imu_serial};

  rm::device::BMI088 imu{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};  ///< bmi088
  rm::modules::MahonyAhrs ahrs{500.0f};  ///< 姿态解算器

  rm::device::DR16 rc{sbus_serial};

  Buzzer buzzer;  ///< 蜂鸣器
  rm::modules::BuzzerController<rm::modules::buzzer_melody::Silent, rm::modules::buzzer_melody::Beeps<1>,
                                rm::modules::buzzer_melody::Beeps<2>, rm::modules::buzzer_melody::Startup,
                                rm::modules::buzzer_melody::Error, rm::modules::buzzer_melody::Success,
                                rm::modules::buzzer_melody::TheLick>
      buzzer_controller;
  LED led;                                                              ///< WS2812 LED灯
  rm::modules::RgbLedController<rm::modules::led_pattern::Off,          //
                                rm::modules::led_pattern::GreenBreath,  //
                                rm::modules::led_pattern::RedFlash,     //
                                rm::modules::led_pattern::RgbFlow>
      led_controller;  ///< LED控制器

  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> yaw_motor{
      can1, {0x10, 0x09, M_PI, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}}};  ///< 云台 Yaw 上电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> pitch_motor{
      can2, {0x05, 0x04, M_PI, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}}};  ///< 云台 Pitch 电机
  Gimbal2Dof gimbal_controller;
  Gimbal2DofDynamics dynamics_;
  rm::device::AimbotCanCommunicator aimbot_comm{can1};  /// 自瞄控制器object
  ChassisCommunicator chassis_comm{can1, 0x119};        ///<  底盘控制器object
  rm::device::DeviceManager<10> device_manager;         ///< 设备管理器
  etl::string<512> status_str{""};                      ///< 描述当前系统状态的字符串

  u_int8_t time_camera{0};  // 摄像头计数器
  u_int16_t imu_count{0};   // IMU计数器
  //  rm::modules::PID yaw_pos_pid {pid_config::kYawPosP,pid_config::kYawPosI,pid_config::kYawPosD,10.f,0.f};
  //  rm::modules::PID yaw_spd_pid { pid_config::kYawSpdP,pid_config:
  //  :kYawSpdI,pid_config::kYawSpdD,6.f,0.f};
  //
  //  rm::modules::PID pitch_pos_pid {pid_config::kPitchPosP,pid_config::kPitchPosI,pid_config::kPitchPosD,10.0f,0.f};
  //  rm::modules::PID pitch_spd_pid { pid_config::kPitchSpdP,pid_config::kPitchSpdI,pid_config::kPitchSpdD,6.0f,0.f};

  //      gimbal_controller.pid().yaw_position.SetKp(800.0f).SetKi(0.0f).SetKd(24000.0f).SetMaxOut(30000.0f).SetMaxIout(0.0f);
  //  gimbal_controller.pid().yaw_speed.SetKp(350.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(30000.0f).SetMaxIout(0.0f);
  /**
   * @brief 初始化所有全局对象
   */
  void Init() {
    // 给设备起一下名字方便看哪个离线
    rc.SetName("rc");

    // 注册设备状态变化事件回调
    device_manager.OnDeviceStatusChange([&](rm::device::Device *device) {});

    // 把设备都注册给设备管理器
    device_manager << &rc;

    buzzer.Init();
    led.Init();
    rc.Begin();
    //    imu.Begin();
    can1.SetFilter(0, 0);
    can2.SetFilter(0, 0);
    can1.Begin();
    can2.Begin();

    //  yaw电机pid
    gimbal_controller.pid()
        .yaw_position.SetKp(pid_config::kYawPosP)
        .SetKi(pid_config::kYawPosI)
        .SetKd(pid_config::kYawPosD)
        .SetMaxOut(10.f)
        .SetMaxIout(0.4f);
    gimbal_controller.pid()
        .yaw_speed.SetKp(pid_config::kYawSpdP)
        .SetKi(pid_config::kYawSpdI)
        .SetKd(pid_config::kYawSpdD)
        .SetMaxOut(6.f)
        .SetMaxIout(0.f);

    // pitch电机pid
    gimbal_controller.pid()
        .pitch_position.SetKp(pid_config::kPitchPosP)
        .SetKi(pid_config::kPitchPosI)
        .SetKd(pid_config::kPitchPosD)
        .SetMaxOut(10.f)
        .SetMaxIout(0.4f);
    gimbal_controller.pid()
        .pitch_speed.SetKp(pid_config::kPitchSpdP)
        .SetKi(pid_config::kPitchSpdI)
        .SetKd(pid_config::kPitchSpdD)
        .SetMaxOut(6.f)
        .SetMaxIout(0.f);
  }
};

extern Globals *globals;