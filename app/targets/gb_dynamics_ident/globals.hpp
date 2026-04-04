#pragma once

#include <librm.hpp>

#include "can.h"
#include "usart.h"

#include "buzzer.hpp"
#include "rgb_led.hpp"
#include "gimbal_2dof.hpp"

/**
 * @brief   全局数据仓库
 */
struct Globals {
  rm::hal::Can can1{hcan1}, can2{hcan2};
  rm::hal::Serial sbus_serial{huart3, 18, rm::hal::stm32::UartMode::kNormal,
                              rm::hal::stm32::UartMode::kDma};  ///< 用于接收遥控器信号的串口
  rm::hal::Serial imu_serial{huart1, 518, rm::hal::stm32::UartMode::kNormal,
                             rm::hal::stm32::UartMode::kDma};  ///< 用于接收遥控器信号的串口

  rm::device::HipnucImu imu{imu_serial};

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

  rm::device::DmMotorMit yaw_motor{can1, {0x12, 0x02, 3.141593f, 30.0f, 10.0f, {0.f, 500.f}, {0.f, 5.f}}};
  rm::device::DmMotorMit pitch_motor{can1, {0x11, 0x01, 3.141593f, 30.0f, 10.0f, {0.f, 500.f}, {0.f, 5.f}}};

  rm::device::DeviceManager<10> device_manager;  ///< 设备管理器
  etl::string<512> status_str{""};               ///< 描述当前系统状态的字符串

  /**
   * @brief 初始化所有全局对象
   */
  void Init() {
    // 给设备起一下名字方便看哪个离线
    rc.SetName("rc");

    // 注册设备状态变化事件回调
    device_manager.OnDeviceStatusChange(
        [&](rm::device::Device *device) { status_str = device_manager.GetSummaryString(); });

    // 把设备都注册给设备管理器
    device_manager << &rc;

    buzzer.Init();
    led.Init();
    rc.Begin();
    imu.Begin();
    can1.SetFilter(0, 0);
    can2.SetFilter(0, 0);
    can1.Begin();
    can2.Begin();
  }
};

extern Globals *globals;