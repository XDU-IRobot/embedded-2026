
#pragma once

#include <librm.hpp>

#include "can.h"
#include "usart.h"
#include "spi.h"

#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "timer_task.hpp"
#include "device_manager.hpp"

namespace test_gb {

struct GlobalWarehouse {
  Buzzer *buzzer{nullptr};  ///< 蜂鸣器
  rm::modules::BuzzerController<rm::modules::buzzer_melody::Beeps<1>, rm::modules::buzzer_melody::Startup>
      buzzer_controller;
  LED *led{nullptr};  ///< RGB LED灯
  rm::modules::RgbLedController<rm::modules::led_pattern::RedFlash, rm::modules::led_pattern::GreenBreath>
      led_controller;

  // 硬件接口 //
  rm::hal::Can *can1{nullptr};     ///< CAN 总线接口
  rm::hal::Serial *dbus{nullptr};  ///< 遥控器串口接口

  // 设备 //
  DeviceManager<10> device_manager;  ///< 设备管理器，维护所有设备在线状态
  rm::device::DR16 *rc{nullptr};     ///< 遥控器
  rm::device::GM6020 *yaw_motor{nullptr};
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr};
  rm::device::BMI088 *imu{nullptr};

  // 控制器 //
  rm::modules::SparseValueWatcher<rm::device::DR16::SwitchPosition, true> rc_l_switch_watcher, rc_r_switch_watcher;
  rm::modules::MahonyAhrs ahrs{500.f};  ///< mahony 姿态解算器，频率 500hz

  void Init() {
    buzzer = new Buzzer;
    led = new LED;

    can1 = new rm::hal::Can{hcan1};
    dbus = new rm::hal::Serial{huart3, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};

    rc = new rm::device::DR16{*dbus};
    yaw_motor = new rm::device::GM6020{*can1, 1};
    pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{*can1, {}};
    imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};

    device_manager << rc << yaw_motor << pitch_motor;

    can1->SetFilter(0, 0);
    can1->Begin();
    buzzer->Init();
    led->Init();
    rc->Begin();  // 启动遥控器接收，这行或许比较适合放到AppMain里面？
  }
};

extern GlobalWarehouse *globals;

}  // namespace test_gb