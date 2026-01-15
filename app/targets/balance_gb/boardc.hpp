#pragma once

#include <librm.hpp>

#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "device_manager.hpp"

using namespace rm;

class BoardC {
  public:
    Buzzer *buzzer{nullptr};  ///< 蜂鸣器
    rm::modules::BuzzerController<rm::modules::buzzer_melody::Silent, rm::modules::buzzer_melody::Startup,
                                  rm::modules::buzzer_melody::Success, rm::modules::buzzer_melody::Error,
                                  rm::modules::buzzer_melody::SuperMario, rm::modules::buzzer_melody::SeeUAgain,
                                  rm::modules::buzzer_melody::TheLick>
    buzzer_controller;
    LED *led{nullptr};  ///< RGB LED灯
    rm::modules::RgbLedController<rm::modules::led_pattern::Off, rm::modules::led_pattern::RedFlash,
                                  rm::modules::led_pattern::GreenBreath, rm::modules::led_pattern::RgbFlow>
        led_controller;  ///< RGB LED控制器

    rm::hal::Serial *dbus{nullptr};     ///< 遥控器串口接口
    rm::device::DR16 *rc{nullptr};      ///< 遥控器
    rm::device::BMI088 *imu{nullptr};   ///< bmi088
    rm::modules::MahonyAhrs ahrs{1000.0f};                  ///< 姿态解算器

    ::DeviceManager<1> device_rc;  ///< 设备管理器，维护所有设备在线状态
  public:
    ///< 姿态解算角
    f32 pitch = 0.f;
    f32 roll = 0.f;
    f32 yaw = 0.f;


    void BoardcInit();

    void EulerUpdate();

  private:

};