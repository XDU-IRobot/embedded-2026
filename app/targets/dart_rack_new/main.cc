//
// Created by JL_HUANG on 2026/7/11.
//
#include <librm.hpp>
#include "tim.h"
#include "gpio.h"
#include "timer_task.hpp"
#include "dart_sys.hpp"

// DartSys* dart_sys=nullptr;

DartSys dart_sys;

void MainLoop() { dart_sys.loop(); }

/**
 *
 */
extern "C" [[noreturn]] void AppMain(void) {
  // dart_sys = new DartSys();
  dart_sys.init();
  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim13,                                   // 默认500hz
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_1000hz.SetPrescalerAndPeriod(84 - 1, 1000 - 1);  // 84MHz / 168 / 1000 = 500Hz
  mainloop_1000hz.Start();

  for (;;) {
    __WFI();
  }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == PitchMotorLimit_Pin) {
    if (HAL_GPIO_ReadPin(PitchMotorLimit_GPIO_Port, PitchMotorLimit_Pin) == GPIO_PIN_SET) {
      dart_sys.pitch_motor_limit_enabled_ = true;
    }
  }
}
