#include "main.hpp"

Gimbal* gimbal = nullptr;

void MainLoop() {
  gimbal->time_++;
  gimbal->SubLoop500Hz();
  gimbal->SubLoop250Hz();
  gimbal->SubLoop100Hz();
  gimbal->SubLoop50Hz();
  gimbal->SubLoop10Hz();
}

extern "C" [[noreturn]] void AppMain(void) {
  gimbal = new Gimbal();
  gimbal->GimbalInit();

  // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{&htim13, etl::delegate<void()>::create<MainLoop>()};
  mainloop_1000hz.SetPrescalerAndPeriod(168 - 1, 1000 - 1);  // 84MHz / 168 / 1000 = 500Hz
  mainloop_1000hz.Start();

  for (;;) {
    __WFI();
  }
}