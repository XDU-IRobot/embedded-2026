#include "main.hpp"
#include "timer_task.hpp"
#include "tim.h"

//定频循环
void MainLoop() {
  //底盘逻辑
  ChassisControl();
  //摩擦轮电机逻辑
  ShooterControl();
  //拨盘电机逻辑
  MagazineControl();
}

extern "C" [[noreturn]] void AppMain(void) {
  /*启动CAN总线
   *启动遥控器
   */
  globals = new GlobalWarehouse;
  globals->Init();

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim13,
      etl::delegate<void()>::create<MainLoop>() //
  };
  mainloop_1000hz.SetPrescalerAndPeriod(164, 1000 - 1); // 84MHz / 84 / 1000 = 1kHz
  mainloop_1000hz.Start(); // 启动定时器

  for (;;) {
    __WFI();
  }
}