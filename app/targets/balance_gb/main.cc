#include <librm.hpp>

#include "tim.h"
#include "timer_task.hpp"

#include "global.hpp"
Global global;

extern rm::hal::Can *can2;

void MainLoop() {
  // 分频计数增加
  global.divide_count++;
  //  任务
  global.fsm.Update_500HZ();
  global.fsm.Update_250HZ();
  global.fsm.Update_100HZ();
  global.fsm.Update_25HZ();
  global.fsm.Update_10HZ();
}

extern "C" {
void AppMain(void) {
  global.bc = new BoardC;
  global.motor = new Motor;
  global.minipc = new MiniPC;

  // 初始化
  global.bc->BoardcInit();
  global.motor->MotorInit();
  global.motor->MotorPidInit();
  global.chassis_controller = new ChassisController{*global.motor->can2};

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{&htim13, etl::delegate<void()>::create<MainLoop>()};
  // 降频到500hz并启动
  mainloop_1000hz.SetPrescalerAndPeriod(168 - 1, 1000 - 1);  // 84MHz / 168 / 1000 = 500Hz
  mainloop_1000hz.Start();

  for (;;) {
    __WFI();
  }
}
}