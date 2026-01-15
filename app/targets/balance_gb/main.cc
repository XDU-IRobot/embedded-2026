#include <librm.hpp>

#include "tim.h"
#include "timer_task.hpp"

#include "global.hpp"
Global global;

void MainLoop() {
  global.fsm.Update();
}

extern "C" {void AppMain(void) {
  global.bc = new BoardC;
  global.motor = new Motor;

  //初始化
  global.bc->BoardcInit();
  global.motor->MotorInit();

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
    &htim13,                                   //
    etl::delegate<void()>::create<MainLoop>()  //
  };

  mainloop_1000hz.Start();

  for (;;) {
    __WFI();
  }
}}