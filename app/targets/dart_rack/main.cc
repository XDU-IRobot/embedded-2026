#include <librm.hpp>
#include "dart_core.hpp"
#include "main.hpp"
#include "tim.h"
#include "timer_task.hpp"
#include "dart_statemachine.hpp"
void MainLoop() { DartStateMachineUpdate(*dart_rack->state); }
extern "C" [[noreturn]] void AppMain(void) {
  dart_rack = new DartRack();
  DartRack::Init();
  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  // mainloop_1000hz.SetPrescalerAndPeriod(168 - 1, 1000 - 1);  // 84MHz / 168 / 1000 = 500Hz
  mainloop_1000hz.Start();

  for (;;) {
    __WFI();
  }
}