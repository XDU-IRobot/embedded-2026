
#include <librm.hpp>

#include "tim.h"

#include "dart_core.hpp"
#include "timer_task.hpp"
#include "dart_statemachine.hpp"

void MainLoop() {
  DartStateMachineUpdate(dart_rack->state_);
  dart_rack->Update();
  rm::device::DjiMotor<>::SendCommand();
}
extern "C" [[noreturn]] void AppMain(void) {
  dart_rack = new DartRack();
  dart_rack->Init();
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