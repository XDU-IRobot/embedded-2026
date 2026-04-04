#include <librm.hpp>

#include "tim.h"
#include "timer_task.hpp"
#include "communiate.hpp"

#include "global.hpp"
Global global;

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
  Sleep(std::chrono::milliseconds(1000));  // 等待设备初始化完成
  global.bc = new BoardC;
  global.motor = new Motor;

  // 初始化
  global.bc->BoardcInit();
  global.motor->MotorInit();

  global.chassis_rx = new ChassisCommunicator(*global.motor->can2, 0x119);
  global.chassis_tx = new ChassisCommunicator{*global.motor->can2, 0x120};

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{&htim13, etl::delegate<void()>::create<MainLoop>()};
  // 降频到500hz并启动
  mainloop_1000hz.SetPrescalerAndPeriod(168 - 1, 1000 - 1);
  mainloop_1000hz.Start();

  for (;;) {
    __WFI();
  }
}
}