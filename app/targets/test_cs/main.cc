
#include <librm.hpp>

#include "tim.h"
#include "usart.h"
#include "timer_task.hpp"

#include "global.hpp"
Global global;

// 实例化RemoteControl对象
RemoteControl remote_ctrl;

void MainLoop() { global.fsm.Update(); }

extern "C" {
void AppMain(void) {
  hal::Serial motor_bus{huart1, 100, hal::stm32::UartMode::kDma, hal::stm32::UartMode::kDma};

  // 初始化RemoteControl
  remote_ctrl.RemoteInit();
  // 将RemoteControl对象地址赋值给global.rc
  global.rc = &remote_ctrl;

  global.chassis = new Chassis{new device::ZdtStepper{motor_bus, 4, 0},  //
                               new device::ZdtStepper{motor_bus, 1, 1},  //
                               new device::ZdtStepper{motor_bus, 2, 0},  //
                               new device::ZdtStepper{motor_bus, 3, 1}};
  global.minipc = new MiniPC;

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };

  mainloop_1000hz.Start();

  for (;;) {
    __WFI();
  }
}
}
