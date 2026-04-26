#include <librm.hpp>

#include "tim.h"

#include "dart_core.hpp"
#include "timer_task.hpp"
#include "dart_statemachine.hpp"
#include "LCDFont.h"
#include "lcd_init.h"
#include "sd_card.h"
#include "../../LVGL/lvgl.h"
#include "librm/device/actuator/dm_motor.hpp"

extern "C" void init_lvgl_demo(void);

// 全局变量标志，用来通知 main 的死循环去跑 LVGL 处理器
bool is_lvgl_running = false;

void MainLoop() {
  DartStateMachineUpdate(dart_rack->state_);
  dart_rack->Update();
  rm::device::DjiMotorBase::SendCommand();
  // LCD_DISPLAY();
}
extern "C" [[noreturn]] void AppMain(void) {
  LCD_init();

  dart_rack = new DartRack();
  dart_rack->Init();

  // 初始化 LVGL 环境与 DEMO
  init_lvgl_demo();

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim14,                                   // 2.0
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_1000hz.SetPrescalerAndPeriod(168 - 1, 1000 - 1);
  mainloop_1000hz.Start();

  for (;;) {
    if (is_lvgl_running) {
      // 只有在两个拨杆都上拨状态下才会进入并执行屏幕刷新和计算
      lv_timer_handler();
      HAL_Delay(5);  // 休息5ms，防止占满CPU
    } else {
      __WFI();  // 休眠等待下一个中断
    }
  }
}