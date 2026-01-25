#include "main.hpp"
#include "timer_task.hpp"
#include "tim.h"
rm::f32 pitch;
rm::f32 yaw;
// 定频循环
void MainLoop() {
  //遥控器输入值
  l_switch_position_last=l_switch_position_now;
  l_switch_position_now=globals->rc->switch_l();
  r_switch_position_last=r_switch_position_now;
  r_switch_position_now=globals->rc->switch_r();
  // 底盘逻辑
  ChassisControl();
  // 摩擦轮电机逻辑
  ShooterControl();
  // 拨盘电机逻辑
  MagazineControl();
  // 云台控制逻辑
  GimbalControl();
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
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_1000hz.SetPrescalerAndPeriod(84, 1000 - 1);  // 84MHz / 84 / 1000 = 1kHz
  mainloop_1000hz.Start();                              // 启动定时器

  for (;;) {
    __WFI();
  }
}