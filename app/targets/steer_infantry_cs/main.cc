
#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "polling_timer.hpp"
#include "device_manager.hpp"
#include "timer_task.hpp"

#include "controllers/quad_steering_chassis.hpp"

AsyncBuzzer *buz;

void MainLoop() { buz->Beep(1); }

extern "C" [[noreturn]] void AppMain(void) {
  buz = new AsyncBuzzer;
  LED *led = new LED;

  buz->Init();
  led->Init();

  buz->Beep(2, 40);
  (*led)(0x77ffff00);

  TimerTask mainloop_1000hz{
      // 创建主循环定时任务，定频1khz
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_1000hz.Start();

  for (;;) {
    __WFI();
  }
}