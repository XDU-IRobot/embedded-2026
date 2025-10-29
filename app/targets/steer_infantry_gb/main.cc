
#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "polling_timer.hpp"
#include "device_manager.hpp"

#include "controllers/gimbal_2dof.hpp"
#include "controllers/shoot_3fric.hpp"
#include "controllers/quad_steering_chassis.hpp"

extern "C" [[noreturn]] void AppMain(void) {
  AsyncBuzzer *buz = new AsyncBuzzer;
  LED *led = new LED;

  buz->Init();
  led->Init();

  buz->Beep(3, 40);
  (*led)(0x77ffff00);

  for (;;) {
    __WFI();
  }
}