
#include "buzzer.hpp"
#include "polling_timer.hpp"
#include "device_manager.hpp"
#include "timer_task.hpp"

#include "controllers/gimbal_2dof.hpp"
#include "controllers/shoot_3fric.hpp"

extern "C" [[noreturn]] void AppMain(void) {
  for (;;) {
    __WFI();
  }
}