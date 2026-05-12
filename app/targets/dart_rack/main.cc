#include <librm.hpp>

#include "tim.h"
#include "gpio.h"

#include "dart_core.hpp"
#include "timer_task.hpp"
#include "dart_statemachine.hpp"
#include "LCDFont.h"
#include "lcd_init.h"
#include "sd_card.h"
#include "../../LVGL/lvgl.h"
#include "librm/device/actuator/dm_motor.hpp"

extern "C" void init_lvgl_demo(void);

extern volatile uint8_t g_trigger_motor_limit_triggered;
extern volatile uint8_t g_add_motor_limit_triggered;
extern volatile uint8_t g_load_motor_l_limit_triggered;
extern volatile uint8_t g_load_motor_r_limit_triggered;

volatile uint8_t g_trigger_limit_ever_hit = 0;
volatile uint8_t g_add_limit_ever_hit = 0;
volatile uint8_t g_load_l_limit_ever_hit = 0;
volatile uint8_t g_load_r_limit_ever_hit = 0;

volatile uint8_t g_add_limit_suppressed = 0;
volatile uint8_t g_trigger_motor_limit_suppressed = 0;
volatile uint8_t g_vision_is_valid = 0;
volatile int32_t g_trigger_error = 0;  // 全局变量用于监视 trigger_error

bool is_lvgl_running = false;

static void LimitSwitchUpdate() {

    if (HAL_GPIO_ReadPin(add_motor_EXTI_GPIO_Port, add_motor_EXTI_Pin) == GPIO_PIN_SET) {
      g_add_limit_suppressed = 0;
    }else if (HAL_GPIO_ReadPin(add_motor_EXTI_GPIO_Port, add_motor_EXTI_Pin) == GPIO_PIN_RESET) {
      g_add_limit_suppressed = 1;
    }

    if (HAL_GPIO_ReadPin(trigger_motor_EXTI_GPIO_Port, trigger_motor_EXTI_Pin) == GPIO_PIN_SET) {
      g_trigger_motor_limit_suppressed = 0;
    }else if (HAL_GPIO_ReadPin(trigger_motor_EXTI_GPIO_Port, trigger_motor_EXTI_Pin) == GPIO_PIN_RESET) {
      g_trigger_motor_limit_suppressed = 1;
    }


  if (g_trigger_motor_limit_triggered && g_trigger_motor_limit_suppressed) {
    g_trigger_motor_limit_triggered = 0;
    dart_rack->trigger_motor_->SetCurrent(0);
    dart_rack->trigger_motor_speed_pid_.Clear();
    dart_rack->trigger_motor_odometer_.Reset();
    g_trigger_limit_ever_hit = 1;
  }
  if (g_add_motor_limit_triggered && g_add_limit_suppressed) {
    g_add_motor_limit_triggered = 0;
    dart_rack->add_motor_->SetCurrent(0);
    dart_rack->add_motor_speed_pid_.Clear();
    dart_rack->add_motor_odometer_.Reset();
    g_add_limit_ever_hit = 1;
  }
  if (g_load_motor_l_limit_triggered) {
    g_load_motor_l_limit_triggered = 0;
    dart_rack->load_motor_l_->SetCurrent(0);
    dart_rack->load_motor_l_speed_pid_.Clear();
    dart_rack->load_motor_l_odometer_.Reset();
    g_load_l_limit_ever_hit = 1;
  }
  if (g_load_motor_r_limit_triggered) {
    g_load_motor_r_limit_triggered = 0;
    dart_rack->load_motor_r_->SetCurrent(0);
    dart_rack->load_motor_r_speed_pid_.Clear();
    dart_rack->load_motor_r_odometer_.Reset();
    g_load_r_limit_ever_hit = 1;
  }
  if (!g_trigger_limit_ever_hit &&
      HAL_GPIO_ReadPin(trigger_motor_EXTI_GPIO_Port, trigger_motor_EXTI_Pin) == GPIO_PIN_RESET) {
    dart_rack->trigger_motor_->SetCurrent(0);
    dart_rack->trigger_motor_speed_pid_.Clear();
    dart_rack->trigger_motor_odometer_.Reset();
    g_trigger_limit_ever_hit = 1;
  }
  if (!g_add_limit_ever_hit &&
      HAL_GPIO_ReadPin(add_motor_EXTI_GPIO_Port, add_motor_EXTI_Pin) == GPIO_PIN_RESET) {
    dart_rack->add_motor_->SetCurrent(0);
    dart_rack->add_motor_speed_pid_.Clear();
    dart_rack->add_motor_odometer_.Reset();
    g_add_limit_ever_hit = 1;
    g_add_limit_suppressed = 1;
  }
  if (!g_load_l_limit_ever_hit &&
      HAL_GPIO_ReadPin(load_motor_left_EXTI_GPIO_Port, load_motor_left_EXTI_Pin) == GPIO_PIN_RESET) {
    dart_rack->load_motor_l_->SetCurrent(0);
    dart_rack->load_motor_l_speed_pid_.Clear();
    dart_rack->load_motor_l_odometer_.Reset();
    g_load_l_limit_ever_hit = 1;
  }
  if (!g_load_r_limit_ever_hit &&
      HAL_GPIO_ReadPin(load_motor_right_EXTI_GPIO_Port, load_motor_right_EXTI_Pin) == GPIO_PIN_RESET) {
    dart_rack->load_motor_r_->SetCurrent(0);
    dart_rack->load_motor_r_speed_pid_.Clear();
    dart_rack->load_motor_r_odometer_.Reset();
    g_load_r_limit_ever_hit = 1;
  }
}

void MainLoop() {
  LimitSwitchUpdate();
  DartStateMachineUpdate(dart_rack->state_);
  dart_rack->Update();
  rm::device::DjiMotorBase::SendCommand();
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