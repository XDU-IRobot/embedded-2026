
#include "fsm.hpp"
#include "tim.h"

#include "timer_task.hpp"
#include "globals.hpp"

Globals *globals{nullptr};

namespace {
void MainLoop() {
  static int loop_divisor = 0;

  if (loop_divisor % 5 == 0) {
    // 100hz subloop
    globals->buzzer.SetFrequency(globals->buzzer_controller.Update().frequency);
    const auto &[r, g, b] = globals->led_controller.Update();
    globals->led(0xff000000 | r << 16 | g << 8 | b);
  }

  fsm::gimbal.receive(fsm::event::ControlLoop{});

  if (globals->rc.online_status() == rm::device::Device::kOk) {
    switch (globals->rc.switch_r()) {
      case rm::device::DR16::SwitchPosition::kUp:
        fsm::gimbal.receive(fsm::event::ForceModeSwitch{fsm::StateId::kFollowTrajectory});
        break;
      case rm::device::DR16::SwitchPosition::kMid:
        fsm::gimbal.receive(fsm::event::ForceModeSwitch{fsm::StateId::kManual});
        break;
      case rm::device::DR16::SwitchPosition::kDown:
        fsm::gimbal.receive(fsm::event::ForceModeSwitch{fsm::StateId::kNoForce});
        break;
      default:
        fsm::gimbal.receive(fsm::event::ForceModeSwitch{fsm::StateId::kNoForce});
        break;
    }
  } else {
    fsm::gimbal.receive(fsm::event::ForceModeSwitch{fsm::StateId::kNoForce});
  }

  loop_divisor = (loop_divisor + 1) % 500;
}

void DmSetZero(int repeats = 10) {
  globals->yaw_motor.SendInstruction(rm::device::DmMotorInstructions::kEnable);
  globals->pitch_motor.SendInstruction(rm::device::DmMotorInstructions::kEnable);
  HAL_Delay(200);
  for (int i = 0; i < repeats; i++) {
    globals->yaw_motor.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
    globals->pitch_motor.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
    HAL_Delay(100);
  }
  HAL_Delay(200);
  globals->yaw_motor.SendInstruction(rm::device::DmMotorInstructions::kDisable);
  globals->pitch_motor.SendInstruction(rm::device::DmMotorInstructions::kDisable);
}
}  // namespace

extern "C" [[noreturn]] void AppMain(void) {
  globals = new Globals;
  globals->Init();
  fsm::Init();

  // DmSetZero();

  for (auto ch : {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4}) {
    HAL_TIM_PWM_Start(&htim1, ch);
  }
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  globals->buzzer_controller.Play<rm::modules::buzzer_melody::Startup>();
  globals->led_controller.SetPattern<rm::modules::led_pattern::RgbFlow>();

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_1000hz.SetPrescalerAndPeriod(168 - 1, 1000 - 1);  // 84MHz / 168 / 1000 = 500Hz
  mainloop_1000hz.Start();

  for (;;) {
    // __WFI();
  }
}