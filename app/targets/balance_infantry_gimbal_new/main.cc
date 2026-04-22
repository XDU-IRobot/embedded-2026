
#include "fsm.hpp"
#include "tim.h"

#include "timer_task.hpp"
#include "globals.hpp"

Globals *globals{nullptr};

void GimbalStateUpdate();
void AimbotComUpdate();
void ChassisStateUpdate();

void Update_500HZ() {
  globals->imu.Update();
  globals->ahrs.Update(rm::modules::ImuData6Dof{-globals->imu.gyro_y(), globals->imu.gyro_x(), globals->imu.gyro_z(),
                                                -globals->imu.accel_y(), globals->imu.accel_x(),
                                                globals->imu.accel_z()});
  GimbalStateUpdate();
  AimbotComUpdate();
  ChassisStateUpdate;
  fsm::gimbal.receive(fsm::event::ControlLoop{});
  globals->loop_divisor = (globals->loop_divisor + 1) % 500;
  rm::device::DjiMotorBase::SendCommand();
};

void Update_250HZ() {
  if (globals->loop_divisor % 2 == 0) {
  }
}

void Update_100HZ() {
  if (globals->loop_divisor % 5 == 0) {
    // 100hz subloop
    globals->buzzer.SetFrequency(globals->buzzer_controller.Update().frequency);
    const auto &[r, g, b] = globals->led_controller.Update();
    globals->led(0xff000000 | r << 16 | g << 8 | b);
  }
}

void Update_50HZ() {
  if (globals->loop_divisor % 10 == 0) {
  }
}

void Update_10HZ() {
  if (globals->loop_divisor % 50 == 0) {
  }
}

void MainLoop() {
  Update_500HZ();
  Update_250HZ();
  Update_100HZ();
  Update_50HZ();
  Update_10HZ();
}

extern "C" [[noreturn]] void AppMain(void) {
  globals = new Globals;
  globals->Init();
  fsm::Init();

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

void GimbalStateUpdate() {
  static bool aimbot_on =
      (globals->aimbot_comm.aimbot_state() >> 0 & 0x01 || globals->aimbot_comm.aimbot_state() >> 0 & 0x03);
  if (globals->rc.online_status() == rm::device::Device::kOk) {
    if (globals->rc.switch_l() == rm::device::DR16::SwitchPosition::kDown) {
      switch (globals->rc.switch_r()) {
        case rm::device::DR16::SwitchPosition::kUp:
          fsm::gimbal.receive(fsm::event::ForceModeSwitch{fsm::StateId::kManual});  // 左下右上
          fsm::gimbal.shoot_state = true;
          break;
        case rm::device::DR16::SwitchPosition::kMid:
          fsm::gimbal.receive(fsm::event::ForceModeSwitch{fsm::StateId::kManual});  // 左下右中
          fsm::gimbal.shoot_state = false;
          break;
        default:
          fsm::gimbal.receive(fsm::event::ForceModeSwitch{fsm::StateId::kNoForce});  // 左下右下
          fsm::gimbal.shoot_state = false;
          break;
      }
    }

    if (globals->rc.switch_l() == rm::device::DR16::SwitchPosition::kMid) {
      switch (globals->rc.switch_r()) {
        case rm::device::DR16::SwitchPosition::kUp:
          fsm::gimbal.receive(fsm::event::ForceModeSwitch{fsm::StateId::kAuto});  // 左中右上
          fsm::gimbal.shoot_state = true;
          break;
        case rm::device::DR16::SwitchPosition::kMid:
          fsm::gimbal.receive(fsm::event::ForceModeSwitch{fsm::StateId::kAuto});  // 左中右中
          fsm::gimbal.shoot_state = false;
          break;
        default:
          fsm::gimbal.receive(fsm::event::ForceModeSwitch{fsm::StateId::kNoForce});  // 左中右下
          fsm::gimbal.shoot_state = false;
          break;
      }
    }

    if (globals->rc.switch_l() == rm::device::DR16::SwitchPosition::kUp && aimbot_on) {
      switch (globals->rc.switch_r()) {
        case rm::device::DR16::SwitchPosition::kUp:
          fsm::gimbal.receive(fsm::event::ForceModeSwitch{fsm::StateId::kAuto});  // 左上右上
          fsm::gimbal.shoot_state = true;
          break;
        case rm::device::DR16::SwitchPosition::kMid:
          fsm::gimbal.receive(fsm::event::ForceModeSwitch{fsm::StateId::kAuto});  // 左上右中
          fsm::gimbal.shoot_state = false;
          break;
        default:
          fsm::gimbal.receive(fsm::event::ForceModeSwitch{fsm::StateId::kNoForce});  // 左上右下
          fsm::gimbal.shoot_state = false;
          break;
      }
    }
  } else {
    fsm::gimbal.receive(fsm::event::ForceModeSwitch{fsm::StateId::kNoForce});
  }
}

void AimbotComUpdate() {
  globals->imu_count++;
  //    globals->time_camera++;
  //    if (globals->time_camera == 10) {
  //        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 65535);
  //        globals->time_camera = 0;
  //    }
  //    if (globals->time_camera == 5) {
  //        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  //    }
  if (globals->imu_count >= 10000) {
    globals->imu_count = 0;
  }
  globals->aimbot_comm.UpdateControl(globals->ahrs.euler_angle().yaw, globals->ahrs.euler_angle().pitch,
                                     globals->ahrs.euler_angle().roll, 3, 1, globals->imu_count, 0);
}

void ChassisStateUpdate() {
  if (globals->rc.online_status() == rm::device::Device::kOk) {
    if (globals->rc.switch_l() == rm::device::DR16::SwitchPosition::kUp ||
        globals->rc.switch_r() == rm::device::DR16::SwitchPosition::kDown) {
      globals->chassis_comm.data_tx.ChassisStateRequest = 0x00;
    } else {
      globals->chassis_comm.data_tx.ChassisMoveXRequest = globals->rc.left_x() / 66.f;
      globals->chassis_comm.data_tx.ChassisMoveYRequest = globals->rc.left_y() / 66.f;
      if (globals->rc.dial() > 400) {
        globals->chassis_comm.data_tx.ChassisStateRequest = 0x02;
      } else if (globals->rc.right_y() > 600) {
        globals->chassis_comm.data_tx.ChassisStateRequest = 0x03;
      } else {
        globals->chassis_comm.data_tx.ChassisStateRequest = 0x01;
      }
    }
  }
}
