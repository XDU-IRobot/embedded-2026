#include <librm.hpp>

#include "can.h"
#include "usart.h"
#include "spi.h"

#include "timer_task.hpp"

#include "main.hpp"
#include "Gimbal.hpp"
#include "freemaster.hpp"

using namespace rm;

void MainLoop() {
  globals->time++;
  globals->SubLoop500Hz();
  if (globals->time % 2 == 0) globals->SubLoop250Hz();
  if (globals->time % 5 == 0) globals->SubLoop100Hz();
  if (globals->time % 10 == 0) globals->SubLoop50Hz();
  if (globals->time % 50 == 0) globals->SubLoop10Hz();
}

extern "C" [[noreturn]] void AppMain(void) {
  rm::Sleep(std::chrono::milliseconds(100));
  globals = new GlobalWarehouse;
  gimbal = new Gimbal;
  globals->Init();

  // 创建主循环定时任务，定频500Hz
  TimerTask mainloop_500hz{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_500hz.SetPrescalerAndPeriod(168 - 1, 1000 - 1);  // 84MHz / 168 / 1000 = 500Hz
  mainloop_500hz.Start();

  for (;;) {
    __WFI();
  }
}

void GlobalWarehouse::Init() {
  buzzer = new Buzzer;
  led = new LED;

  can1 = new rm::hal::Can{hcan1};
  can2 = new rm::hal::Can{hcan2};
  aimbot_communicator = new rm::device::AimbotCanCommunicator{*can2};
  dbus = new rm::hal::Serial<18>{huart3, false, true};
  ident_uart = new rm::hal::Serial<128>{huart1, false, true};

  imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
  rc = new rm::device::DR16{*dbus};
  rc->SetHeartbeatTimeout(std::chrono::milliseconds(100));
  // DM4310 MIT：CAN2 反馈 ID 0x12，控制 ID 0x02
  yaw_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>  //
      {*can1, {0x12, 0x02, 12.5f, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}}};
  // DM4310 MIT：CAN2 反馈 ID 0x11，控制 ID 0x01
  pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>  //
      {*can1, {0x11, 0x01, 12.5f, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}}};

  // 波形生成器
  yaw_wave = WaveFactory::Trapezoid(4.f, 0.1745f / 2, 0.2f, 0.2f, 0.1);
  pitch_wave = WaveFactory::Trapezoid(6.f, 0.1745f / 2, 0.2f, 0.2f, 0.2);

  device_rc << rc;                            // 遥控器
  device_gimbal << yaw_motor << pitch_motor;  // 云台电机
  device_nuc << aimbot_communicator;          // NUC

  can1->SetFilter(0, 0);
  can2->SetFilter(0, 0);
  can1->Begin();
  can2->Begin();
  rc->Begin();
  ident_uart->Start();
  buzzer->Init();
  led->Init();

  led_controller.SetPattern<modules::led_pattern::GreenBreath>();
  buzzer_controller.Play<modules::buzzer_melody::Startup>();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  globals->GimbalSMCInit();
  globals->GimbalPIDInit();
  gimbal->GimbalInit();

  freemaster_init();
}

void GlobalWarehouse::GimbalSMCInit() {
  // 初始化 SMC
  // gimbal_controller.motor().yaw.smc().SMCInit(30, 400, 0, 0.001, 21, 27, 30000, 90, 0.5, 1.0);
  // gimbal_controller.motor().pitch.smc().SMCInit(33, 380, 0, 0.0, 21, 27, 10, 0.005, 10, 1.0);
  gimbal_controller.motor().yaw.smc().SMCInit(60, 40, 0, 0.0003, 27, 27, 10, 0.007, 50, 1);
  gimbal_controller.motor().pitch.smc().SMCInit(50, 60, 0, 0.0003, 27, 27, 10, 0.001, 10, 1);
}

void GlobalWarehouse::GimbalPIDInit() {
  // 初始化 PID (dt = 1.0f)
  // Yaw PID 参数
  // gimbal_controller.motor().yaw.pid().position.SetKp(380.0f).SetKi(0.0f).SetKd(9000.0f).SetMaxOut(12000.0f).SetMaxIout(0.0f);
  // gimbal_controller.motor().yaw.pid().speed.SetKp(580.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(15000.0f).SetMaxIout(0.0f);
  gimbal_controller.motor().yaw.pid().position.SetKp(60.0f).SetKi(0.0f).SetKd(700.0f).SetMaxOut(10000.0f).SetMaxIout(
      0.0f);
  gimbal_controller.motor().yaw.pid().speed.SetKp(0.5f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10.0f).SetMaxIout(0.0f);
  // pitch PID 参数
  gimbal_controller.motor().pitch.pid().position.SetKp(60.0f).SetKi(0.0f).SetKd(700.0f).SetMaxOut(10000.0f).SetMaxIout(
      0.0f);
  gimbal_controller.motor().pitch.pid().speed.SetKp(0.5f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10.0f).SetMaxIout(0.0f);
}

void GlobalWarehouse::RCStateUpdate() {
  const bool dt17_online = globals->device_rc.all_device_ok();
  if (dt17_online) {
    globals->remote_source = RemoteControlSource::kDt17;
  } else {
    globals->remote_source = RemoteControlSource::kNone;
  }

  if (globals->init_time > 0) {
    globals->StateMachine_ = kNoForce;
    globals->init_time--;
    return;
  }
  if (globals->remote_source == RemoteControlSource::kDt17) {
    switch (globals->rc->switch_r()) {
      case rm::device::DR16::SwitchPosition::kUp:
        // 右拨杆打到最上侧挡位
        switch (globals->rc->switch_l()) {
          case rm::device::DR16::SwitchPosition::kDown:
            globals->StateMachine_ = kMatch;
            break;
          case rm::device::DR16::SwitchPosition::kMid:
            globals->StateMachine_ = kTest;
            gimbal->GimbalMove_ = kGbFfVerify;
            break;
          case rm::device::DR16::SwitchPosition::kUp:
            globals->StateMachine_ = kTest;
            gimbal->GimbalMove_ = kGbIdentify;
            break;
          default:
            globals->StateMachine_ = kNoForce;
            break;
        }
        break;

      case rm::device::DR16::SwitchPosition::kMid:
        // 右拨杆打到中间挡位
        switch (globals->rc->switch_l()) {
          case rm::device::DR16::SwitchPosition::kDown:
            globals->StateMachine_ = kTest;
            gimbal->GimbalMove_ = kGbRemote;
            break;
          case rm::device::DR16::SwitchPosition::kMid:
            globals->StateMachine_ = kTest;
            gimbal->GimbalMove_ = kGbAimbotFu;
            break;
          case rm::device::DR16::SwitchPosition::kUp:
            globals->StateMachine_ = kTest;
            gimbal->GimbalMove_ = kGbAimbot;
            break;
          default:
            globals->StateMachine_ = kNoForce;
            break;
        }
        break;

      case rm::device::DR16::SwitchPosition::kDown:
        if (globals->rc->switch_l() == rm::device::DR16::SwitchPosition::kUp) {
          globals->Music();
        }
        globals->StateMachine_ = kNoForce;
        break;
      default:
        globals->StateMachine_ = kNoForce;  // 如果遥控器离线，进入无力模式
        break;
    }
  } else {
    globals->StateMachine_ = kNoForce;
  }
}

void GlobalWarehouse::RemoteInputUpdate() {
  globals->remote_input = {};
  if (globals->remote_source != RemoteControlSource::kDt17) {
    return;
  }

  constexpr f32 kDt17Scale = 660.0f;
  globals->remote_input.left_x = static_cast<f32>(globals->rc->left_x()) / kDt17Scale;
  globals->remote_input.left_y = static_cast<f32>(globals->rc->left_y()) / kDt17Scale;
  globals->remote_input.right_x = static_cast<f32>(globals->rc->right_x()) / kDt17Scale;
  globals->remote_input.right_y = static_cast<f32>(globals->rc->right_y()) / kDt17Scale;
  globals->remote_input.dial = static_cast<f32>(globals->rc->dial()) / kDt17Scale;
  globals->remote_input.mouse_x = globals->rc->mouse_x();
  globals->remote_input.mouse_y = globals->rc->mouse_y();
  globals->remote_input.mouse_z = globals->rc->mouse_z();
  globals->remote_input.mouse_left = globals->rc->mouse_button_left();
  globals->remote_input.mouse_right = globals->rc->mouse_button_right();
  for (u8 bit = 0; bit < 16; ++bit) {
    const auto key = static_cast<rm::device::DR16::Key>(1U << bit);
    if (globals->rc->key(key)) globals->remote_input.keyboard |= static_cast<u16>(1U << bit);
  }
}

void GlobalWarehouse::AimModeUpdate() {
  // 打符模式切换
  if (globals->remote_input.key(static_cast<u16>(rm::device::DR16::Key::kF)) && !globals->xf_state &&
      globals->StateMachine_ == kMatch) {
    globals->df_flag = true;
  } else if (globals->df_flag) {
    globals->df_flag = false;
    globals->df_state ^= true;
  }
  if (globals->remote_input.key(static_cast<u16>(rm::device::DR16::Key::kG)) && !globals->df_state &&
      globals->StateMachine_ == kMatch) {
    globals->xf_flag = true;
  } else if (globals->xf_flag) {
    globals->xf_flag = false;
    globals->xf_state ^= true;
  }
  if (globals->StateMachine_ == kMatch) {
    if (globals->df_state) {
      globals->aim_mode = 0x02;
    } else if (globals->xf_state) {
      globals->aim_mode = 0x03;
    } else {
      globals->aim_mode = 0x01;
    }
  }
}

void GlobalWarehouse::Music() {
  // Music() is called while decoding the DT17 switch combination, before the
  // unified input snapshot for this control cycle is refreshed.
  if (globals->rc->dial() >= 650) {
    globals->music_play_flag = true;
  }
  if (globals->rc->dial() <= -650 && !globals->music_change_flag) {
    globals->music_choice++;
    globals->buzzer_controller.Play<modules::buzzer_melody::Beeps<1>>();
    globals->music_change_flag = true;
  } else if (globals->rc->dial() >= 0) {
    globals->music_change_flag = false;
  }
  if (globals->music_choice == 3) {
    globals->music_choice = 0;
  }
  if (globals->music_play_flag) {
    if (globals->music_choice == 1) {
      globals->buzzer_controller.Play<modules::buzzer_melody::SeeUAgain>();
      globals->music_play_flag = false;
    }
    if (globals->music_choice == 2) {
      globals->buzzer_controller.Play<modules::buzzer_melody::SuperMario>();
      globals->music_play_flag = false;
    }
  }
}

void GlobalWarehouse::SubLoop500Hz() {
  globals->imu->Update();
  // globals->ahrs.Update(rm::modules::ImuData6Dof{-globals->imu->gyro_y(), globals->imu->gyro_x(),
  //                                               globals->imu->gyro_z() + 0.00075f, -globals->imu->accel_y(),
  //                                               globals->imu->accel_x(), globals->imu->accel_z()});
  globals->ahrs.Update(rm::modules::ImuData6Dof{-globals->imu->gyro_x(), -globals->imu->gyro_y(),
                                                globals->imu->gyro_z() - 0.0031f, -globals->imu->accel_x(),
                                                -globals->imu->accel_y(), globals->imu->accel_z()});
  // can 通信
  globals->aimbot_communicator->UpdateControl(globals->ahrs.euler_angle().yaw, globals->ahrs.euler_angle().pitch,
                                              globals->ahrs.euler_angle().roll, globals->aimbot_robot_id,
                                              globals->aim_mode, globals->imu_count, globals->aimbot_bullet_speed);
  globals->RCStateUpdate();
  globals->RemoteInputUpdate();
  globals->AimModeUpdate();
  gimbal->GimbalTask();
  globals->yaw_motor->SetMitCommand(0, 0, gimbal->yaw_torque_, 0, 0);
  globals->pitch_motor->SetMitCommand(0, 0, gimbal->pitch_torque_, 0, 0);

  globals->yaw_wave.Update(0.002f);
  globals->pitch_wave.Update(0.002f);
}

void GlobalWarehouse::SubLoop250Hz() {}

void GlobalWarehouse::SubLoop100Hz() {
  globals->device_rc.Update();
  globals->device_nuc.Update();
  globals->device_gimbal.Update();
  if (globals->rc->switch_l() != rm::device::DR16::SwitchPosition::kUnknown &&
      globals->rc->switch_r() != rm::device::DR16::SwitchPosition::kUnknown) {
    if (globals->rc->switch_l() != globals->last_switch_l || globals->rc->switch_r() != globals->last_switch_r) {
      globals->buzzer_controller.Play<modules::buzzer_melody::Beeps<1>>();
      globals->last_switch_l = globals->rc->switch_l();
      globals->last_switch_r = globals->rc->switch_r();
    }
  }
  freemaster();
}

void GlobalWarehouse::SubLoop50Hz() {
  gimbal->GimbalIdentifyDataSend();
  const auto &[led_r, led_g, led_b] = globals->led_controller.Update();
  (*globals->led)(0xff000000 | led_r << 16 | led_g << 8 | led_b);
  buzzer->SetFrequency(globals->buzzer_controller.Update().frequency);
}

void GlobalWarehouse::SubLoop10Hz() { globals->time = 0; }
