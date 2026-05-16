#include <librm.hpp>

#include "can.h"
#include "usart.h"
#include "spi.h"

#include "timer_task.hpp"

#include "main.hpp"
#include "Gimbal.hpp"

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

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_1000hz.SetPrescalerAndPeriod(168 - 1, 1000 - 1);  // 84MHz / 168 / 1000 = 500Hz
  mainloop_1000hz.Start();

  for (;;) {
    __WFI();
  }
}

void GlobalWarehouse::Init() {
  buzzer = new Buzzer;
  led = new LED;

  can1 = new rm::hal::Can{hcan1};
  can2 = new rm::hal::Can{hcan2};
  aimbot_communicator = new rm::device::AimbotCanCommunicator{*can1};
  chassis_communicator = new rm::device::ChassisCommunicator{*can1};
  super_cap = new rm::device::GkSupercap{*can1};
  dbus = new rm::hal::Serial{huart3, 18, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
  referee_uart = new rm::hal::Serial{huart6, 128, hal::stm32::UartMode::kNormal, hal::stm32::UartMode::kDma};
  ident_uart = new rm::hal::Serial{huart1, 128, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
  rx_referee = new rm::device::RxReferee{*referee_uart};
  image_data = new rm::device::VT03;

  imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
  rc = new rm::device::DR16{*dbus};
  yaw_motor = new rm::device::GM6020{*can1, 4};
  pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>  //
      {*can2, {0x03, 0x02, 12.5f, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}}};
  friction_left = new rm::device::M3508{*can2, 1};
  friction_right = new rm::device::M3508{*can2, 2};
  dial_motor = new rm::device::M3508{*can1, 5};

  device_rc << rc;                                                // 遥控器
  device_gimbal << yaw_motor << pitch_motor;                      // 云台电机
  device_shoot << friction_left << friction_right << dial_motor;  // 发射机构电机
  device_nuc << aimbot_communicator;                              // NUC
  device_referee << image_data;                                   // 裁判系统

  can1->SetFilter(0, 0);
  can1->Begin();
  can2->SetFilter(0, 0);
  can2->Begin();
  rc->Begin();
  rx_referee->Begin();
  ident_uart->Begin();
  buzzer->Init();
  led->Init();

  led_controller.SetPattern<modules::led_pattern::GreenBreath>();
  buzzer_controller.Play<modules::buzzer_melody::Startup>();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  globals->GimbalPIDInit();
  globals->ShootPIDInit();
  gimbal->GimbalInit();
}

void GlobalWarehouse::GimbalPIDInit() {
  // 初始化PID
  // Yaw PID 参数
  gimbal_controller.pid().yaw_position.SetKp(100.0f).SetKi(0.0f).SetKd(2000.0f).SetMaxOut(30000.0f).SetMaxIout(0.0f);
  // gimbal_controller.pid().yaw_position.SetKp(100.0f).SetKi(0.0f).SetKd(2000.0f).SetMaxOut(0.0f).SetMaxIout(0.0f);
  gimbal_controller.pid().yaw_speed.SetKp(600.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(30000.0f).SetMaxIout(0.0f);
  // gimbal_controller.pid().yaw_speed.SetKp(600.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(0.0f).SetMaxIout(0.0f);
  // pitch PID 参数
  gimbal_controller.pid().pitch_position.SetKp(10.0f).SetKi(0.0f).SetKd(200.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  gimbal_controller.pid().pitch_speed.SetKp(0.45f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10.0f).SetMaxIout(0.0f);
  // gimbal_controller.pid().pitch_position.SetKp(10.0f).SetKi(0.0f).SetKd(200.0f).SetMaxOut(0.0f).SetMaxIout(0.0f);
  // gimbal_controller.pid().pitch_speed.SetKp(0.45f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(0.0f).SetMaxIout(0.0f);
}

void GlobalWarehouse::ShootPIDInit() {
  shoot_controller.pid().fric_1_speed.SetKp(8.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(16384.0f).SetMaxIout(0.0f);
  shoot_controller.pid().fric_2_speed.SetKp(8.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(16384.0f).SetMaxIout(0.0f);
  shoot_controller.pid().loader_position.SetKp(500.0f).SetKi(0.0f).SetKd(10.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  shoot_controller.pid().loader_speed.SetKp(8.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
}

void GlobalWarehouse::RCStateUpdate() {
  if (!globals->device_rc.all_device_ok() || !globals->chassis_communicator->gimbal_power_state()) {
    globals->StateMachine_ = kUnable;
  } else {
    if (globals->init_time > 0) {
      globals->StateMachine_ = kNoForce;
      globals->init_time--;
      return;
    }
    switch (globals->rc->switch_r()) {
      case rm::device::DR16::SwitchPosition::kUp:
        // 右拨杆打到最上侧挡位
        switch (globals->rc->switch_l()) {
          case rm::device::DR16::SwitchPosition::kDown:
            globals->StateMachine_ = kMatch;
            break;
          case rm::device::DR16::SwitchPosition::kMid:
          case rm::device::DR16::SwitchPosition::kUp:
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
        switch (globals->rc->switch_l()) {
          case rm::device::DR16::SwitchPosition::kUp:
            globals->StateMachine_ = kTest;
            gimbal->GimbalMove_ = kGbIdentify;
            break;
          case rm::device::DR16::SwitchPosition::kMid:
            globals->StateMachine_ = kTest;
            gimbal->GimbalMove_ = kGbFfVerify;
            break;
          case rm::device::DR16::SwitchPosition::kDown:
          default:
            globals->StateMachine_ = kNoForce;  // 左拨杆拨到下侧，进入比赛模式，此时全部系统都上电工作
            break;
        }
        break;
      default:
        globals->StateMachine_ = kNoForce;  // 如果遥控器离线，进入无力模式
        break;
    }
  }
}

void GlobalWarehouse::ChassisStateUpdate() {
  // 前后左右
  globals->chassis_move_x = rm::modules::Clamp(
      static_cast<f32>(globals->rc->right_x()) / 6.6f +
          static_cast<f32>(globals->image_update_flag ? (globals->image_data->data().keyboard_key >> 3 & 0x01) -
                                                            (globals->image_data->data().keyboard_key >> 2 & 0x01)
                                                      : globals->rc->key(rm::device::DR16::Key::kD) -
                                                            globals->rc->key(rm::device::DR16::Key::kA)) *
              100.0f,
      -100.0f, 100.0f);
  globals->chassis_move_y = rm::modules::Clamp(
      static_cast<f32>(globals->rc->right_y()) / 6.6f +
          static_cast<f32>(globals->image_update_flag ? (globals->image_data->data().keyboard_key >> 0 & 0x01) -
                                                            (globals->image_data->data().keyboard_key >> 1 & 0x01)
                                                      : globals->rc->key(rm::device::DR16::Key::kW) -
                                                            globals->rc->key(rm::device::DR16::Key::kS)) *
              100.0f,
      -100.0f, 100.0f);
  // 有无力
  if ((globals->StateMachine_ == kTest && gimbal->GimbalMove_ == kGbRemote) || globals->StateMachine_ == kMatch) {
    globals->chassis_state |= static_cast<u8>(1 << 0);
  } else {
    globals->chassis_state &= ~static_cast<u8>(1 << 0);
  }
  // 小陀螺
  if ((globals->StateMachine_ == kTest && globals->rc->dial() >= 650) ||
      (globals->image_update_flag ? globals->image_data->data().keyboard_key >> 4 & 0x01
                                  : globals->rc->key(rm::device::DR16::Key::kShift))) {
    globals->chassis_state |= static_cast<u8>(1 << 1);
    globals->chassis_state &= ~static_cast<u8>(1 << 2);
  } else if (globals->StateMachine_ == kTest && globals->rc->dial() <= -650) {
    globals->chassis_state |= static_cast<u8>(1 << 2);
    globals->chassis_state &= ~static_cast<u8>(1 << 1);
  } else {
    globals->chassis_state &= ~static_cast<u8>(1 << 1);
    globals->chassis_state &= ~static_cast<u8>(1 << 2);
  }
  // 高速模式
  // if (globals->super_cap->CapEnergy() <= 80 || globals->super_cap->ErrorCode()) {
  //   globals->chassis_state &= ~static_cast<u8>(1 << 3);
  // } else if (globals->StateMachine_ == kMatch) {
  //   if (globals->image_update_flag ? globals->image_data->data().keyboard_key >> 13 & 0x01
  //                                  : globals->rc->key(rm::device::DR16::Key::kC)) {
  //     globals->speed_change_flag = true;
  //   } else if (globals->speed_change_flag == 1) {
  //     globals->speed_change_flag = false;
  //     globals->chassis_state ^= static_cast<u8>(1 << 3);
  //   }
  // } else if (globals->rc->switch_r() == rm::device::DR16::SwitchPosition::kMid &&
  //            globals->rc->switch_l() == rm::device::DR16::SwitchPosition::kMid) {
  //   globals->chassis_state |= static_cast<u8>(1 << 3);
  // } else {
  //   globals->chassis_state &= ~static_cast<u8>(1 << 3);
  // }
  // 打符模式切换
  if ((globals->image_update_flag ? globals->image_data->data().keyboard_key >> 9 & 0x01
                                  : globals->rc->key(rm::device::DR16::Key::kF)) &&
      !globals->xf_state && globals->StateMachine_ == kMatch) {
    globals->df_flag = true;
  } else if (globals->df_flag) {
    globals->df_flag = false;
    globals->df_state ^= true;
  }
  if ((globals->image_update_flag ? globals->image_data->data().keyboard_key >> 10 & 0x01
                                  : globals->rc->key(rm::device::DR16::Key::kG)) &&
      !globals->df_state && globals->StateMachine_ == kMatch) {
    globals->xf_flag = true;
  } else if (globals->xf_flag) {
    globals->xf_flag = false;
    globals->xf_state ^= true;
  }
  if (globals->df_state) {
    globals->chassis_state |= static_cast<u8>(1 << 4);
    globals->chassis_state &= ~static_cast<u8>(1 << 5);
    globals->aim_mode = 0x02;
  } else if (globals->xf_state) {
    globals->chassis_state |= static_cast<u8>(1 << 5);
    globals->chassis_state &= ~static_cast<u8>(1 << 4);
    globals->aim_mode = 0x03;
  } else {
    globals->aim_mode = 0x01;
    globals->chassis_state &= ~static_cast<u8>(1 << 4);
    globals->chassis_state &= ~static_cast<u8>(1 << 5);
  }
  // UI信息
  globals->ui_refresh_flag = globals->image_update_flag ? globals->image_data->data().keyboard_key >> 8 & 0x01
                                                        : globals->rc->key(rm::device::DR16::Key::kR);
  globals->get_target_flag = globals->aimbot_communicator->aimbot_state() >> 0 & 0x01;
  globals->suggest_fire_flag = globals->aimbot_communicator->aimbot_state() >> 1 & 0x01;
  // 弹速调整
  if (globals->image_update_flag ? globals->image_data->data().keyboard_key >> 5 & 0x01
                                 : globals->rc->key(rm::device::DR16::Key::kCtrl)) {
    if (globals->image_update_flag ? globals->image_data->data().keyboard_key >> 14 & 0x01
                                   : globals->rc->key(rm::device::DR16::Key::kV)) {
      globals->aim_speed_change_flag = -1;
    } else if (globals->aim_speed_change_flag == -1) {
      globals->aim_speed_change--;
      if (globals->aim_speed_change < -10) globals->aim_speed_change = -10;
      globals->aim_speed_change_flag = 0;
    }
    if (globals->image_update_flag ? globals->image_data->data().keyboard_key >> 15 & 0x01
                                   : globals->rc->key(rm::device::DR16::Key::kB)) {
      globals->aim_speed_change_flag = 1;
    } else if (globals->aim_speed_change_flag == 1) {
      globals->aim_speed_change++;
      if (globals->aim_speed_change > 10) globals->aim_speed_change = 10;
      globals->aim_speed_change_flag = 0;
    }
  }
}

void GlobalWarehouse::Music() {
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
  globals->ahrs.Update(rm::modules::ImuData6Dof{-globals->imu->gyro_y(), globals->imu->gyro_x(),
                                                globals->imu->gyro_z() + 0.00075f, -globals->imu->accel_y(),
                                                globals->imu->accel_x(), globals->imu->accel_z()});
  // 硬触发
  // if (globals->aimbot_communicator->nuc_start_flag() && globals->device_nuc.all_device_ok()) {
  //   globals->imu_count++;
  //   globals->time_camera++;
  //   if (globals->time_camera == 10) {
  //     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 65535u);
  //   }
  //   if (globals->time_camera == 5) {
  //     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
  //   }
  // } else {
  //   globals->imu_count = 0;
  //   globals->time_camera = 0;
  // }
  // if (globals->imu_count >= 10000) {
  //   globals->imu_count = 0;
  // }
  // can 通信
  f32 ammo_speed;
  if (globals->chassis_communicator->ammo_speed() > 20.0f) {
    ammo_speed = globals->chassis_communicator->ammo_speed();
  } else {
    ammo_speed = 23.5f;
  }
  globals->aimbot_communicator->UpdateControl(
      globals->ahrs.euler_angle().yaw, globals->ahrs.euler_angle().pitch, globals->ahrs.euler_angle().roll,
      globals->chassis_communicator->robot_id() ? 103 : 3, 1, globals->imu_count, ammo_speed);
  globals->chassis_communicator->SendChassisCommand(
      globals->chassis_move_x, globals->chassis_move_y, globals->chassis_state, globals->ui_refresh_flag,
      globals->get_target_flag, globals->suggest_fire_flag, globals->aim_speed_change);
  globals->RCStateUpdate();
  globals->ChassisStateUpdate();
  gimbal->GimbalTask();
  globals->pitch_motor->SetMitCommand(0, 0, gimbal->pitch_torque_, 0, 0);
  // globals->pitch_motor->SetMitCommand(0, 0, 0, 0, 0);
  rm::device::DjiMotorBase::SendCommand(*can1);
  rm::device::DjiMotorBase::SendCommand(*can2);
}

void GlobalWarehouse::SubLoop250Hz() {}

void GlobalWarehouse::SubLoop100Hz() {
  globals->device_rc.Update();
  globals->device_nuc.Update();
  globals->device_gimbal.Update();
  globals->device_shoot.Update();
  globals->device_referee.Update();
  gimbal->GimbalIdentifyDataSend();
  if (globals->rc->switch_l() != rm::device::DR16::SwitchPosition::kUnknown &&
      globals->rc->switch_r() != rm::device::DR16::SwitchPosition::kUnknown) {
    if (globals->rc->switch_l() != globals->last_switch_l || globals->rc->switch_r() != globals->last_switch_r) {
      globals->buzzer_controller.Play<modules::buzzer_melody::Beeps<1>>();
      globals->last_switch_l = globals->rc->switch_l();
      globals->last_switch_r = globals->rc->switch_r();
    }
  }
}

void GlobalWarehouse::SubLoop50Hz() {
  const auto &[led_r, led_g, led_b] = globals->led_controller.Update();
  (*globals->led)(0xff000000 | led_r << 16 | led_g << 8 | led_b);
  buzzer->SetFrequency(globals->buzzer_controller.Update().frequency);
}

void GlobalWarehouse::SubLoop10Hz() {
  if (globals->init_time == 0) {
    globals->image_update_flag = globals->device_referee.all_device_ok();
  }
  globals->time = 0;
}
