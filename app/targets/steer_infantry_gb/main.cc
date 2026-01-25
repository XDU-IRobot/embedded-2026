#include <librm.hpp>

#include "can.h"
#include "usart.h"
#include "spi.h"

#include "timer_task.hpp"

#include "main.hpp"
#include "Gimbal.hpp"
float yaw_set;
float yaw_ecd;
float pitch_set;
float pitch_ecd;
using namespace rm;

void MainLoop() {
  globals->time++;
  globals->SubLoop500Hz();
  globals->SubLoop250Hz();
  globals->SubLoop100Hz();
  globals->SubLoop50Hz();
  globals->SubLoop10Hz();
}

extern "C" [[noreturn]] void AppMain(void) {
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
  can_communicator = new rm::device::AimbotCanCommunicator{*can2};
  dbus = new rm::hal::Serial{huart3, 18, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
  referee_uart = new rm::hal::Serial{huart6, 128, hal::stm32::UartMode::kNormal, hal::stm32::UartMode::kDma};

  imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
  rc = new rm::device::DR16{*dbus};
  yaw_motor = new rm::device::GM6020{*can1, 4};
  pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>  //
      {*can2, {0x03, 0x02, 12.5f, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}}};
  friction_left = new rm::device::M3508{*can2, 1};
  friction_right = new rm::device::M3508{*can2, 2};
  dial_motor = new rm::device::M3508{*can1, 3};

  device_rc << rc;                                                // 遥控器
  device_gimbal << yaw_motor << pitch_motor;                      // 云台电机
  device_shoot << friction_left << friction_right << dial_motor;  // 发射机构电机
  device_nuc << can_communicator;                                 // NUC

  can1->SetFilter(0, 0);
  can1->Begin();
  can2->SetFilter(0, 0);
  can2->Begin();
  rc->Begin();
  buzzer->Init();
  led->Init();

  led_controller.SetPattern<modules::led_pattern::GreenBreath>();
  buzzer_controller.Play<modules::buzzer_melody::Startup>();

  globals->GimbalPIDInit();
  globals->ShootPIDInit();
  gimbal->GimbalInit();
}

void GlobalWarehouse::GimbalPIDInit() {
  // 初始化PID
  // Yaw PID 参数
  gimbal_controller.pid().yaw_position.SetKp(1000.0f).SetKi(0.0f).SetKd(24000.0f).SetMaxOut(30000.0f).SetMaxIout(0.0f);
  gimbal_controller.pid().yaw_speed.SetKp(300.0f).SetKi(0.0f).SetKd(100.0f).SetMaxOut(30000.0f).SetMaxIout(0.0f);
  // pitch PID 参数
  gimbal_controller.pid().pitch_position.SetKp(70.0f).SetKi(0.0f).SetKd(1300.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  gimbal_controller.pid().pitch_speed.SetKp(0.45f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10.0f).SetMaxIout(0.0f);
}

void GlobalWarehouse::ShootPIDInit() {
  shoot_controller.pid().fric_1_speed.SetKp(8.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(16384.0f).SetMaxIout(0.0f);
  shoot_controller.pid().fric_2_speed.SetKp(8.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(16384.0f).SetMaxIout(0.0f);
  shoot_controller.pid().loader_position.SetKp(500.0f).SetKi(0.0f).SetKd(10.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  shoot_controller.pid().loader_speed.SetKp(8.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
}

void GlobalWarehouse::RCStateUpdate() {
  // if (!globals->device_rc.all_device_ok()) {
  //   globals->StateMachine_ = kUnable;
  // } else {
  switch (globals->rc->switch_r()) {
    case rm::device::DR16::SwitchPosition::kUp:
      // 右拨杆打到最上侧挡位
      switch (globals->rc->switch_l()) {
        case rm::device::DR16::SwitchPosition::kDown:
        case rm::device::DR16::SwitchPosition::kMid:
        case rm::device::DR16::SwitchPosition::kUp:
        default:
          globals->StateMachine_ = kNoForce;
          gimbal->GimbalMove_ = kGbRemote;
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
          gimbal->GimbalMove_ = kGbAimbot;
          break;
        case rm::device::DR16::SwitchPosition::kUp:
        default:
          globals->StateMachine_ = kNoForce;  // 左拨杆拨到下侧，进入比赛模式，此时全部系统都上电工作
          gimbal->GimbalMove_ = kGbRemote;
          break;
      }
      break;

    case rm::device::DR16::SwitchPosition::kDown:
      switch (globals->rc->switch_l()) {
        case rm::device::DR16::SwitchPosition::kUp:
          globals->Music();
        case rm::device::DR16::SwitchPosition::kMid:
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
  // }
}

void GlobalWarehouse::Music() {
  if (globals->rc->dial() >= 650) {
    globals->music = true;
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
  if (music) {
    if (globals->music_choice == 1) {
      globals->buzzer_controller.Play<modules::buzzer_melody::SeeUAgain>();
      globals->music = false;
    }
    if (globals->music_choice == 2) {
      globals->buzzer_controller.Play<modules::buzzer_melody::SuperMario>();
      globals->music = false;
    }
  }
}

void GlobalWarehouse::SubLoop500Hz() {
  globals->imu->Update();
  globals->ahrs.Update(rm::modules::ImuData6Dof{
      globals->imu->gyro_y(), globals->imu->gyro_z(), globals->imu->gyro_x() + globals->yaw_gyro_bias_,
      globals->imu->accel_y(), globals->imu->accel_z(), globals->imu->accel_x()});

  imu_time = HAL_GetTick();
  yaw_set = globals->can_communicator->yaw();
  pitch_set = globals->can_communicator->pitch();
  yaw_ecd = globals->ahrs.euler_angle().yaw;
  pitch_ecd = globals->ahrs.euler_angle().pitch;
  // 激光
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 8399);
  // 硬触发
  if (globals->can_communicator->nuc_start_flag() && globals->device_nuc.all_device_ok()) {
    globals->imu_count++;
    globals->time_camera++;
    if (globals->time_camera == 10) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 65535);
      globals->time_camera = 0;
    }
    if (globals->time_camera == 5) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    }
  } else {
    globals->imu_count = 0;
    globals->time_camera = 0;
  }
  if (globals->imu_count >= 10000) {
    globals->imu_count = 0;
  }
  // can 通信
  globals->can_communicator->UpdateControl(globals->ahrs.quaternion().w, globals->ahrs.quaternion().x,
                                           globals->ahrs.quaternion().y, globals->ahrs.quaternion().z, 103, 0,
                                           globals->imu_count, 22.0f);
  globals->RCStateUpdate();
  gimbal->GimbalTask();
  rm::device::DjiMotor<>::SendCommand(*can1);
  rm::device::DjiMotor<>::SendCommand(*can2);
}

void GlobalWarehouse::SubLoop250Hz() {
  if (globals->time % 2 == 0) {
    globals->pitch_motor->SetPosition(0, 0, gimbal->pitch_torque_, 0, 0);
  }
}

void GlobalWarehouse::SubLoop100Hz() {
  if (globals->time % 5 == 0) {
    if (globals->rc->switch_l() != rm::device::DR16::SwitchPosition::kUnknown &&
        globals->rc->switch_r() != rm::device::DR16::SwitchPosition::kUnknown) {
      if (globals->rc->switch_l() != globals->last_switch_l || globals->rc->switch_r() != globals->last_switch_r) {
        globals->buzzer_controller.Play<modules::buzzer_melody::Beeps<1>>();
        globals->last_switch_l = globals->rc->switch_l();
        globals->last_switch_r = globals->rc->switch_r();
      }
    }
  }
}

void GlobalWarehouse::SubLoop50Hz() {
  if (globals->time % 10 == 0) {
    const auto &[led_r, led_g, led_b] = globals->led_controller.Update();
    (*globals->led)(0xff000000 | led_r << 16 | led_g << 8 | led_b);
    buzzer->SetFrequency(globals->buzzer_controller.Update().frequency);
  }
}

void GlobalWarehouse::SubLoop10Hz() {
  if (globals->time % 50 == 0) {
    globals->time = 0;
  }
}
