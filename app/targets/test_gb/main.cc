#include <librm.hpp>

#include "can.h"
#include "usart.h"
#include "spi.h"

#include "timer_task.hpp"

#include "main.hpp"
#include "Gimbal.hpp"

using namespace rm;

void MainLoop() {
  globals->time_++;
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

  for (auto ch : {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4}) {
    HAL_TIM_PWM_Start(&htim1, ch);
  }
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

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

void GlobalWarehouse::Init() {
  buzzer = new Buzzer;
  led = new LED;

  can1 = new rm::hal::Can{hcan1};
  can2 = new rm::hal::Can{hcan2};
  can_communicator = new device::AimbotCanCommunicator(*can2);
  dbus = new rm::hal::Serial{huart3, 18, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
  imu_uart = new rm::hal::Serial{huart1, 1024, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};

  rc = new device::DR16{*dbus};
  imu = new device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
  hipnuc_imu = new device::HipnucImuCan{*can2, 8};
  yaw_motor = new device::DmMotor<device::DmMotorControlMode::kMit>  //
      {*can1, {0x12, 0x02, 3.141593f, 30.0f, 10.0f, {0.f, 500.f}, {0.f, 5.f}}};
  pitch_motor = new device::DmMotor<device::DmMotorControlMode::kMit>  //
      {*can1, {0x11, 0x01, 3.141593f, 30.0f, 10.0f, {0.f, 500.f}, {0.f, 5.f}}};

  can1->SetFilter(0, 0);
  can2->SetFilter(0, 0);
  can1->Begin();
  can2->Begin();
  rc->Begin();
  // hipnuc_imu->Begin();
  buzzer->Init();
  led->Init();

  device_rc << rc;                            // 遥控器
  device_gimbal << yaw_motor << pitch_motor;  // 云台电机
  device_nuc << can_communicator;

  // device_gimbal.OnDeviceFaultOrOffline([&](device::Device *offline_device) {
  //   if (time_offline[0] == 0) {
  //     if (offline_device == yaw_motor) {
  //       buzzer_controller.Play<modules::buzzer_melody::Beeps<1>>();
  //     }
  //     if (offline_device == pitch_motor) {
  //       buzzer_controller.Play<modules::buzzer_melody::Beeps<2>>();
  //     }
  //   }
  //   time_offline[0]++;
  //   if (time_offline[0] >= 1000) {
  //     time_offline[0] = 0;
  //   }
  // });

  // device_nuc.OnDeviceFaultOrOffline([&](device::Device *offline_device) {
  //   if (time_offline[0] == 0) {
  //     if (offline_device == can_communicator) {
  //       led_controller.SetPattern<modules::led_pattern::RedFlash>();
  //     }
  //   }
  //   time_offline[0]++;
  //   if (time_offline[0] >= 1000) {
  //     time_offline[0] = 0;
  //   }
  // });

  led_controller.SetPattern<modules::led_pattern::GreenBreath>();
  buzzer_controller.Play<modules::buzzer_melody::Startup>();

  globals->GimbalPIDInit();
  gimbal->GimbalInit();
}

void GlobalWarehouse::GimbalPIDInit() {
  // 初始化PID
  // Yaw PID 参数
  gimbal_controller.pid().yaw_position.SetKp(20.0f).SetKi(0.0f).SetKd(3.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  gimbal_controller.pid().yaw_speed.SetKp(0.4f).SetKi(0.0f).SetKd(0.2f).SetMaxOut(10.0f).SetMaxIout(0.0f);
  // pitch PID 参数
  gimbal_controller.pid().pitch_position.SetKp(18.0f).SetKi(0.0f).SetKd(2.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  gimbal_controller.pid().pitch_speed.SetKp(0.4f).SetKi(0.0f).SetKd(0.15f).SetMaxOut(10.0f).SetMaxIout(0.0f);
}

void GlobalWarehouse::RCStateUpdate() {
  if (globals->device_rc.all_device_ok()) {
    switch (globals->rc->switch_r()) {
      case device::DR16::SwitchPosition::kUp:
        // 右拨杆打到最上侧挡位
        switch (globals->rc->switch_l()) {
          case device::DR16::SwitchPosition::kDown:
          case device::DR16::SwitchPosition::kMid:
          case device::DR16::SwitchPosition::kUp:
          default:
            globals->StateMachine_ = kNoForce;  // 左拨杆拨到下侧，进入比赛模式，此时全部系统都上电工作
            break;
        }
        break;

      case device::DR16::SwitchPosition::kMid:
        // 右拨杆打到中间挡位
        switch (globals->rc->switch_l()) {
          case device::DR16::SwitchPosition::kDown:
            globals->StateMachine_ = kTest;  // 左拨杆拨到下侧，进入测试模式
            gimbal->GimbalMove_ = kGbRemote;
            break;
          case device::DR16::SwitchPosition::kMid:
            globals->StateMachine_ = kTest;
            gimbal->GimbalMove_ = kGbAimbot;
            break;
          case device::DR16::SwitchPosition::kUp:
          default:
            globals->StateMachine_ = kNoForce;
            break;
        }
        break;

      case device::DR16::SwitchPosition::kDown:
      default:
        globals->StateMachine_ = kNoForce;  // 如果遥控器离线，进入无力模式
        break;
    }
  }
}

void GlobalWarehouse::SubLoop500Hz() {
  // imu 解算
  // globals->imu->Update();
  // globals->ahrs.Update(  //
  //     rm::modules::ImuData6Dof{-globals->imu->gyro_x(), -globals->imu->gyro_y(), globals->imu->gyro_z(),
  //                              -globals->imu->accel_x(), -globals->imu->accel_y(), globals->imu->accel_z()});
  imu_time = HAL_GetTick();
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
  globals->can_communicator->UpdateQuaternion(globals->ahrs.quaternion().w, globals->ahrs.quaternion().x,
                                              globals->ahrs.quaternion().y, globals->ahrs.quaternion().z);
  globals->can_communicator->UpdateControlFlag(0, globals->aim_mode, globals->imu_count, globals->imu_time);

  globals->RCStateUpdate();
  gimbal->GimbalTask();
}

void GlobalWarehouse::SubLoop250Hz() {
  if (globals->time_ % 2 == 0) {
    globals->yaw_motor->SetPosition(0, 0, globals->gimbal_controller.output().yaw, 0, 0);
    globals->pitch_motor->SetPosition(0, 0, globals->gimbal_controller.output().pitch, 0, 0);
  }
}

void GlobalWarehouse::SubLoop100Hz() {
  if (globals->time_ % 5 == 0) {
    // 在线检测
    globals->device_rc.Update();
    globals->device_gimbal.Update();
    globals->device_nuc.Update();
    GimbalDataSend();
  }
}

void GlobalWarehouse::SubLoop50Hz() {
  if (globals->time_ % 10 == 0) {
    const auto &[led_r, led_g, led_b] = globals->led_controller.Update();
    (*globals->led)(0xff000000 | led_r << 16 | led_g << 8 | led_b);
    buzzer->SetFrequency(globals->buzzer_controller.Update().frequency);
  }
}

void GlobalWarehouse::SubLoop10Hz() {
  if (globals->time_ % 50 == 0) {
    globals->time_ = 0;
  }
}
