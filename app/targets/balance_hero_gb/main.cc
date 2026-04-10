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
  dbus = new rm::hal::Serial{huart3, 18, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};

  rc = new rm::device::DR16{*dbus};
  imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
  yaw_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>  //
      {*can2, {0x21, 0x11, 3.141593f, 30.0f, 10.0f, {0.f, 500.f}, {0.f, 5.f}}};
  pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>  //
      {*can2, {0x22, 0x12, 3.141593f, 30.0f, 10.0f, {0.f, 500.f}, {0.f, 5.f}}};
  dial_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>
      {*can2, {0x10, 0x09, 3.141593f, 30.0f, 10.0f, {0.f, 500.f}, {0.f, 5.f}}};
  friction_left = new rm::device::M3508{*can1, 2};
  friction_right = new rm::device::M3508{*can1, 3};
  friction_up = new rm::device::M3508{*can1, 1};

  yaw_speed_feedforward = new YawSpeedFeedforward(0.002, 1);
  shoot3_fric = new Shoot3Fric(6,1.f);

  can1->SetFilter(0, 0);
  can2->SetFilter(0, 0);
  can1->Begin();
  can2->Begin();
  rc->Begin();
  buzzer->Init();
  led->Init();
  device_rc << rc;                            // 遥控器
  device_gimbal << yaw_motor << pitch_motor;  // 云台电机

  led_controller.SetPattern<modules::led_pattern::GreenBreath>();
  buzzer_controller.Play<modules::buzzer_melody::Startup>();

  globals->GimbalPIDInit();
  gimbal->GimbalInit();
}

void GlobalWarehouse::GimbalPIDInit() {
  // 初始化PID
  // Yaw PID 参数
  gimbal_controller.pid().yaw_position.SetKp(20.0f).SetKi(0.0f).SetKd(3.f).SetMaxOut(10000.0f).SetMaxIout(0.f);
  gimbal_controller.pid().yaw_speed.SetKp(0.4f).SetKi(0.0f).SetKd(0.2f).SetMaxOut(10.0f).SetMaxIout(0.f);
  // pitch PID 参数
  gimbal_controller.pid().pitch_position.SetKp(18.0f).SetKi(0.f).SetKd(2.0f).SetMaxOut(10000.0f).SetMaxIout(0.f);
  gimbal_controller.pid().pitch_speed.SetKp(0.4f).SetKi(0.f).SetKd(0.15f).SetMaxOut(10.0f).SetMaxIout(0.f);
}

void GlobalWarehouse::RCStateUpdate() {
  if (globals->device_rc.all_device_ok()) switch (globals->rc->switch_r()) {
      case rm::device::DR16::SwitchPosition::kUp:
        // 右拨杆打到最上侧挡位
        switch (globals->rc->switch_l()) {
          case rm::device::DR16::SwitchPosition::kDown:
          case rm::device::DR16::SwitchPosition::kMid:
          case rm::device::DR16::SwitchPosition::kUp:
          default:
            globals->StateMachine_ = kNoForce;  // 左拨杆拨到下侧，进入比赛模式，此时全部系统都上电工作
            break;
        }
        break;

      case rm::device::DR16::SwitchPosition::kMid:
        // 右拨杆打到中间挡位
        switch (globals->rc->switch_l()) {
          case rm::device::DR16::SwitchPosition::kDown:
            globals->StateMachine_ = kTest;  // 左拨杆拨到下侧，进入测试模式
            gimbal->GimbalMove_ = kGbRemote;
            break;
          case rm::device::DR16::SwitchPosition::kMid:
            globals->StateMachine_ = kNoForce;
            gimbal->GimbalMove_ = kNoForce;
            break;
          case rm::device::DR16::SwitchPosition::kUp:
            globals->StateMachine_ = kNoForce;
            gimbal->GimbalMove_ = kNoForce;
            break;
          default:
            globals->StateMachine_ = kNoForce;
            break;
        }
        break;

      case rm::device::DR16::SwitchPosition::kDown:
      default:
        globals->StateMachine_ = kNoForce;  // 如果遥控器离线，进入无力模式
        break;
    }
}

void GlobalWarehouse::SubLoop500Hz() {
  // imu 解算
  globals->imu->Update();
  globals->ahrs.Update(  //
      rm::modules::ImuData6Dof{-globals->imu->gyro_x(), -globals->imu->gyro_y(), globals->imu->gyro_z(),
                               -globals->imu->accel_x(), -globals->imu->accel_y(), globals->imu->accel_z()});

  globals->RCStateUpdate();
  gimbal->GimbalTask();

  // globals->yaw_motor->SetPosition(0, 0, globals->gimbal_controller.output().yaw, 0, 0);
  // globals->pitch_motor->SetPosition(0, 0, globals->gimbal_controller.output().pitch, 0, 0);
  globals->yaw_motor->SetPosition(0, 0, 0 ,0, 0);
  globals->pitch_motor->SetPosition(0, 0, 0, 0, 0);
}

void GlobalWarehouse::SubLoop250Hz() {
  if (globals->time_ % 2 == 0) {
  }
}

void GlobalWarehouse::SubLoop100Hz() {
  if (globals->time_ % 5 == 0) {
    // 在线检测
    globals->device_rc.Update();
    globals->device_gimbal.Update();
    // globals->device_nuc.Update();
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
