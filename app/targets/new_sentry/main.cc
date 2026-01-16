#include <librm.hpp>

#include "can.h"
#include "usart.h"
#include "spi.h"

#include "timer_task.hpp"

#include "main.hpp"
#include "Gimbal.hpp"
#include "Chassis.hpp"

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
  chassis = new Chassis;
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
  dbus = new rm::hal::Serial{huart3, 18, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
  referee_uart = new rm::hal::Serial{huart6, 128, hal::stm32::UartMode::kNormal, hal::stm32::UartMode::kDma};

  rx_referee = new rm::device::RxReferee{*globals->referee_uart};
  referee_data_buffer = new rm::device::Referee<rm::device::RefereeRevision::kV170>;
  imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
  rc = new rm::device::DR16{*dbus};
  up_yaw_motor = new rm::device::GM6020{*can1, 5};
  down_yaw_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>  //
      {*can2, {0x05, 0x04, 12.56637f, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}}};
  pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>  //
      {*can1, {0x03, 0x02, 12.56637f, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}}};
  friction_left = new rm::device::M3508{*can1, 7};
  friction_right = new rm::device::M3508{*can1, 6};
  dial_motor = new rm::device::M2006{*can1, 8};

  wheel_lf = new rm::device::M3508{*can2, 1};
  wheel_rf = new rm::device::M3508{*can2, 3};
  wheel_lb = new rm::device::M3508{*can2, 2};
  wheel_rb = new rm::device::M3508{*can2, 4};

  device_rc << rc;                                                 // 遥控器
  device_gimbal << up_yaw_motor << down_yaw_motor << pitch_motor;  // 云台电机
  device_shoot << friction_left << friction_right << dial_motor;   // 发射机构电机
  device_chassis << wheel_lf << wheel_rf << wheel_lb << wheel_rb;  // 底盘电机

  can1->SetFilter(0, 0);
  can1->Begin();
  can2->SetFilter(0, 0);
  can2->Begin();
  rc->Begin();
  rx_referee->Begin();
  buzzer->Init();
  led->Init();

  led_controller.SetPattern<modules::led_pattern::GreenBreath>();
  buzzer_controller.Play<modules::buzzer_melody::Startup>();

  globals->GimbalPIDInit();
  globals->ChassisPIDInit();
  globals->ShootPIDInit();
  gimbal->GimbalInit();
  chassis->ChassisInit();
}

void GlobalWarehouse::GimbalPIDInit() {
  // 初始化PID
  // 上部 Yaw PID 参数
  gimbal_controller.pid().up_yaw_position.SetKp(0.25f).SetKi(0.0f).SetKd(0.25f).SetMaxOut(35000.0f).SetMaxIout(0.0f);
  gimbal_controller.pid().up_yaw_speed.SetKp(520.0f).SetKi(0.0f).SetKd(50.0f).SetMaxOut(16384.0f).SetMaxIout(0.0f);
  // 下部 Yaw PID 参数
  gimbal_controller.pid().down_yaw_position.SetKp(60.0f).SetKi(0.0f).SetKd(2600.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  gimbal_controller.pid().down_yaw_speed.SetKp(1.8f).SetKi(0.0f).SetKd(4.5f).SetMaxOut(10.0f).SetMaxIout(0.0f);
  // pitch PID 参数
  gimbal_controller.pid().pitch_position.SetKp(15.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  gimbal_controller.pid().pitch_speed.SetKp(1.6f).SetKi(0.0f).SetKd(4.5f).SetMaxOut(10.0f).SetMaxIout(0.0f);
}

void GlobalWarehouse::ChassisPIDInit() {
  chassis_controller.pid().lf_wheel.SetKp(4.0f).SetKi(0.0f).SetKd(5.0f).SetMaxOut(6000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rf_wheel.SetKp(4.0f).SetKi(0.0f).SetKd(5.0f).SetMaxOut(6000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().lb_wheel.SetKp(4.0f).SetKi(0.0f).SetKd(5.0f).SetMaxOut(6000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rb_wheel.SetKp(4.0f).SetKi(0.0f).SetKd(5.0f).SetMaxOut(6000.0f).SetMaxIout(0.0f);
}

void GlobalWarehouse::ShootPIDInit() {
  shoot_controller.pid().fric_1_speed.SetKp(8.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(16384.0f).SetMaxIout(0.0f);
  shoot_controller.pid().fric_2_speed.SetKp(8.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(16384.0f).SetMaxIout(0.0f);
  shoot_controller.pid().loader_position.SetKp(500.0f).SetKi(0.0f).SetKd(10.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  shoot_controller.pid().loader_speed.SetKp(8.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
}

void GlobalWarehouse::RCStateUpdate() {
  if (!globals->device_rc.all_device_ok()) {
    globals->StateMachine_ = kUnable;
  } else {
    switch (globals->rc->switch_r()) {
      case rm::device::DR16::SwitchPosition::kUp:
        // 右拨杆打到最上侧挡位
        switch (globals->rc->switch_l()) {
          case rm::device::DR16::SwitchPosition::kDown:
            globals->StateMachine_ = kMatch;  // 左拨杆拨到下侧，进入比赛模式，此时全部系统都上电工作
            break;
          case rm::device::DR16::SwitchPosition::kUp:
            globals->StateMachine_ = kMatch;
            gimbal->GimbalMove_ = kGbAimbot;
            chassis->ChassisMove_ = kNoForce;
          case rm::device::DR16::SwitchPosition::kMid:
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
            chassis->ChassisMove_ = kCsRemote;
            break;
          case rm::device::DR16::SwitchPosition::kMid:
            globals->StateMachine_ = kTest;
            gimbal->GimbalMove_ = kGbNavigate;
            chassis->ChassisMove_ = kCsNavigate;
            break;
          case rm::device::DR16::SwitchPosition::kUp:
            globals->StateMachine_ = kTest;
            gimbal->GimbalMove_ = kGbAimbot;
            chassis->ChassisMove_ = kNoForce;
            break;
          default:
            globals->StateMachine_ = kNoForce;  // 左拨杆拨到下侧，进入比赛模式，此时全部系统都上电工作
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
  }
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
  globals->RCStateUpdate();
  gimbal->GimbalTask();
  chassis->ChassisTask();
  // rm::device::DjiMotor<>::SendCommand(*can1);
  // rm::device::DjiMotor<>::SendCommand(*can2);
  if (USB_selection) {
    GimbalDataSend();
    USB_selection ^= 1;
  } else {
    RefereeDataSend();
    USB_selection ^= 1;
  }
}

void GlobalWarehouse::SubLoop250Hz() {
  if (globals->time % 2 == 0) {
    // globals->down_yaw_motor->SetPosition(0, 0, globals->gimbal_controller.output().down_yaw, 0, 0);
    // globals->pitch_motor->SetPosition(0, 0, gimbal->pitch_torque_, 0, 0);
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
