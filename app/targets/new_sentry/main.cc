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
  if (globals->time % 2 == 0) globals->SubLoop250Hz();
  if (globals->time % 5 == 0) globals->SubLoop100Hz();
  if (globals->time % 10 == 0) globals->SubLoop50Hz();
  if (globals->time % 50 == 0) globals->SubLoop10Hz();
}

extern "C" [[noreturn]] void AppMain(void) {
  rm::Sleep(std::chrono::milliseconds(100));  // 等待设备初始化完成
  globals = new GlobalWarehouse;
  gimbal = new Gimbal;
  chassis = new Chassis;
  globals->Init();

  for (auto ch : {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4}) {
    HAL_TIM_PWM_Start(&htim1, ch);
  }

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_1000hz.SetPrescalerAndPeriod(168 - 1, 1000 - 1);  // 84MHz / 168 / 1000 = 500Hz
  mainloop_1000hz.Start();

  for (;;) {
    // (void)globals->can1->Process();
    // (void)globals->can2->Process();
    // const auto &can1status = globals->can1->stats();
    // const auto &can2status = globals->can2->stats();
    // __WFI();
  }
}

void GlobalWarehouse::Init() {
  buzzer = new Buzzer;
  led = new LED;

  can1 = new rm::hal::Can{hcan1};
  can2 = new rm::hal::Can{hcan2};
  aimbot_communicator = new rm::device::AimbotCanCommunicator(*can1);
  navigate_communicator = new rm::device::NavigateCanCommunicator(*can2);
  ident_uart = new rm::hal::Serial<128>{huart1, false, true};
  dbus = new rm::hal::Serial<25>{huart3, false, true};
  referee_uart = new rm::hal::Serial<128>{huart6, false, true};

  rx_referee = new rm::device::RxReferee{*globals->referee_uart};
  imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
  hipnuc_imu = new rm::device::HipnucImuCan{*can2, 8};

  wfly_et16s = new WflyET16s{*dbus};
  up_yaw_motor = new rm::device::GM6020{*can2, 1};
  down_yaw_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>  //
      {*can1, {0x07, 0x06, 3.14159, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}}};
  pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>  //
      {*can2, {0x03, 0x02, 3.14159, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}}};
  friction_left = new rm::device::M3508{*can2, 3};
  friction_right = new rm::device::M3508{*can2, 2};
  dial_motor = new rm::device::M3508{*can2, 4};

  referee_data = new rm::device::Referee<rm::device::RefereeRevision::kNewV110>;

  wheel_lf = new rm::device::M3508{*can1, 1};
  wheel_rf = new rm::device::M3508{*can1, 3};
  wheel_lb = new rm::device::M3508{*can1, 4};
  wheel_rb = new rm::device::M3508{*can1, 2};

  device_rc << wfly_et16s;                                         // 遥控器
  device_nuc << aimbot_communicator << navigate_communicator;      // nuc
  device_gimbal << up_yaw_motor << down_yaw_motor << pitch_motor;  // 云台电机
  device_shoot << friction_left << friction_right << dial_motor;   // 发射机构电机
  device_chassis << wheel_lf << wheel_rf << wheel_lb << wheel_rb;  // 底盘电机

  can1->SetFilter(0, 0);
  can1->Begin();
  can2->SetFilter(0, 0);
  can2->Begin();
  wfly_et16s->Begin();
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
  gimbal_controller.pid().up_yaw_position.SetKp(20.0f).SetKi(0.0f).SetKd(120.0f).SetMaxOut(20000.0f).SetMaxIout(0.0f);
  gimbal_controller.pid().up_yaw_speed.SetKp(7000.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(25000.0f).SetMaxIout(0.0f);
  // 下部 Yaw PID 参数
  gimbal_controller.pid().down_yaw_position.SetKp(38.0f).SetKi(0.0f).SetKd(2000.0f).SetMaxOut(30.0f).SetMaxIout(0.0f);
  gimbal_controller.pid().down_yaw_speed.SetKp(2.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10.0f).SetMaxIout(0.0f);
  // pitch PID 参数
  gimbal_controller.pid().pitch_position.SetKp(70.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(30.0f).SetMaxIout(0.0f);
  gimbal_controller.pid().pitch_speed.SetKp(0.6f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10.0f).SetMaxIout(0.0f);
}

void GlobalWarehouse::ChassisPIDInit() {
  chassis_controller.pid().lf_wheel.SetKp(8.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(15000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rf_wheel.SetKp(8.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(15000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().lb_wheel.SetKp(8.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(15000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rb_wheel.SetKp(8.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(15000.0f).SetMaxIout(0.0f);
}

void GlobalWarehouse::ShootPIDInit() {
  shoot_controller.pid().fric_1_speed.SetKp(8.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(16384.0f).SetMaxIout(0.0f);
  shoot_controller.pid().fric_2_speed.SetKp(8.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(16384.0f).SetMaxIout(0.0f);
  shoot_controller.pid().loader_position.SetKp(0.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  shoot_controller.pid().loader_speed.SetKp(8.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
}

void GlobalWarehouse::RCStateUpdate() {
  if (globals->referee_data->data().robot_status.power_management_gimbal_output && !globals->last_gimbal_power) {
    globals->gimbal_init_time = 500;
  }
  globals->last_gimbal_power = globals->referee_data->data().robot_status.power_management_gimbal_output;
  if (globals->gimbal_init_time > 0) {
    globals->gimbal_init_time--;
  }
  if (!globals->device_rc.all_device_ok() || globals->gimbal_init_time > 0 ||
      !globals->referee_data->data().robot_status.power_management_gimbal_output) {
    globals->StateMachine_ = kUnable;
  } else {
    switch (globals->wfly_et16s->switch_position(rc_ch::SD)) {
      case SwitchPosition::kUp:
        // 右拨杆打到最上侧挡位
        switch (globals->wfly_et16s->switch_position(rc_ch::SA)) {
          case SwitchPosition::kDown:
            globals->StateMachine_ = kMatch;  // 左拨杆拨到下侧，进入比赛模式，此时全部系统都上电工作
            break;
          case SwitchPosition::kUp:
          case SwitchPosition::kMid:
          default:
            globals->StateMachine_ = kNoForce;  // 左拨杆拨到下侧，进入比赛模式，此时全部系统都上电工作
            break;
        }
        break;

      case SwitchPosition::kMid:
        // 右拨杆打到中间挡位
        switch (globals->wfly_et16s->switch_position(rc_ch::SA)) {
          case SwitchPosition::kDown:
            if (globals->wfly_et16s->switch_position(rc_ch::SE) == SwitchPosition::kMid) {
              globals->StateMachine_ = kTest;  // 左拨杆拨到下侧，进入测试模式
              gimbal->GimbalMove_ = kGbIdentify;
              chassis->ChassisMove_ = kNoForce;
            } else if (globals->wfly_et16s->switch_position(rc_ch::SE) == SwitchPosition::kDown) {
              globals->StateMachine_ = kTest;  // 左拨杆拨到下侧，进入测试模式
              gimbal->GimbalMove_ = kGbFfVerify;
              chassis->ChassisMove_ = kNoForce;
            } else {
              globals->StateMachine_ = kTest;  // 左拨杆拨到下侧，进入测试模式
              gimbal->GimbalMove_ = kGbRemote;
              chassis->ChassisMove_ = kCsRemote;
            }
            break;
          case SwitchPosition::kMid:
            globals->StateMachine_ = kTest;
            gimbal->GimbalMove_ = kGbScan;
            chassis->ChassisMove_ = kCsNavigate;
            break;
          case SwitchPosition::kUp:
            globals->StateMachine_ = kTest;
            gimbal->GimbalMove_ = kGbAimbot;
            chassis->ChassisMove_ = kNoForce;
            break;
          default:
            globals->StateMachine_ = kNoForce;  // 左拨杆拨到下侧，进入比赛模式，此时全部系统都上电工作
            break;
        }
        break;

      case SwitchPosition::kDown:
        switch (globals->wfly_et16s->switch_position(rc_ch::SA)) {
          case SwitchPosition::kUp:
            globals->Music();
            globals->StateMachine_ = kNoForce;  // 左拨杆拨到下侧，进入比赛模式，此时全部系统都上电工作
          case SwitchPosition::kMid:
          case SwitchPosition::kDown:
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
  if (globals->wfly_et16s->wheel_position(rc_ch::LS) >= 650 && globals->music == 0 &&
      globals->wfly_et16s->switch_position(rc_ch::SH) == SwitchPosition::kDown) {
    globals->music = 1;
  }
  if (globals->wfly_et16s->wheel_position(rc_ch::LS) <= -650 && globals->music_change_flag == false &&
      globals->wfly_et16s->switch_position(rc_ch::SH) == SwitchPosition::kDown) {
    globals->music_choice++;
    if (globals->music_choice == 3) {
      globals->music_choice = 0;
    }
    globals->buzzer_controller.Play<modules::buzzer_melody::Beeps<1>>();
    globals->music_change_flag = true;
  }
  if (globals->wfly_et16s->switch_position(rc_ch::SH) == SwitchPosition::kUp &&
      (globals->music == 0 || globals->music == 2)) {
    globals->music_change_flag = false;
    globals->music = 0;
  }
  if (globals->music == 1) {
    if (globals->music_choice == 1) {
      globals->buzzer_controller.Play<modules::buzzer_melody::SeeUAgain>();
      globals->music = 2;
    }
    if (globals->music_choice == 2) {
      globals->buzzer_controller.Play<modules::buzzer_melody::SuperMario>();
      globals->music = 2;
    }
  }
}

void GlobalWarehouse::SubLoop500Hz() {
  globals->imu->Update();
  globals->ahrs.Update(rm::modules::ImuData6Dof{globals->imu->gyro_x(), globals->imu->gyro_y(),
                                                globals->imu->gyro_z() - 0.0015f, globals->imu->accel_x(),
                                                globals->imu->accel_y(), globals->imu->accel_z()});
  rm::device::DjiMotorBase::SendCommand(*can1);
  rm::device::DjiMotorBase::SendCommand(*can2);
  globals->RCStateUpdate();
  gimbal->GimbalTask();
  chassis->ChassisTask();
  f32 shoot_initial_speed = 0.0f;
  if (globals->referee_data->data().shoot_data.initial_speed >= 18.f &&
      globals->referee_data->data().shoot_data.initial_speed <= 26.f) {
    shoot_initial_speed = globals->referee_data->data().shoot_data.initial_speed;
  } else {
    shoot_initial_speed = 22.5f;
  }
  globals->aimbot_communicator->UpdateControl(
      globals->hipnuc_imu->yaw(), globals->hipnuc_imu->pitch(), -globals->hipnuc_imu->roll(),
      globals->referee_data->data().robot_status.robot_id, globals->aim_mode, globals->imu_count, shoot_initial_speed);
  globals->down_yaw_motor->SetMitCommand(0, 0, -globals->gimbal_controller.output().down_yaw, 0, 0);
  globals->pitch_motor->SetMitCommand(0, 0, -gimbal->pitch_torque_, 0, 3.4f);
}

void GlobalWarehouse::SubLoop250Hz() {}

void GlobalWarehouse::SubLoop100Hz() {
  globals->device_rc.Update();
  globals->device_nuc.Update();
  globals->device_gimbal.Update();
  globals->device_shoot.Update();
  globals->device_chassis.Update();
  for (i8 i = 0; i < 7; i++) {
    if (globals->wfly_et16s->switch_position(i + 4) != SwitchPosition::kUnknown) {
      if (globals->last_switch[i] != globals->wfly_et16s->switch_position(i + 4)) {
        globals->buzzer_controller.Play<modules::buzzer_melody::Beeps<1>>();
        globals->last_switch[i] = globals->wfly_et16s->switch_position(i + 4);
      }
    }
  }
}

void GlobalWarehouse::SubLoop50Hz() {
  gimbal->GimbalIdentifyDataSend();
  const auto &[led_r, led_g, led_b] = globals->led_controller.Update();
  (*globals->led)(0xff000000 | led_r << 16 | led_g << 8 | led_b);
  buzzer->SetFrequency(globals->buzzer_controller.Update().frequency);
}

void GlobalWarehouse::SubLoop10Hz() { globals->time = 0; }
