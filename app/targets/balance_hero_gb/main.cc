#include "can.h"
#include "usart.h"
#include "spi.h"

#include "timer_task.hpp"

#include "main.hpp"
#include "Gimbal.hpp"
#include <cmath>

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
  chassis_communicate = new Communicate{*can2};

  rc = new rm::device::DR16{*dbus};
  imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
  yaw_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>  //
      {*can2, {0x21, 0x11, 3.141593f, 30.0f, 10.0f, {0.f, 500.f}, {0.f, 5.f}}};
  pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>  //
      {*can2, {0x13, 0x12, 3.141593f, 30.0f, 10.0f, {0.f, 500.f}, {0.f, 5.f}}};
  shoot_controller = new Shoot_Controller{*can2, *can1};

  yaw_speed_feedforward = new YawSpeedFeedforward(0.002, 1);

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
  globals->ShootInit();
  shoot_controller->Init();
}

void GlobalWarehouse::GimbalPIDInit() {
  // 初始化PID
  // Yaw PID 参数
  gimbal_controller.pid().yaw_position.SetKp(20.0f).SetKi(0.0f).SetKd(3.f).SetMaxOut(10000.0f).SetMaxIout(0.f);
  gimbal_controller.pid().yaw_speed.SetKp(0.6f).SetKi(0.04f).SetKd(0.2f).SetMaxOut(10.0f).SetMaxIout(0.8f);
  // pitch PID 参数
  gimbal_controller.pid().pitch_position.SetKp(20.0f).SetKi(0.f).SetKd(0.f).SetMaxOut(10000.0f).SetMaxIout(0.f);
  gimbal_controller.pid().pitch_speed.SetKp(0.4f).SetKi(0.f).SetKd(0.f).SetMaxOut(10.0f).SetMaxIout(0.f);
}

void GlobalWarehouse::ShootInit() {}

void GlobalWarehouse::RCStateUpdate() {
  // if (globals->device_rc.all_device_ok())
  switch (globals->rc->switch_r()) {
    case rm::device::DR16::SwitchPosition::kUp:
      // 右拨杆打到最上侧挡位
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
          gimbal->GimbalMove_ = kNoForce;  // 左拨杆拨到下侧，进入比赛模式，此时全部系统都上电工作
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
          globals->StateMachine_ = kTest;  // 左拨杆拨到下侧，进入测试模式
          gimbal->GimbalMove_ = kGbRemote;
          break;
        case rm::device::DR16::SwitchPosition::kUp:
          globals->StateMachine_ = kTest;  // 左拨杆拨到下侧，进入测试模式
          gimbal->GimbalMove_ = kGbRemote;
          break;
        default:
          globals->StateMachine_ = kNoForce;
          gimbal->GimbalMove_ = kNoForce;
          break;
      }
      break;

    case rm::device::DR16::SwitchPosition::kDown:
      globals->StateMachine_ = kNoForce;
      gimbal->GimbalMove_ = kNoForce;
      break;
    default:
      globals->StateMachine_ = kNoForce;  // 如果遥控器离线，进入无力模式
      gimbal->GimbalMove_ = kNoForce;
      break;
  }
}

void GlobalWarehouse::CommunicateUpdate() {
  ChassisState target_state = ChassisState::UNABLE;
  switch (rc->switch_r()) {
    case DR16::SwitchPosition::kMid:
      target_state = ChassisState::FOLLOW;
      break;
    case DR16::SwitchPosition::kUp:
      target_state = ChassisState::ROTATE;
      break;
    case DR16::SwitchPosition::kDown:
      target_state = ChassisState::UNABLE;
      break;
    default:
      target_state = ChassisState::UNABLE;
      break;
  }

  static ChassisState last_raw_state = ChassisState::UNABLE;
  static bool is_yaw_aligning = false;

  // 检测到从 UNABLE 切换到 FOLLOW 或 ROTATE 时，启动偏航对齐
  if (target_state != last_raw_state) {
    if ((target_state == ChassisState::FOLLOW || target_state == ChassisState::ROTATE) &&
        last_raw_state == ChassisState::UNABLE) {
      is_yaw_aligning = true;
    } else if (target_state == ChassisState::UNABLE) {
      is_yaw_aligning = false;
    }
    last_raw_state = target_state;
  }

  // 当准备进入跟随或小陀螺模式时，若 yaw 未到达特定角度，则先发送 UNABLE 并转动 yaw 电机
  float target_yaw_angle = -0.981f;  // 示例：特定的偏航角度
  if (is_yaw_aligning) {
    if (std::abs(yaw_motor->pos() - target_yaw_angle) > 0.02f) {
      mychassis._command.chassis.state = ChassisState::UNABLE;
      gimbal_controller.SetTarget(target_yaw_angle, 0, 0);
      gimbal_controller.Update(yaw_motor->pos(), yaw_motor->vel(), 0, 0);
      gimbal->SetGimbalYawTarget(ahrs.euler_angle().yaw);
    } else {
      is_yaw_aligning = false;
      mychassis._command.chassis.state = target_state;
    }
  } else {
    mychassis._command.chassis.state = target_state;
  }

  switch (rc->switch_l()) {
    case DR16::SwitchPosition::kUp:
      mychassis._command.chassis.leg_length = LegLength::HIGH;
      break;
    case DR16::SwitchPosition::kMid:
      mychassis._command.chassis.leg_length = LegLength::NORMAL;
      break;
    case DR16::SwitchPosition::kDown:
      mychassis._command.chassis.leg_length = LegLength::LOW;
      break;
    default:
      mychassis._command.chassis.leg_length = LegLength::LOW;
      break;
  }

  mychassis._command.chassis.state = ChassisState::UNABLE;

  mychassis._command.chassis.move_x = static_cast<int8_t>(rc->left_x() * 127.0f / 660.0f);
  mychassis._command.chassis.move_y = static_cast<int8_t>(rc->left_y() * 127.0f / 660.0f);

  mychassis._command.ui.ui1 = 0;
  mychassis._command.ui.ui2 = 0;
  mychassis._command.ui.ui3 = 0;
  mychassis._command.ui.ui4 = 0;

  chassis_communicate->SendChassisCommand();
}

void GlobalWarehouse::SubLoop500Hz() {
  // imu 解算
  globals->imu->Update();
  globals->ahrs.Update(  //
      rm::modules::ImuData6Dof{-globals->imu->gyro_x(), -globals->imu->gyro_y(), globals->imu->gyro_z(),
                               -globals->imu->accel_x(), -globals->imu->accel_y(), globals->imu->accel_z()});

  globals->RCStateUpdate();
  gimbal->GimbalTask();
  globals->CommunicateUpdate();
  shoot_controller->Task();

  // globals->yaw_motor->SetPosition(0, 0, gimbal_controller.output().yaw ,0, 0);
  globals->yaw_motor->SetPosition(0, 0, 0, 0, 0);
  globals->pitch_motor->SetPosition(0, 0, 0, 0, 0);

  Deubg();
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

f32 fric_l, fric_r, fric_up, yaw_mot, pitch_mot, dial_mot_pos, dial_mot_rpm, yaw, pitch, roll, yaw_t, pitch_t, left_x,
    yaw_satus;
rm::device::DR16::SwitchPosition left_s, right_s;
f32 yaw_pid_out, pitch_pid_out, fric_l_out, fric_r_out, fric_up_out, yaw_pos;
void GlobalWarehouse::Deubg() {
  // fric_l = friction_left->rpm();
  // fric_r = friction_right->rpm();
  // fric_up = friction_up->rpm();
  yaw_mot = yaw_motor->pos();
  pitch_mot = pitch_motor->pos();
  // dial_mot_pos = dial_motor->pos();
  // dial_mot_rpm = dial_motor->vel();
  yaw = ahrs.euler_angle().yaw;
  pitch = ahrs.euler_angle().pitch;
  roll = ahrs.euler_angle().roll;
  yaw_t = gimbal_controller.target().yaw_position;
  pitch_t = gimbal_controller.target().pitch_position;
  left_x = rc->left_x();
  left_s = rc->switch_l();
  right_s = rc->switch_r();
  yaw_satus = yaw_motor->status();
  yaw_pid_out = gimbal_controller.pid().yaw_position.out();
  pitch_pid_out = gimbal_controller.pid().pitch_position.out();
  yaw_pos = yaw_motor->pos();
}
