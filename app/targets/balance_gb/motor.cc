#include <librm.hpp>

#include "can.h"

#include "motor.hpp"
#include "global.hpp"
#include "firstorderfilter.hpp"
#include "usart.h"

FirstOrderFilter pitch_vel_filter(1.f / 500.f, 0.006f);

using namespace rm::modules;

rm::hal::Can *can1;
rm::hal::Can *can2;

f32 gravity_compensation_ = 0.f;
f32 pitch_pid_debug, yaw_pid_debug;
f32 yaw_pos;
i16 debug;
i16 rpm_left, rpm_right;
f32 left_x;
u8 status;
f32 yaw_target,pitch_target;
static float current_vel = 0.0f;
// extern Debug pid_debug;

void Motor::MotorInit() {
  can1 = new rm::hal::Can{hcan1};
  can2 = new rm::hal::Can{hcan2};

  pitch_motor = new DmMotor<DmMotorControlMode::kMit>  //
      {*can1, {0x19, 0x09, 3.141593f, 30.f, 10.f, std::make_pair(0.0f, 500.0f), std::make_pair(0.0f, 5.0f)}};
  yaw_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>  //
      {*can2, {0x04, 0x03, 3.141593f, 30.f, 10.f, std::make_pair(0.0f, 500.0f), std::make_pair(0.0f, 5.0f)}};

  aimbot_comm = new device::AimbotCanCommunicator(*can1);
  ammo_left = new M3508{*can1, 7};
  ammo_right = new M3508{*can1, 8};
  dial_motor = new M3508{*can2, 1};

  yaw_feedforward = new YawSpeedFeedforward(0.002,-1);

  //vofa_plotter = new VofaPlotter;

  //sweep_controller = new SineSweep(0.f, 2.f,);

  can1->SetFilter(0, 0);
  can1->Begin();
  can2->SetFilter(0, 0);
  can2->Begin();
}

void Motor::CalcYawPos(f32 pos) { yaw_motor_pos = -yaw_motor->pos() + 0.85f; };

void Motor::MotorPidInit() {
  // 初始pid参数
  //  yaw电机pid
  gimbal_controller.pid().yaw_position.SetKp(-18.f).SetKi(0.f).SetKd(-350.f).SetMaxOut(20.f).SetMaxIout(0.f);
  gimbal_controller.pid().yaw_speed.SetKp(1.2f).SetKi(0.f).SetKd(0.5f).SetMaxOut(6.f).SetMaxIout(0.f);

  // pitch电机pid
  gimbal_controller.pid().pitch_position.SetKp(32.f).SetKi(0.f).SetKd(100.f).SetMaxOut(10.f).SetMaxIout(0.f);
  gimbal_controller.pid().pitch_speed.SetKp(1.1f).SetKi(0.01f).SetKd(0.f).SetMaxOut(0.f).SetMaxIout(0.4f);

  // 摩擦轮电机
  shoot_controller.pid().fric_1_speed.SetKp(700.f).SetKi(0.f).SetKd(0.f).SetMaxOut(16384.f).SetMaxIout(0.f);
  shoot_controller.pid().fric_2_speed.SetKp(700.f).SetKi(0.f).SetKd(0.f).SetMaxOut(16384.f).SetMaxIout(0.f);

  // 拨盘电机
  shoot_controller.pid().loader_position.SetKp(25.f).SetKi(0.f).SetKd(0.f).SetMaxOut(1000.f).SetMaxIout(0.f);
  shoot_controller.pid().loader_speed.SetKp(2.f).SetKi(0.f).SetKd(0.f).SetMaxOut(16384.f).SetMaxIout(0.f);
}

/*
@brief:电机失能或者失能
*/
void Motor::DMEnable() {
  gimbal_controller.Enable(true);
  if (DMEnable_ == 1) {
    yaw_motor->SendInstruction(DmMotorInstructions::kClearError);
    pitch_motor->SendInstruction(DmMotorInstructions::kClearError);
    pitch_motor->SendInstruction(DmMotorInstructions::kEnable);  // 使达妙电机使能
    yaw_motor->SendInstruction(DmMotorInstructions::kEnable);
    DMEnable_ = 0;
  }
}

void Motor::DMDisable() {
  gimbal_controller.Enable(false);
  if (DMEnable_ == 0) {
    pitch_motor->SendInstruction(DmMotorInstructions::kDisable);  // 使达妙电机失能
    yaw_motor->SendInstruction(DmMotorInstructions::kDisable);
    DMEnable_ = 1;
  }
}

void Motor::ShootEnable() {
  // gimbal_controller.Enable(true);
  shoot_controller.Enable(true);
  shoot_controller.Arm(true);
  shoot_enabled_ = true;
}

void Motor::ShootDisable() {
  gimbal_controller.Enable(true);
  shoot_controller.Enable(false);
  shoot_controller.Arm(false);
  shoot_enabled_ = false;
}

/*
@brief:电机控制量更新
*/

void Motor::DMInitControl() {
  rc_request_pitch = pitch_init;
  rc_request_yaw = yaw_init;
  yaw_feedforward->Update(rc_request_yaw);
  gimbal_controller.SetTarget(rc_request_yaw, rc_request_pitch,yaw_feedforward->GetYawSpeedFeedforward());
  pitch_vel_filter.Update(pitch_motor->vel());
  gimbal_controller.Update(yaw_motor_pos, yaw_motor->vel(), global.bc->pitch / 57.3f, pitch_motor->vel());
  if (yaw_motor_pos > rc_request_yaw + 0.03f || yaw_motor_pos < rc_request_yaw - 0.03f) {
    global.fsm.init_count_ = 0;
    global.fsm.inited_ = true;
  }
  yaw_target = rc_request_yaw;
  pitch_target = rc_request_pitch;
  current_vel = pitch_motor->vel();
  // vofa_plotter->AddVariable(current_vel);
  // vofa_plotter->Update();
  // std::string buf = vofa_plotter->buffer(); // 先获取缓冲区
  // uint16_t send_len = std::min(static_cast<uint16_t>(buf.size()), static_cast<uint16_t>(16)); // 取最小长度
  // HAL_UART_Transmit_DMA(&huart6,
  //                       ""),
  //                       send_len);
}
// void Motor::DMControl() {S
//
//   left_x = global.bc->rc->left_x();
//
//   rc_request_pitch += Map(global.bc->rc->right_y() + global.bc->rc->mouse_y() + global.bc->tcremote.data().mouse_y,
//   -660, 660, -0.11f, 0.11f); rc_request_pitch = Clamp(rc_request_pitch, -15.f, 22.f);
//
//   rc_request_yaw -= Map(global.bc->rc->left_x() + global.bc->rc->mouse_x() + global.bc->tcremote.data().mouse_x,
//   -660, 660, -0.18f, 0.18f); rc_request_yaw = Wrap(rc_request_yaw, -180.f, 180.f);
//
//   if (reset_yaw_flag == 0) {
//     reset_yaw = global.bc->yaw;
//     rc_request_yaw = 0.f;
//     reset_yaw_flag = 1;
//   }
//   if (reset_yaw_flag == 1) {
//     gimbal_controller.SetTarget((reset_yaw - rc_request_yaw) / 57.3f, rc_request_pitch / 57.3f);
//     gimbal_controller.Update(global.bc->yaw / 57.3f, yaw_motor->vel(), global.bc->pitch / 57.3f, pitch_motor->vel());
//   }
// }
void Motor::DMAutoControl() {
  if (reset_yaw_flag == 0) {
    reset_yaw = global.bc->yaw;
    rc_request_yaw = 0.f;
    reset_yaw_flag = 1;
  }
  pitch_vel_filter.Update(pitch_motor->vel());
  if (aimbot_comm->aimbot_state() == 0) {
    rc_request_pitch += Map(global.bc->rc->right_y() + global.bc->rc->mouse_y() + global.bc->tcremote.data().mouse_y,
                            -660, 660, -0.11f, 0.11f);
    rc_request_pitch = Clamp(rc_request_pitch, -15.f, 20.f);

    rc_request_yaw -= Map(global.bc->rc->left_x() + global.bc->rc->mouse_x() + global.bc->tcremote.data().mouse_x, -660,
                          660, -0.23f, 0.23f);
    rc_request_yaw = Wrap(rc_request_yaw, -180.f, 180.f);
    yaw_feedforward->Update((reset_yaw - rc_request_yaw) / 57.3f);
    gimbal_controller.SetTarget((reset_yaw - rc_request_yaw) / 57.3f, rc_request_pitch / 57.3f,yaw_feedforward->GetYawSpeedFeedforward());
    gimbal_controller.Update(global.bc->yaw / 57.3f, yaw_motor->vel(), global.bc->pitch / 57.3f, pitch_motor->vel());
  } else {
    rc_request_pitch = aimbot_comm->pitch();
    rc_request_pitch = Clamp(rc_request_pitch, -15.f / 57.3f, 22.f / 57.3f);
    rc_request_yaw = aimbot_comm->yaw();
    rc_request_yaw = Wrap(rc_request_yaw, -std::numbers::pi, std::numbers::pi);
    yaw_feedforward->Update((reset_yaw) / 57.3f);
    gimbal_controller.SetTarget((reset_yaw) / 57.3f, rc_request_pitch / 57.3f,yaw_feedforward->GetYawSpeedFeedforward());
    gimbal_controller.Update(global.bc->yaw / 57.3f, yaw_motor->vel(), global.bc->pitch / 57.3f, pitch_motor->vel());
  }
  yaw_target = (reset_yaw - rc_request_yaw) / 57.3f;
  pitch_target = rc_request_pitch;
}

void Motor::ShootControl() {
  shoot_controller.SetArmSpeed(70.f);
  if (global.bc->rc->dial() > 650 || global.bc->rc->mouse_button_left() == 1 ||
      global.bc->tcremote.data().mouse_button_left == 1) {
    shoot_controller.SetMode(Shoot2Fric::kFullAuto);
    shoot_frequency = -15.0f;
    shoot_controller.SetShootFrequency(shoot_frequency);
  } else {
    shoot_controller.SetMode(Shoot2Fric::kStop);
  }
  rpm_left = ammo_left->rpm();
  rpm_right = -ammo_right->rpm();
  shoot_controller.Fire();
  shoot_controller.Update(ammo_left->rpm() / 60, ammo_right->rpm() / 60, dial_motor->pos_rad(), dial_motor->rpm());
}

/*
@brief:发送电机信息
*/

void Motor::SendDMCommand() {
  status = yaw_motor->status();
  yaw_pos = yaw_motor->pos();
  yaw_pid_debug = gimbal_controller.output().yaw;
  pitch_pid_debug = gimbal_controller.output().pitch;
  gravity_compensation_ = 1.f * std::cos(global.bc->pitch / 57.3f);
  pitch_motor->SetMitCommand(0.f, 0.f, gravity_compensation_ + gimbal_controller.output().pitch, 0.f, 0.f);
  yaw_motor->SetMitCommand(0.f, 0.f, gimbal_controller.output().yaw, 0.f, 0.f);
  // pitch_motor->SetPosition(0.f,0.f, 0.f,0.f,0.f);
  // yaw_motor->SetMitCommand(0.f,0.f,0.f,0.f,0.f);
}

void Motor::SendDjiCommand() {
  o1 = static_cast<i16>(shoot_controller.output().fric_1);
  o2 = static_cast<i16>(shoot_controller.output().fric_2);
  o3 = static_cast<i16>(shoot_controller.output().loader);
  if (shoot_enabled_ == 0) {
    ammo_left->SetCurrent(0);
    ammo_right->SetCurrent(0);
    dial_motor->SetCurrent(0);
  } else {
    ammo_left->SetCurrent(o1);
    ammo_right->SetCurrent(o2);
    dial_motor->SetCurrent(o3);
  }

  debug = o3;

  DjiMotorBase::SendCommand(*can1);
  DjiMotorBase::SendCommand(*can2);
}

void Motor::Transit_initmode(InitFlag new_mode) {
  if (new_mode != init_mode) {
    reset_yaw_flag = 0;
    global.fsm.init_count_ = 0;
    global.bc->buzzer_controller.Play<modules::buzzer_melody::Success>();
  }
  // 替换现有状态
  init_mode = new_mode;
}
