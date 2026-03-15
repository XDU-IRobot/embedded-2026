#include <librm.hpp>

#include "can.h"

#include "motor.hpp"
#include "global.hpp"
#include "firstorderfilter.hpp"
#include "usart.h"
using namespace rm::modules;

rm::hal::Can *can1;
rm::hal::Can *can2;

f32 gravity_compensation = 0.f, yaw_compensation = 0.f;
f32 pitch_pid_debug, yaw_pid_debug;
f32 yaw_pos, pitch_speed;
i16 debug;
i16 rpm_left_, rpm_right_;
u8 status, state_debug;
f32 yaw_target, pitch_target;
static float current_vel = 0.f;
f32 auto_yaw_debug, auto_pitch_debug;
f32 dial_debug;
f32 pitch_speed_target;
int time_debug;
bool flag;
f32 dial_pos, dial_target;
bool last_aimbot_state = false;
f32 ammo_left_current,ammo_right_currednt;
// extern Debug pid_debug;

void Motor::MotorInit() {
  can1 = new rm::hal::Can{hcan1};
  can2 = new rm::hal::Can{hcan2};

  pitch_motor = new DmMotor<DmMotorControlMode::kMit>  //
      {*can1, {0x12, 0x11, 3.141593f, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}}};
  yaw_motor = new DmMotor<DmMotorControlMode::kMit>  //
      {*can2, {0x04, 0x03, 3.141593f, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}}};

  aimbot_comm = new device::AimbotCanCommunicator(*can1);
  ammo_left = new M3508{*can1, 7};
  ammo_right = new M3508{*can1, 8};
  dial_motor = new M3508{*can2, 1};

  yaw_feedforward = new YawSpeedFeedforward(0.002, 1);

  // sweep_controller = new SineSweep(0.f, 2.f,);

  can1->SetFilter(0, 0);
  can1->Begin();
  can2->SetFilter(0, 0);
  can2->Begin();
}

void Motor::CalcYawPos(f32 pos) { yaw_motor_pos = yaw_motor->pos() - 0.85f; };

void Motor::MotorPidInit() {
  // 初始pid参数
  //  yaw电机pid
  gimbal_controller.pid().yaw_position.SetKp(20.f).SetKi(0.f).SetKd(400.f).SetMaxOut(10.f).SetMaxIout(0.f);
  gimbal_controller.pid().yaw_speed.SetKp(0.8f).SetKi(0.0f).SetKd(0.f).SetMaxOut(6.f).SetMaxIout(0.4f);

  // pitch电机pid
  gimbal_controller.pid().pitch_position.SetKp(30.f).SetKi(0.f).SetKd(300.f).SetMaxOut(10.f).SetMaxIout(0.9f);
  gimbal_controller.pid().pitch_speed.SetKp(0.6f).SetKi(0.f).SetKd(1.5f).SetMaxOut(8.f).SetMaxIout(0.2f);

  // 摩擦轮电机
  shoot_controller.pid().fric_1_speed.SetKp(20.f).SetKi(0.f).SetKd(0.f).SetMaxOut(12000.f).SetMaxIout(0.f);
  shoot_controller.pid().fric_2_speed.SetKp(20.f).SetKi(0.f).SetKd(0.f).SetMaxOut(12000.f).SetMaxIout(0.f);

  // 拨盘电机
  shoot_controller.pid().loader_position.SetKp(0.f).SetKi(0.f).SetKd(0.f).SetMaxOut(1200.f).SetMaxIout(0.f);
  shoot_controller.pid().loader_speed.SetKp(-4.f).SetKi(0.f).SetKd(0.f).SetMaxOut(10000.f).SetMaxIout(0.f);
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
  shoot_controller.Enable(true);
  shoot_controller.Arm(true);
  shoot_enabled_ = true;
}

void Motor::ShootDisable() {
  gimbal_controller.Enable(true);
  shoot_controller.Enable(false);
  shoot_controller.Arm(false);
  shoot_enabled_ = false;
  single_shoot_temp = 0;
}

/*
@brief:电机控制量更新
*/

void Motor::DMInitControl() {
  rc_request_pitch = pitch_init;
  rc_request_yaw = yaw_init;
  yaw_feedforward->Update(rc_request_yaw);
  gimbal_controller.SetTarget(rc_request_yaw, rc_request_pitch, yaw_feedforward->GetYawSpeedFeedforward());
  gimbal_controller.Update(yaw_motor_pos, yaw_motor->vel(), global.bc->pitch / 57.3f, pitch_motor->vel());
  if (yaw_motor_pos > rc_request_yaw + 0.03f || yaw_motor_pos < rc_request_yaw - 0.03f) {
    global.fsm.init_count_ = 0;
    global.fsm.inited_ = true;
  }
  yaw_target = rc_request_yaw;
  pitch_target = rc_request_pitch;
  current_vel = pitch_motor->vel();
}
// void Motor::DMControl() {
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

  bool aimbot_on = aimbot_comm->aimbot_state() >> 0 & 0x01;

  // ===== 检测自瞄 -> 手动切换 =====
  if (!aimbot_on && last_aimbot_state) {
    // 用当前云台角度初始化手动模式
    rc_request_pitch = global.bc->pitch;
    rc_request_yaw = reset_yaw - global.bc->yaw;
  }

  last_aimbot_state = aimbot_on;

  if (aimbot_on) {
    auto_yaw_debug = aimbot_comm->yaw() * 57.3f;
    auto_pitch_debug = aimbot_comm->pitch() * 57.3f;

    rc_request_pitch = aimbot_comm->pitch();
    rc_request_pitch = Clamp(rc_request_pitch, -15.f / 57.3f, 22.f / 57.3f);

    rc_request_yaw = aimbot_comm->yaw();
    rc_request_yaw = Wrap(rc_request_yaw, -std::numbers::pi, std::numbers::pi);

    yaw_feedforward->Update(rc_request_yaw);

    gimbal_controller.SetTarget(rc_request_yaw, rc_request_pitch, yaw_feedforward->GetYawSpeedFeedforward());

  } else {
    rc_request_pitch += Map(global.bc->rc->right_y() + global.bc->rc->mouse_y() + global.bc->tcremote.data().mouse_y,
                            -660, 660, -0.11f, 0.11f);

    rc_request_pitch = Clamp(rc_request_pitch, -15.f, 20.f);

    rc_request_yaw += Map(global.bc->rc->left_x() + global.bc->rc->mouse_x() + global.bc->tcremote.data().mouse_x, -660,
                          660, -0.3f, 0.3f);

    rc_request_yaw = Wrap(rc_request_yaw, -180.f, 180.f);

    yaw_feedforward->Update((reset_yaw - rc_request_yaw) / 57.3f);

    gimbal_controller.SetTarget((reset_yaw - rc_request_yaw) / 57.3f, rc_request_pitch / 57.3f,
                                yaw_feedforward->GetYawSpeedFeedforward());
  }

  gimbal_controller.Update(global.bc->yaw / 57.3f, yaw_motor->vel(), global.bc->pitch / 57.3f, pitch_motor->vel());

  yaw_target = (reset_yaw - rc_request_yaw) / 57.3f;
  pitch_target = rc_request_pitch;
}

void Motor::ShootControl() {
  global.motor->dail_encoder_counter.Update(global.motor->dial_motor->encoder());
  shoot_controller.SetLeftArmSpeed(6000.f);
  shoot_controller.SetRightArmSpeed(6100.f);
  if (global.bc->rc->dial() >= 650||global.bc->rc->mouse_button_left() == 1
      // && heat_limit_ - heat_current_ > 100
  ) {
    if (!single_shoot_flag_) {
      global.motor->shoot_controller.SetMode(Shoot3Fric::kSingleShot);
      single_shoot_flag_ = true;
    } else {
      global.motor->shoot_controller.SetMode(Shoot3Fric::kStop);
    }
  } else {
    global.motor->shoot_controller.SetMode(Shoot3Fric::kStop);
    single_shoot_flag_ = false;
  }
  // if (global.bc->rc->dial() < -650 || global.bc->rc->mouse_button_left() == 1 ||
  //     global.bc->tcremote.data().mouse_button_left == 1) {
  //   shoot_controller.SetMode(Shoot3Fric::kFullAuto);
  //   shoot_frequency = -25.0f;
  //   shoot_controller.SetShootFrequency(shoot_frequency);
  //     } else {
  //       shoot_controller.SetMode(Shoot3Fric::kStop);
  //     }
  global.motor->shoot_controller.Fire();
  global.motor->shoot_controller.Update(global.motor->ammo_left->rpm(), global.motor->ammo_right->rpm(), 0,
                                        static_cast<f32>(global.motor->dail_encoder_counter.revolutions()) * 8191.0f +
                                            static_cast<f32>(global.motor->dail_encoder_counter.last_ecd()),
                                        global.motor->dial_motor->rpm());
  time_debug = single_shoot_temp;


  flag = single_flag;
  dial_pos = static_cast<f32>(global.motor->dail_encoder_counter.revolutions()) * 8191.0f +
             static_cast<f32>(global.motor->dail_encoder_counter.last_ecd());
  dial_target = shoot_controller.target().loader_position;
  ammo_left_current = ammo_left->current();
  ammo_right_currednt = ammo_right->current();
}
// if (global.bc->rc->dial() > 650 || global.bc->rc->mouse_button_left() == 1 ||
//     global.bc->tcremote.data().mouse_button_left == 1) {
//   shoot_controller.SetMode(Shoot2Fric::kFullAuto);
//   shoot_frequency = -8.0f;
//   shoot_controller.SetShootFrequency(shoot_frequency);
//     } else {
//       shoot_controller.SetMode(Shoot2Fric::kStop);
//     }

/*
@brief:发送电机信息
*/

void Motor::SendDMCommand() {
  status = yaw_motor->status();
  yaw_pos = yaw_motor->pos() * 57.3;
  yaw_pid_debug = gimbal_controller.output().yaw;
  global.motor->gravity_compensation_ = 1.2f * std::cos(global.bc->pitch / 57.3f + 0.29f);
  gravity_compensation = global.motor->gravity_compensation_;
  pitch_motor->SetMitCommand(0.f, 0.f, gravity_compensation + gimbal_controller.output().pitch, 0.f, 0.f);
  pitch_speed_target = gimbal_controller.state().pitch_speed_target;
  pitch_pid_debug = pitch_motor->tau();
  pitch_speed = pitch_motor->vel();
  yaw_motor->SetMitCommand(0.f, 0.f, global.motor->yaw_compensation_ + gimbal_controller.output().yaw, 0.f, 0.f);
  // pitch_motor->SetMitCommand(0.f,0.f, 0.f,0.f,0.f);
  // yaw_motor->SetMitCommand(0.f,0.f,0.f,0.f,0.f);
}

void Motor::SendDjiCommand() {
  dial_debug = o3;
  rpm_left_ = ammo_left->rpm();
  rpm_right_ = -ammo_right->rpm();
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
    dial_motor->SetCurrent(-o3);
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
