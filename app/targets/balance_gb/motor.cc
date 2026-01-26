#include <librm.hpp>

#include "can.h"

#include "motor.hpp"
#include "global.hpp"
#include "pid_debug.h"

using namespace rm::modules;

rm::hal::Can *can1;
rm::hal::Can *can2;

f32 gravity_compensation_ = 0.f;
f32 pitch_pid_debug, yaw_pid_debug;
f32 yaw_pos;
i16 debug;
f32 rpm_left,rpm_right;
f32 left_x;
u8 status;


extern Debug pid_debug;

void Motor::MotorInit() {
  can1 = new rm::hal::Can{hcan1};
  can2 = new rm::hal::Can{hcan2};

  pitch_motor = new DmMotor<DmMotorControlMode::kMit>  //
      {*can1, {0x19, 0x09, 3.141593f, 30.f, 10.f, std::make_pair(0.0f, 500.0f), std::make_pair(0.0f, 5.0f)}};
  yaw_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>  //
      {*can2, {0x04, 0x03, 3.141593f, 30.f, 10.f, std::make_pair(0.0f, 500.0f), std::make_pair(0.0f, 5.0f)}};

  ammo_left =  new M3508{*can1, 7};
  ammo_right =  new M3508{*can1, 8};
  dial_motor = new M3508{*can2, 1};

  can1->SetFilter(0, 0);
  can1->Begin();
  can2->SetFilter(0, 0);
  can2->Begin();
}

void Motor::CalcYawPos(f32 pos) {
  yaw_motor_pos = -yaw_motor->pos()-2.1f;
}
;


void Motor::MotorPidInit() {
  //初始pid参数
  // yaw电机pid
  gimbal_controller.pid().yaw_position.SetKp(-35.f).SetKi(0.f).SetKd(-300.f).SetMaxOut(200.f).SetMaxIout(0.f);
  gimbal_controller.pid().yaw_speed.SetKp(0.7f).SetKi(0.f).SetKd(0.01f).SetMaxOut(6.f).SetMaxIout(0.f);

  //pitch电机pid
  gimbal_controller.pid().pitch_position.SetKp(80.f).SetKi(0.f).SetKd(620.f).SetMaxOut(100.f).SetMaxIout(0.f);
  gimbal_controller.pid().pitch_speed.SetKp(0.6f).SetKi(0.01f).SetKd(1.f).SetMaxOut(6.f).SetMaxIout(0.8f);

  //摩擦轮电机
  shoot_controller.pid().fric_1_speed.SetKp(700.f).SetKi(0.f).SetKd(0.f).SetMaxOut(16384.f).SetMaxIout(0.f);

  shoot_controller.pid().fric_2_speed.SetKp(700.f).SetKi(0.f).SetKd(0.f).SetMaxOut(16384.f).SetMaxIout(0.f);

  //拨盘电机
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
    //gimbal_controller.Enable(true);
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
    gimbal_controller.SetTarget(rc_request_yaw , rc_request_pitch);
    gimbal_controller.Update(yaw_motor_pos, yaw_motor->vel(), global.bc->pitch/57.3f,pitch_motor->vel());
  if (yaw_motor_pos >-1.8f||yaw_motor_pos<-2.4f) {
    global.fsm.init_count_ =0;
  }

}
void Motor::DMControl() {

  left_x = global.bc->rc->left_x();

  rc_request_pitch += Map(global.bc->rc->right_y(), -660, 660, -0.11f, 0.11f);
  rc_request_pitch = Clamp(rc_request_pitch, -15.f , 22.f);

  rc_request_yaw -= Map(global.bc->rc->left_x() , -660, 660, -0.18f, 0.18f);
  rc_request_yaw = Wrap(rc_request_yaw, -180.f , 180.f);


  if (reset_yaw_flag == 0) {
    reset_yaw = global.bc->yaw;
    rc_request_yaw = 0.f;
    reset_yaw_flag = 1;
  }
  if (reset_yaw_flag == 1){
    gimbal_controller.SetTarget( (reset_yaw-rc_request_yaw)/57.3f, rc_request_pitch/57.3f);
    gimbal_controller.Update(global.bc->yaw/57.3f, yaw_motor->vel(), global.bc->pitch/57.3f, pitch_motor->vel());
  }

}

void Motor::ShootControl() {
  shoot_controller.SetArmSpeed(70.f);
  if (global.bc->rc->dial()>650) {
    shoot_controller.SetMode(Shoot2Fric::kFullAuto);
    shoot_frequency = -15.0f;
    shoot_controller.SetShootFrequency(shoot_frequency);
  }else {
    shoot_controller.SetMode(Shoot2Fric::kStop);
  }
  rpm_left = ammo_left->rpm();
  rpm_right = -ammo_right->rpm();
  shoot_controller.Fire();
  shoot_controller.Update(ammo_left->rpm()/60,ammo_right->rpm()/60,dial_motor->pos_rad(),dial_motor->rpm());

}

/*
@brief:发送电机信息
*/

void Motor::SendDMCommand() {
  status = yaw_motor->status();
  yaw_pos = yaw_motor->pos();
  yaw_pid_debug = gimbal_controller.output().yaw;
  pitch_pid_debug = gimbal_controller.output().pitch;
  gravity_compensation_ = 1.f * std::cos(global.bc->pitch/57.3f);
  pitch_motor->SetPosition(0.f,0.f, gravity_compensation_ +gimbal_controller.output().pitch,0.f,0.f);
  yaw_motor->SetPosition(0.f,0.f,gimbal_controller.output().yaw,0.f,0.f);
  //pitch_motor->SetPosition(0.f,0.f, 0.f,0.f,0.f);
  //yaw_motor->SetPosition(0.f,0.f,0.f,0.f,0.f);
}

void Motor::SendDjiCommand() {
  o1=static_cast<i16>(shoot_controller.output().fric_1);
  o2=static_cast<i16>(shoot_controller.output().fric_2);
  o3= static_cast<i16>(shoot_controller.output().loader);
  if (shoot_enabled_ == 0) {
    ammo_left->SetCurrent(0);
    ammo_right->SetCurrent(0);
    dial_motor->SetCurrent(0);
  }else {
    ammo_left->SetCurrent(o1);
    ammo_right->SetCurrent(o2);
    dial_motor->SetCurrent(o3);
  }

  debug = o3;

  rm::device::DjiMotor<>::SendCommand(*can1);
  rm::device::DjiMotor<>::SendCommand(*can2);

}
