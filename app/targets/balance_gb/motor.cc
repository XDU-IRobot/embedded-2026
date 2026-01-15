#include <librm.hpp>

#include "can.h"

#include "motor.hpp"
#include "global.hpp"

using namespace rm::modules;

void Motor::MotorInit() {
  can1 = new rm::hal::Can{hcan1};
  can2 = new rm::hal::Can{hcan2};

  pitch_motor = new DmMotor<DmMotorControlMode::kMit>  //
      {*can2, {0x00, 0x01, 3.141593f, 45.f, 54.f, std::make_pair(0.0f, 500.0f), std::make_pair(0.0f, 5.0f)}};
  yaw_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>  //
      {*can2, {0x02, 0x03, 3.141593f, 45.f, 54.f, std::make_pair(0.0f, 500.0f), std::make_pair(0.0f, 5.0f)}};

  ammo_left =  new M3508{*can1, 1};
  ammo_right =  new M3508{*can1, 2};
  dial_motor = new M2006{*can1, 3};

  can1->SetFilter(0, 0);
  can1->Begin();
  can2->SetFilter(0, 0);
  can2->Begin();
}

void Motor::MotorPidInit() {
  //初始pid参数
  // yaw电机pid
  gimbal_controller.pid().yaw_position.SetKp(0.f);
  gimbal_controller.pid().yaw_position.SetKi(0.f);
  gimbal_controller.pid().yaw_position.SetKd(0.f);
  gimbal_controller.pid().yaw_position.SetMaxOut(0.f);
  gimbal_controller.pid().yaw_position.SetMaxIout(0.f);
  gimbal_controller.pid().yaw_speed.SetKp(0.f);
  gimbal_controller.pid().yaw_speed.SetKi(0.f);
  gimbal_controller.pid().yaw_speed.SetKd(0.f);
  gimbal_controller.pid().yaw_speed.SetMaxOut(0.f);
  gimbal_controller.pid().yaw_speed.SetMaxIout(0.f);

  //pitch电机pid
  gimbal_controller.pid().pitch_position.SetKp(0.f);
  gimbal_controller.pid().pitch_position.SetKi(0.f);
  gimbal_controller.pid().pitch_position.SetKd(0.f);
  gimbal_controller.pid().pitch_position.SetMaxOut(0.f);
  gimbal_controller.pid().pitch_position.SetMaxIout(0.f);
  gimbal_controller.pid().pitch_speed.SetKp(0.f);
  gimbal_controller.pid().pitch_speed.SetKi(0.f);
  gimbal_controller.pid().pitch_speed.SetKd(0.f);
  gimbal_controller.pid().pitch_speed.SetMaxOut(0.f);
  gimbal_controller.pid().pitch_speed.SetMaxIout(0.f);

  //摩擦轮电机
  shoot_controller.pid().fric_1_speed.SetKp(0.f);
  shoot_controller.pid().fric_1_speed.SetKi(0.f);
  shoot_controller.pid().fric_1_speed.SetKd(0.f);
  shoot_controller.pid().fric_1_speed.SetMaxOut(16384.f);
  shoot_controller.pid().fric_1_speed.SetMaxIout(0.f);

  shoot_controller.pid().fric_2_speed.SetKp(0.f);
  shoot_controller.pid().fric_2_speed.SetKi(0.f);
  shoot_controller.pid().fric_2_speed.SetKd(0.f);
  shoot_controller.pid().fric_2_speed.SetMaxOut(16384.f);
  shoot_controller.pid().fric_2_speed.SetMaxIout(0.f);

  //拨盘电机
  shoot_controller.pid().loader_position.SetKp(0.f);
  shoot_controller.pid().loader_position.SetKi(0.f);
  shoot_controller.pid().loader_position.SetKd(0.f);
  shoot_controller.pid().loader_position.SetMaxOut(0.f);
  shoot_controller.pid().loader_position.SetMaxIout(0.f);
  shoot_controller.pid().loader_speed.SetKp(0.f);
  shoot_controller.pid().loader_speed.SetKi(0.f);
  shoot_controller.pid().loader_speed.SetKd(0.f);
  shoot_controller.pid().loader_speed.SetMaxOut(0.f);
  shoot_controller.pid().loader_speed.SetMaxIout(0.f);
}

void Motor::DMEnable() {
    if (DMEnable_ == 1) {
      pitch_motor->SendInstruction(DmMotorInstructions::kDisable);  // 使达妙电机失能
      yaw_motor->SendInstruction(DmMotorInstructions::kDisable);
      DMEnable_ = 0;
    }
}

void Motor::DMDisable() {
  if (DMEnable_ == 0) {
    pitch_motor->SendInstruction(DmMotorInstructions::kDisable);  // 使达妙电机失能
    yaw_motor->SendInstruction(DmMotorInstructions::kDisable);
    DMEnable_ = 1;
  }
}

void Motor::ShootEnable(bool enable) {
  shoot_enabled_ = enable;
  if (!shoot_enabled_) {
    gimbal_controller.Enable(false);
    shoot_controller.Enable(false);
    shoot_controller.Arm(false);
  }else {
    gimbal_controller.Enable(true);
    shoot_controller.Enable(true);
    shoot_controller.Arm(true);
  }
}

void Motor::DMInit() {
  init_count++;
  if (init_count <100) {
    gimbal_controller.SetTarget(global.bc->yaw , pitch_init*57.3f);
    gimbal_controller.Update(global.bc->yaw , yaw_motor->vel(), global.bc->pitch,pitch_motor->vel());
  }else {
    gimbal_controller.SetTarget(yaw_init*57.3f , pitch_init*57.3f);
    gimbal_controller.Update(global.bc->yaw , yaw_motor->vel(), global.bc->pitch,pitch_motor->vel());
  }
}
void Motor::DMControl() {
  rc_request_pitch += Map(global.bc->rc->left_y(), -660, 660, -0.2f, 0.2f);
  rc_request_pitch = Clamp(rc_request_pitch, -12.f , 18.f);

  rc_request_yaw -= Map(global.bc->rc->left_x() , -660, 660, -0.1f, 0.1f);
  rc_request_yaw = Wrap(rc_request_yaw, 0.f , 360.f);

  if (reset_yaw_flag == 0) {
    rc_request_yaw = 0.f;
    reset_yaw_flag = 1;
    reset_yaw = global.bc->yaw;
  }else {
    gimbal_controller.SetTarget(reset_yaw+rc_request_yaw, rc_request_pitch);
    gimbal_controller.Update(global.bc->yaw, yaw_motor->vel(), global.bc->pitch, pitch_motor->vel());
  }

  pitch_motor->SetPosition(0.f,0.f,gimbal_controller.output().pitch,0.f,0.f);
  yaw_motor->SetPosition(0.f,0.f,gimbal_controller.output().yaw,0.f,0.f);
}

void Motor::ShootControl() {

}
