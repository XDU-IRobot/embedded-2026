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
i16 rpm_left_, rpm_right_;
u16 heat;
u8 status, state_debug;
f32 yaw_target, pitch_target;
f32 auto_yaw_debug, auto_pitch_debug;
f32 dial_debug;
f32 pitch_tau;
bool last_aimbot_state = false;

extern VT03 tcremote;

void Motor::MotorInit() {
  can1 = new rm::hal::Can{hcan1};
  can2 = new rm::hal::Can{hcan2};

  pitch_motor = new DmMotor<DmMotorControlMode::kMit>  //
      {*can1, {0x12, 0x11, 3.141593f, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}}};
  yaw_motor = new DmMotor<DmMotorControlMode::kMit>  //
      {*can2, {0x04, 0x03, 3.141593f, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}}};

  aimbot_comm = new device::AimbotCanCommunicator(*can1);
  device_aimbot << aimbot_comm;
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

void Motor::CalcYawPos(f32 pos) { yaw_motor_pos = yaw_motor->pos() + 2.14f; };

void Motor::MotorPidInit() {
  // 初始pid参数
  bool aimbot_on = (aimbot_comm->aimbot_state() >> 0 & 0x01 || aimbot_comm->aimbot_state() >> 0 & 0x03) &&
                   global.bc->rc->mouse_button_right();
  if (aimbot_on == 1) {
    yaw_pos_kp = 20.f;
    yaw_pos_ki = 0.f;
    yaw_pos_kd = 200.f;
    yaw_vel_kp = 0.8f;
    yaw_vel_ki = 0.f;
    yaw_vel_kd = 0.f;
  } else {
    yaw_pos_kp = 20.f;
    yaw_pos_ki = 0.f;
    yaw_pos_kd = 400.f;
    yaw_vel_kp = 0.8f;
    yaw_vel_ki = 0.f;
    yaw_vel_kd = 0.f;
  }
  //  yaw电机pid
  gimbal_controller.pid()
      .yaw_position.SetKp(yaw_pos_kp)
      .SetKi(yaw_pos_ki)
      .SetKd(yaw_pos_kd)
      .SetMaxOut(10.f)
      .SetMaxIout(0.f);
  gimbal_controller.pid()
      .yaw_speed.SetKp(yaw_vel_kp)
      .SetKi(yaw_vel_ki)
      .SetKd(yaw_vel_kd)
      .SetMaxOut(6.f)
      .SetMaxIout(0.4f);

  // pitch电机pid
  gimbal_controller.pid().pitch_position.SetKp(26.f).SetKi(0.f).SetKd(400.f).SetMaxOut(10.f).SetMaxIout(0.9f);
  gimbal_controller.pid().pitch_speed.SetKp(0.5f).SetKi(0.f).SetKd(0.f).SetMaxOut(6.f).SetMaxIout(0.2f);

  // 摩擦轮电机
  shoot_controller.pid().fric_1_speed.SetKp(19.f).SetKi(0.f).SetKd(1.f).SetMaxOut(16000.f).SetMaxIout(0.f);
  shoot_controller.pid().fric_2_speed.SetKp(19.f).SetKi(0.f).SetKd(1.f).SetMaxOut(16000.f).SetMaxIout(0.f);

  // 拨盘电机
  shoot_controller.pid().loader_position.SetKp(0.f).SetKi(0.f).SetKd(0.f).SetMaxOut(1200.f).SetMaxIout(0.f);
  shoot_controller.pid().loader_speed.SetKp(-8.f).SetKi(0.f).SetKd(0.f).SetMaxOut(16000.f).SetMaxIout(0.f);
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
@brief:子弹弹丸计数并热量更新
*/
void Motor::ShooterCounter() {
  if (ammo_right->rpm() < 6200 && ammo_right->rpm() > 5800) {
    fric_on_flag_ = true;
  } else {
    fric_on_flag_ = false;
  }
  if (fric_on_flag_) {
    if (last_right_rpm - ammo_left->rpm() > 60 && shoot_one_flag_ == 0) {  // 不再处在下降沿再计
      fric_reduce_flag_ = 1;
    }
    if (last_right_rpm - ammo_left->rpm() < -30) {
      shoot_one_flag_ = 0;
      fric_reduce_flag_ = 0;
    }
    if (fric_reduce_flag_ == 1) {
      shoot_number++;
      fric_reduce_flag_ = 0;
      shoot_one_flag_ = 1;
      heat_ultimate += 10;
    }
    last_right_rpm = ammo_left->rpm();
  }
}

void Motor::HeatUpdate() {
  heat_ultimate = Deadline(heat_ultimate - global.chassis_rx->chassis_data_rx.CoolingSpeed / 500.f, 0,
                           global.chassis_rx->chassis_data_rx.HeatLimit);
}

/*
@brief:电机控制量更新
*/

void Motor::DMInitControl() {
  rc_request_pitch = pitch_init;
  rc_request_yaw = yaw_init ;
  yaw_feedforward->Update(rc_request_yaw);
  gimbal_controller.SetTarget(rc_request_yaw, rc_request_pitch, yaw_feedforward->GetYawSpeedFeedforward());
  gimbal_controller.Update(yaw_motor_pos, global.bc->hipnuc_imu->gyro_z(), global.bc->pitch / 57.3f, -global.bc->hipnuc_imu->gyro_x());
  if (yaw_motor_pos > rc_request_yaw + 0.03f || yaw_motor_pos < rc_request_yaw - 0.03f) {
    global.fsm.init_count_ = 0;
    global.fsm.inited_ = true;
  }
  yaw_target = rc_request_yaw;
  pitch_target = rc_request_pitch;
}

void Motor::DMAutoControl() {
  if (reset_yaw_flag == 0) {
    reset_yaw = global.bc->yaw;
    rc_request_yaw = 0.f;
    reset_yaw_flag = 1;
  }

  bool aimbot_on = (aimbot_comm->aimbot_state() >> 0 & 0x01 || aimbot_comm->aimbot_state() >> 0 & 0x03) &&
                   global.bc->rc->mouse_button_right();
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
    rc_request_pitch += Map(global.bc->rc->right_y() + global.bc->rc->mouse_y() + tcremote.data().mouse_y,
                            -660, 660, -0.11f, 0.11f);

    rc_request_pitch = Clamp(rc_request_pitch, -15.f, 20.f);

    rc_request_yaw += Map(global.bc->rc->left_x() + global.bc->rc->mouse_x() + tcremote.data().mouse_x, -660,
                          660, -0.3f, 0.3f);

    rc_request_yaw = Wrap(rc_request_yaw, -180.f, 180.f);

    yaw_feedforward->Update((reset_yaw - rc_request_yaw) / 57.3f);

    gimbal_controller.SetTarget((reset_yaw - rc_request_yaw) / 57.3f, rc_request_pitch / 57.3f,
                                yaw_feedforward->GetYawSpeedFeedforward());
  }

  gimbal_controller.Update(global.bc->yaw / 57.3f, global.bc->hipnuc_imu->gyro_z(), global.bc->pitch / 57.3f, -global.bc->hipnuc_imu->gyro_x());

  yaw_target = (reset_yaw - rc_request_yaw) / 57.3f;
  pitch_target = rc_request_pitch;
}

void Motor::ShootNormalControl() {
  global.motor->dail_encoder_counter.Update(global.motor->dial_motor->encoder());
  shoot_controller.SetLeftArmSpeed(7000.f);
  shoot_controller.SetRightArmSpeed(7000.f);
  if (global.bc->rc->right_x() < -650 || global.bc->rc->mouse_button_left() == 1 ||
      tcremote.data().mouse_button_left == 1) {
    shoot_controller.SetMode(Shoot3Fric::kFullAuto);
    shoot_frequency = -14.0f;
    shoot_controller.SetShootFrequency(shoot_frequency);
  } else {
    shoot_controller.SetMode(Shoot3Fric::kStop);
  }
  global.motor->shoot_controller.Fire();
  global.motor->shoot_controller.Update(global.motor->ammo_left->rpm(), global.motor->ammo_right->rpm(), 0,
                                        static_cast<f32>(global.motor->dail_encoder_counter.revolutions()) * 8191.0f +
                                            static_cast<f32>(global.motor->dail_encoder_counter.last_ecd()),
                                        global.motor->dial_motor->rpm());
}

void Motor::ShootAutoControl() {
  global.motor->dail_encoder_counter.Update(global.motor->dial_motor->encoder());
  shoot_controller.SetLeftArmSpeed(7000.f);
  shoot_controller.SetRightArmSpeed(7000.f);
  device_aimbot.Update();
  ShooterCounter();
  HeatUpdate();

  if (!device_aimbot.all_device_ok()) {
    if (global.bc->rc->right_x() < -650 || global.bc->rc->mouse_button_left() == 1 ||
        tcremote.data().mouse_button_left == 1) {
      shoot_controller.SetMode(Shoot3Fric::kFullAuto);
      shoot_frequency = -14.0f;
      shoot_controller.SetShootFrequency(shoot_frequency);
    } else {
      shoot_controller.SetMode(Shoot3Fric::kStop);
    }
  } else {
    heat = global.chassis_rx->chassis_data_rx.HeatLimit - heat_ultimate;
    if (global.bc->rc->right_x() < -650 ||
        (global.bc->rc->mouse_button_left() == 1 && global.bc->rc->mouse_button_right() == 1 &&
         aimbot_comm->aimbot_state() >> 0 & 0x03 &&
         global.chassis_rx->chassis_data_rx.HeatLimit - heat_ultimate > 40) ||
        tcremote.data().mouse_button_left == 1 ||
        (global.bc->rc->mouse_button_left() == 1 &&
         global.chassis_rx->chassis_data_rx.HeatLimit - heat_ultimate > 40)) {
      shoot_controller.SetMode(Shoot3Fric::kFullAuto);
      shoot_frequency = -14.0f;
      shoot_controller.SetShootFrequency(shoot_frequency);
    } else {
      shoot_controller.SetMode(Shoot3Fric::kStop);
    }
  }

  global.motor->shoot_controller.Fire();
  global.motor->shoot_controller.Update(global.motor->ammo_left->rpm(), global.motor->ammo_right->rpm(), 0,
                                        static_cast<f32>(global.motor->dail_encoder_counter.revolutions()) * 8191.0f +
                                            static_cast<f32>(global.motor->dail_encoder_counter.last_ecd()),
                                        global.motor->dial_motor->rpm());
}

/*
@brief:发送电机信息
*/

void Motor::SendDMCommand() {
  status = pitch_motor->status();
  yaw_pos = yaw_motor->pos() ;
  yaw_pid_debug = gimbal_controller.output().yaw;
  global.motor->gravity_compensation_ = 1.1f * std::cos(global.bc->pitch / 57.3f + 0.29f);
  gravity_compensation = global.motor->gravity_compensation_;
  pitch_motor->SetMitCommand(0.f, 0.f, gravity_compensation + gimbal_controller.output().pitch, 0.f, 0.f);
  //pitch_motor->SetMitCommand(0.f, 0.f, 0.f, 0.f, 0.f);
  pitch_pid_debug = pitch_motor->tau();
  pitch_speed = pitch_motor->vel();
  yaw_motor->SetMitCommand(0.f, 0.f, gimbal_controller.output().yaw, 0.f, 0.f);
  //yaw_motor->SetMitCommand(0.f, 0.f, 0.f, 0.f, 0.f);
  pitch_tau = gravity_compensation + gimbal_controller.output().pitch;
  // pitch_motor->SetMitCommand(0.f,0.f, 0.f,0.f,0.f);
  // yaw_motor->SetMitCommand(0.f,0.f,0.f,0.f,0.f);
}

void Motor::SendDjiCommand() {

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
  dial_debug = dial_motor->current();

  state_debug = aimbot_comm->aimbot_state();

  DjiMotorBase::SendCommand(*can1);
  DjiMotorBase::SendCommand(*can2);
}

void Motor::Transit_initmode(bool keyboard_e) {
  // static bool last_keyboard_e = 0;  // 上一帧状态
  //
  // // 上升沿检测：从0变1
  // if (keyboard_e == 1 && last_keyboard_e == 0) {
  //   change_yaw_init_flag = true;
  //   yaw_init -= M_PI;
  // }
  //
  // last_keyboard_e = keyboard_e;  // 更新状态
  //
  // const float EPS = 1e-6f;
  // if (fabs(yaw_init + 2 * M_PI) < EPS) {
  //   yaw_init += 2 * M_PI;
  // }
}
