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
u8 RcKey_KC_flag = 0, TCRcKey_KC_flag = 0;
u8 RcKey_KV_flag = 0, TCRcKey_KV_flag = 0;
int Rc_kfric_speed;
f32 yaw_target, pitch_target;
f32 auto_yaw_debug, auto_pitch_debug;
f32 dial_debug;
f32 pitch_tau;
bool last_aimbot_state = false;
bool button;
int s_number;
u16 h_ultimate_ = 0;
f32 pitch_pos_ki_;
int shoot_cycle_counter;
int a = 0;

extern VT03 tcremote;

void Motor::MotorInit() {
  can1 = new rm::hal::Can{hcan1};
  can2 = new rm::hal::Can{hcan2};

  pitch_motor = new DmMotor<DmMotorControlMode::kMit>  //
      {*can2, {0x05, 0x04, 3.141593f, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}}};
  yaw_motor = new DmMotor<DmMotorControlMode::kMit>  //
      {*can1, {0x10, 0x09, 3.141593f, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}}};

  aimbot_comm = new device::AimbotCanCommunicator(*can1);
  device_aimbot << aimbot_comm;
  ammo_left = new M3508{*can2, 2};
  ammo_right = new M3508{*can2, 4};
  dial_motor = new M3508{*can1, 3};

  yaw_feedforward = new YawSpeedFeedforward(0.002, 1);

  // sweep_controller = new SineSweep(0.f, 2.f,);

  can1->SetFilter(0, 0);
  can1->Begin();
  can2->SetFilter(0, 0);
  can2->Begin();
}

void Motor::CalcYawPos(f32 pos) { yaw_motor_pos = yaw_motor->pos(); };

void Motor::MotorPidInit() {
  // 初始pid参数
  bool aimbot_on = (aimbot_comm->aimbot_state() >> 0 & 0x01 || (aimbot_comm->aimbot_state() >> 1 & 0x01));
  if (aimbot_on == 1) {
    yaw_pos_kp = 18.f;
    yaw_pos_ki = 0.3f;
    yaw_pos_kd = 100.f;
    yaw_vel_kp = 0.6f;
    yaw_vel_ki = 0.f;
    yaw_vel_kd = 0.f;
    pitch_pos_kp = 30.f;
    pitch_pos_ki = 0.5f;
    pitch_pos_kd = 100.f;
    pitch_vel_kp = 0.6f;
    pitch_vel_ki = 0.f;
    pitch_vel_kd = 0.f;
  } else {
    yaw_pos_kp = 15.f;
    yaw_pos_ki = 0.f;
    yaw_pos_kd = 300.f;
    yaw_vel_kp = 0.6f;
    yaw_vel_ki = 0.f;
    yaw_vel_kd = 0.f;
    pitch_pos_kp = 26.f;
    pitch_pos_ki = 0.f;
    pitch_pos_kd = 300.f;
    pitch_vel_kp = 0.5f;
    pitch_vel_ki = 0.f;
    pitch_vel_kd = 0.f;
  }
  pitch_pos_ki_ = pitch_pos_ki;
  //  yaw电机pid
  gimbal_controller.pid()
      .yaw_position.SetKp(yaw_pos_kp)
      .SetKi(yaw_pos_ki)
      .SetKd(yaw_pos_kd)
      .SetMaxOut(10.f)
      .SetMaxIout(1.f);
  gimbal_controller.pid()
      .yaw_speed.SetKp(yaw_vel_kp)
      .SetKi(yaw_vel_ki)
      .SetKd(yaw_vel_kd)
      .SetMaxOut(6.f)
      .SetMaxIout(0.4f);

  // pitch电机pid
  gimbal_controller.pid()
      .pitch_position.SetKp(pitch_pos_kp)
      .SetKi(pitch_pos_ki)
      .SetKd(pitch_pos_kd)
      .SetMaxOut(10.f)
      .SetMaxIout(1.2f);
  gimbal_controller.pid()
      .pitch_speed.SetKp(pitch_vel_kp)
      .SetKi(pitch_vel_ki)
      .SetKd(pitch_vel_kd)
      .SetMaxOut(6.f)
      .SetMaxIout(0.2f);

  // 摩擦轮电机
  shoot_controller.pid().fric_1_speed.SetKp(19.f).SetKi(0.f).SetKd(1.f).SetMaxOut(13000.f).SetMaxIout(0.f);
  shoot_controller.pid().fric_2_speed.SetKp(19.f).SetKi(0.f).SetKd(1.f).SetMaxOut(13000.f).SetMaxIout(0.f);

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
  if (-ammo_right->rpm() < right_set_speed + 200 && -ammo_right->rpm() > right_set_speed - 200) {
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
      heat_ultimate_ += 10;
    }
    last_right_rpm = ammo_left->rpm();
  }
  s_number = shoot_number;
}

void Motor::HeatUpdate() {
  heat_ultimate_ = Clamp(heat_ultimate - global.chassis_rx->chassis_data_rx.CoolingSpeed / 500.f, 0,
                         global.chassis_rx->chassis_data_rx.HeatLimit);
  h_ultimate_ = heat_ultimate_;
}

/*
@brief:电机控制量更新
*/

void Motor::DMInitControl() {
  rc_request_pitch = pitch_init;
  rc_request_yaw = yaw_init;
  yaw_feedforward->Update(rc_request_yaw);
  gimbal_controller.SetTarget(rc_request_yaw / 57.3f, rc_request_pitch / 57.3f,
                              yaw_feedforward->GetYawSpeedFeedforward());
  gimbal_controller.Update(yaw_motor_pos, global.bc->hipnuc_imu->gyro_z(), global.bc->pitch / 57.3f,
                           -global.bc->hipnuc_imu->gyro_x());
  if (yaw_motor_pos > rc_request_yaw + 0.1f || yaw_motor_pos < rc_request_yaw - 0.1f) {
    global.fsm.init_count_ = 0;
    global.fsm.inited_ = true;
  }
  yaw_target = rc_request_yaw;
  pitch_target = rc_request_pitch;
}

void Motor::DMAimControl() {
  if (reset_yaw_flag == 0) {
    reset_yaw = global.bc->yaw;
    rc_request_yaw = 0.f;
    reset_yaw_flag = 1;
  }

  bool aimbot_on = (aimbot_comm->aimbot_state() >> 0 & 0x01 || (aimbot_comm->aimbot_state() >> 1 & 0x01));
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
    rc_request_pitch +=
        Map(global.bc->rc->right_y() + global.bc->rc->mouse_y() + tcremote.data().mouse_y, -660, 660, -0.2f, 0.2f);

    rc_request_pitch = Clamp(rc_request_pitch, -18.f, 35.f);

    rc_request_yaw +=
        Map(global.bc->rc->left_x() + global.bc->rc->mouse_x() + tcremote.data().mouse_x, -660, 660, -0.38f, 0.38f);

    rc_request_yaw = Wrap(rc_request_yaw, -180.f, 180.f);

    yaw_feedforward->Update((reset_yaw - rc_request_yaw) / 57.3f);

    gimbal_controller.SetTarget((reset_yaw - rc_request_yaw) / 57.3f, rc_request_pitch / 57.3f,
                                yaw_feedforward->GetYawSpeedFeedforward());
  }

  gimbal_controller.Update(global.bc->yaw / 57.3f, global.bc->hipnuc_imu->gyro_z(), global.bc->pitch / 57.3f,
                           -global.bc->hipnuc_imu->gyro_x());

  yaw_target = (reset_yaw - rc_request_yaw) / 57.3f;
  pitch_target = rc_request_pitch;
}

void Motor::DMAutoControl() {
  if (reset_yaw_flag == 0) {
    reset_yaw = global.bc->yaw;
    rc_request_yaw = 0.f;
    reset_yaw_flag = 1;
  }
  button = tcremote.data().mouse_button_right;
  // bool aimbot_on = (aimbot_comm->aimbot_state() >> 0 & 0x01 || (aimbot_comm->aimbot_state() >> 1 & 0x01)) ;
  bool aimbot_on = (aimbot_comm->aimbot_state() >> 0 & 0x01 || (aimbot_comm->aimbot_state() >> 1 & 0x01)) &&
                   (global.bc->rc->mouse_button_right() == 1 || tcremote.data().mouse_button_right == 1);
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
    rc_request_pitch +=
        Map(global.bc->rc->right_y() + global.bc->rc->mouse_y() + tcremote.data().mouse_y, -660, 660, -0.2f, 0.2f);

    rc_request_pitch = Clamp(rc_request_pitch, -18.f, 35.f);

    rc_request_yaw +=
        Map(global.bc->rc->left_x() + global.bc->rc->mouse_x() + tcremote.data().mouse_x, -660, 660, -0.38f, 0.38f);

    rc_request_yaw = Wrap(rc_request_yaw, -180.f, 180.f);

    yaw_feedforward->Update((reset_yaw - rc_request_yaw) / 57.3f);

    gimbal_controller.SetTarget((reset_yaw - rc_request_yaw) / 57.3f, rc_request_pitch / 57.3f,
                                yaw_feedforward->GetYawSpeedFeedforward());
  }

  gimbal_controller.Update(global.bc->yaw / 57.3f, global.bc->hipnuc_imu->gyro_z(), global.bc->pitch / 57.3f,
                           -global.bc->hipnuc_imu->gyro_x());

  yaw_target = (reset_yaw - rc_request_yaw) / 57.3f;
  pitch_target = rc_request_pitch;
}

void Motor::FricSpeedUpdate() {
  // C键
  if (global.bc->rc->key(DR16::Key::kC) == 1) {  // 摩擦轮转速降低
    RcKey_KC_flag = 1;
  }
  if (global.bc->rc->key(DR16::Key::kC) == 0 && RcKey_KC_flag == 1) {
    Rc_kfric_speed--;
    RcKey_KC_flag = 0;
  }
  if (tcremote.data().keyboard_key >> 13 == 1) {  // 摩擦轮转速降低
    TCRcKey_KC_flag = 1;
  }
  if (tcremote.data().keyboard_key >> 13 == 0 && TCRcKey_KC_flag == 1) {
    Rc_kfric_speed--;
    TCRcKey_KC_flag = 0;
  }

  // V键
  if (global.bc->rc->key(DR16::Key::kV) == 1) {  // 摩擦轮转速降低
    RcKey_KV_flag = 1;
  }
  if (global.bc->rc->key(DR16::Key::kV) == 0 && RcKey_KV_flag == 1) {
    Rc_kfric_speed++;
    RcKey_KV_flag = 0;
  }
  if (tcremote.data().keyboard_key >> 14 == 1) {  // 摩擦轮转速降低
    TCRcKey_KV_flag = 1;
  }
  if (tcremote.data().keyboard_key >> 14 == 0 && TCRcKey_KV_flag == 1) {
    Rc_kfric_speed++;
    TCRcKey_KV_flag = 0;
  }
  left_set_speed = Clamp(6500 + 10 * Rc_kfric_speed, left_fric_speed_min, left_fric_speed_max);
  right_set_speed = Clamp(6500 + 10 * Rc_kfric_speed, right_fric_speed_min, right_fric_speed_max);
}

void Motor::ShootNormalControl() {
  FricSpeedUpdate();
  ShooterCounter();
  HeatUpdate();
  global.motor->dail_encoder_counter.Update(global.motor->dial_motor->encoder());
  shoot_controller.SetLeftArmSpeed(left_set_speed);
  shoot_controller.SetRightArmSpeed(right_set_speed);
  if ((aimbot_comm->aimbot_state() >> 1 & 0x01) || global.bc->rc->dial() < -600) {
    shoot_controller.SetMode(Shoot3Fric::kFullAuto);
    shoot_frequency = -24.0f;
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

void Motor::ShootAutoFuControl() {
  FricSpeedUpdate();
  global.motor->dail_encoder_counter.Update(global.motor->dial_motor->encoder());
  shoot_controller.SetLeftArmSpeed(left_set_speed);
  shoot_controller.SetRightArmSpeed(right_set_speed);

  // 读取开火信号（1=允许发射，0=停止）
  uint8_t current_state = (aimbot_comm->aimbot_state() >> 1 & 0x01);

  if (current_state == 1) {
    // 允许发射：检查间隔是否已满
    if (shoot_cycle_counter_ >= 280 && (global.chassis_rx->chassis_data_rx.HeatLimit - heat_ultimate_ > 20)) {
      // 执行单发
      global.motor->shoot_controller.SetMode(Shoot3Fric::kSingleShot);
      shoot_cycle_counter_ = 0;  // 发射后重置计数器
      a++;
    } else {
      // 间隔未满：仅计数，不发射
      shoot_cycle_counter_++;
    }
    last_is_three = true;
  } else {
    // 无开火信号：停止电机，并继续计数（保持间隔计时）
    global.motor->shoot_controller.SetMode(Shoot3Fric::kStop);
    shoot_cycle_counter_++;
    last_is_three = false;
  }
  shoot_cycle_counter = shoot_cycle_counter_;

  global.motor->shoot_controller.Fire();
  global.motor->shoot_controller.Update(global.motor->ammo_left->rpm(), global.motor->ammo_right->rpm(), 0,
                                        static_cast<f32>(global.motor->dail_encoder_counter.revolutions()) * 8191.0f +
                                            static_cast<f32>(global.motor->dail_encoder_counter.last_ecd()),
                                        global.motor->dial_motor->rpm());
}

void Motor::ShootAutoControl() {
  FricSpeedUpdate();
  global.motor->dail_encoder_counter.Update(global.motor->dial_motor->encoder());
  shoot_controller.SetLeftArmSpeed(left_set_speed);
  shoot_controller.SetRightArmSpeed(right_set_speed);
  device_aimbot.Update();
  ShooterCounter();
  HeatUpdate();

  if (!device_aimbot.all_device_ok()) {
    if ((global.bc->rc->mouse_button_left() == 1 || tcremote.data().mouse_button_left == 1 ||
         global.bc->rc->right_x() < -650) &&
        (global.chassis_rx->chassis_data_rx.HeatLimit - heat_ultimate_ > 20)) {
      shoot_controller.SetMode(Shoot3Fric::kFullAuto);
      shoot_frequency = -24.0f;
      shoot_controller.SetShootFrequency(shoot_frequency);
    } else {
      shoot_controller.SetMode(Shoot3Fric::kStop);
    }
  } else {
    heat = global.chassis_rx->chassis_data_rx.HeatLimit - global.chassis_rx->chassis_data_rx.HeatCurrent;
    if ((global.bc->rc->right_x() < -650) ||
        ((global.bc->rc->mouse_button_left() == 1 || tcremote.data().mouse_button_left == 1) &&
         (global.bc->rc->mouse_button_right() == 1 || tcremote.data().mouse_button_right == 1) &&
         (aimbot_comm->aimbot_state() >> 1 & 0x01) &&
         (global.chassis_rx->chassis_data_rx.HeatLimit - heat_ultimate_ > 20)) ||
        (tcremote.data().mouse_button_left == 1 && (global.chassis_rx->chassis_data_rx.HeatLimit - heat_ultimate_ > 20))
        // (global.bc->rc->mouse_button_left() == 1 &&
        //  global.chassis_rx->chassis_data_rx.HeatLimit - heat_ultimate > 40)
    ) {
      shoot_controller.SetMode(Shoot3Fric::kFullAuto);
      shoot_frequency = -24.0f;
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
  yaw_pos = yaw_motor->pos();
  yaw_pid_debug = gimbal_controller.output().yaw;
  global.motor->gravity_compensation_ = 1.6f * std::cos(global.bc->pitch / 57.3f + 0.29f);
  gravity_compensation = global.motor->gravity_compensation_;
  pitch_motor->SetMitCommand(0.f, 0.f, gravity_compensation + gimbal_controller.output().pitch, 0.f, 0.f);
  pitch_pid_debug = pitch_motor->tau();
  pitch_speed = pitch_motor->vel();
  yaw_motor->SetMitCommand(0.f, 0.f, gimbal_controller.output().yaw + global.motor->yaw_compensation_, 0.f, 0.f);
  pitch_tau = gravity_compensation + gimbal_controller.output().pitch;
}

void Motor::SendDjiCommand() {
  rpm_left_ = ammo_left->rpm();
  rpm_right_ = -ammo_right->rpm();
  o1 = static_cast<i16>(shoot_controller.output().fric_1);
  o2 = static_cast<i16>(shoot_controller.output().fric_2);
  o3 = static_cast<i16>(shoot_controller.output().loader);
  if (shoot_enabled_ == 0) {
    shoot_controller.SetLeftArmSpeed(0.);
    shoot_controller.SetRightArmSpeed(0.);
    shoot_controller.SetMode(Shoot3Fric::kStop);
    global.motor->shoot_controller.Fire();
    global.motor->shoot_controller.Update(global.motor->ammo_left->rpm(), global.motor->ammo_right->rpm(), 0,
                                          static_cast<f32>(global.motor->dail_encoder_counter.revolutions()) * 8191.0f +
                                              static_cast<f32>(global.motor->dail_encoder_counter.last_ecd()),
                                          global.motor->dial_motor->rpm());
    o1 = static_cast<i16>(shoot_controller.output().fric_1);
    o2 = static_cast<i16>(shoot_controller.output().fric_2);
    o3 = static_cast<i16>(shoot_controller.output().loader);
    ammo_left->SetCurrent(o1);
    ammo_right->SetCurrent(o2);
    dial_motor->SetCurrent(-o3);
  } else {
    ammo_left->SetCurrent(o1);
    ammo_right->SetCurrent(o2);
    dial_motor->SetCurrent(-o3);
  }
  dial_debug = o1;

  state_debug = aimbot_comm->aimbot_state();

  DjiMotorBase::SendCommand(*can1);
  DjiMotorBase::SendCommand(*can2);
}

void Motor::Transit_initmode(bool keyboard_e) {
  static bool last_keyboard_e = 0;  // 上一帧按键状态
  static bool yaw_toggle_flag = 0;  // 0: 0rad, 1: -pi

  if (keyboard_e && !last_keyboard_e) {
    yaw_toggle_flag = !yaw_toggle_flag;  // 翻转状态

    if (yaw_toggle_flag) {
      yaw_init = -M_PI;
    } else {
      yaw_init = 0.f;
    }

    // 进入初始化流程
    global.fsm.init_count_ = 0;
  }

  last_keyboard_e = keyboard_e;
}