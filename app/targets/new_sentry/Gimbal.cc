#include "Gimbal.hpp"

void Gimbal::GimbalInit() {
  gimbal->gimbal_up_yaw_target_ = globals->hipnuc_imu->yaw();
  gimbal->gimbal_down_yaw_target_ = globals->ahrs.euler_angle().yaw;
  gimbal->gimbal_pitch_target_ = globals->hipnuc_imu->roll();
}

void Gimbal::GimbalTask() {
  gimbal->GimbalStateUpdate();
  gimbal->heat_limit_ = globals->referee_data_buffer->data().robot_status.shooter_barrel_heat_limit;
  gimbal->heat_current_ = globals->referee_data_buffer->data().power_heat_data.shooter_17mm_1_barrel_heat;
}

void Gimbal::GimbalStateUpdate() {
  if (
      // !globals->referee_data_buffer->data().robot_status.power_management_gimbal_output ||
      !globals->device_gimbal.all_device_ok()) {
    gimbal->GimbalDisableUpdate();  // 云台电机失能计算
  } else {
    switch (globals->StateMachine_) {
      case kNoForce:                    // 无力模式下，所有电机失能
        gimbal->GimbalDisableUpdate();  // 云台电机失能计算
        break;

      case kTest:                      // 测试模式下，发射系统与拨盘电机失能
        gimbal->GimbalEnableUpdate();  // 云台电机使能计算
        break;

      case kMatch:                    // 比赛模式下，所有电机正常工作
        gimbal->GimbalMatchUpdate();  // 云台电机使能计算
        break;

      default:                          // 错误状态，所有电机失能
        gimbal->GimbalDisableUpdate();  // 云台电机失能计算
        break;
    }
  }
  // if (!globals->device_shoot.all_device_ok()) {
  //   gimbal->ShootDisableUpdate();  // 发射机构失能计算
  // } else {
    switch (globals->StateMachine_) {
      case kMatch:                    // 比赛模式下，发射系统与拨盘电使能
        gimbal->ShootEnableUpdate();  // 发射机构使能计算
        break;
      case kTest:
        switch (gimbal->GimbalMove_) {
          case kGbAimbot:
            gimbal->ShootEnableUpdate();  // 发射机构使能计算
            break;
          case kGbRemote:
          default:
            gimbal->ShootDisableUpdate();  // 发射机构失能计算
            break;
        }
        break;
      case kNoForce:                   // 无力模式下，所有电机失能
      default:                         // 错误状态，所有电机失能
        gimbal->ShootDisableUpdate();  // 发射机构失能计算
        break;
    }
  // }
}

void Gimbal::GimbalRCTargetUpdate() {
  // if (globals->up_yaw_motor->encoder() >= gimbal->max_up_yaw_pos_ &&
  //     globals->rc->left_x() < 0 && !gimbal->down_yaw_move_flag_) {
  //   gimbal->down_yaw_move_flag_ = true;
  // } else if (globals->up_yaw_motor->encoder() <= gimbal->min_up_yaw_pos &&
  //            globals->rc->left_x() > 0 && !gimbal->down_yaw_move_flag_) {
  //   gimbal->down_yaw_move_flag_ = true;
  // } else {
  //   gimbal->gimbal_up_yaw_target_ -= rm::modules::Map(globals->rc->left_x(), -660, 660,  // 上部yaw轴目标值
  //                                                     -gimbal->sensitivity_up_yaw_, gimbal->sensitivity_up_yaw_);
  // }
  // if (gimbal->down_yaw_move_flag_ && globals->rc->left_x() < 0) {
  //   gimbal->gimbal_up_yaw_target_ = globals->hipnuc_imu->yaw();
  //   gimbal->gimbal_down_yaw_target_ += 0.003;
  // } else if (gimbal->down_yaw_move_flag_ && globals->rc->left_x() > 0) {
  //   gimbal->gimbal_up_yaw_target_ = globals->hipnuc_imu->yaw();
  //   gimbal->gimbal_down_yaw_target_ -= 0.003;
  // } else {
  //   gimbal->down_yaw_move_flag_ = false;
  // }
  gimbal->gimbal_up_yaw_target_ = globals->hipnuc_imu->yaw();
  gimbal->gimbal_down_yaw_target_ -= rm::modules::Map(globals->rc->left_x(), -660, 660,  // 上部yaw轴目标值
                                                      -gimbal->sensitivity_down_yaw_, gimbal->sensitivity_down_yaw_);
  gimbal->gimbal_pitch_target_ += rm::modules::Map(globals->rc->left_y(), -660, 660,  // pitch轴目标值
                                                   -gimbal->sensitivity_pitch_, gimbal->sensitivity_pitch_);
  gimbal->gimbal_up_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_up_yaw_target_,  // 上部yaw轴周期限制
                                                    -static_cast<f32>(M_PI), M_PI);
  gimbal->gimbal_down_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_down_yaw_target_,  // 下部yaw轴周期限制
                                                      -static_cast<f32>(M_PI), M_PI);
  gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_,  // pitch轴限位
                                                    gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);
}

void Gimbal::GimbalScanTargetUpdate() {
  if (globals->up_yaw_motor->encoder() >= gimbal->max_up_yaw_pos_) {
    gimbal->scan_yaw_flag_ = true;
  } else if (globals->up_yaw_motor->encoder() <= gimbal->min_up_yaw_pos_) {
    gimbal->scan_yaw_flag_ = false;
  }
  if (gimbal->scan_yaw_flag_) {
    gimbal->gimbal_up_yaw_target_ -= 0.002f;
  } else {
    gimbal->gimbal_up_yaw_target_ += 0.002f;
  }
  if (gimbal->GimbalMove_ == kGbNavigate) {
    gimbal->gimbal_up_yaw_target_ +=
        rm::modules::Map(rm::modules::Clamp(globals->NucControl.yaw_speed, -1.0f, 1.0f), -1.0f, 1.0f, -0.01f, 0.01f);
  } else {
    gimbal->gimbal_up_yaw_target_ += 0.001f;
  }
  gimbal->gimbal_up_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_up_yaw_target_,  // 上部yaw轴周期限制
                                                    -static_cast<f32>(M_PI), M_PI);
  if (gimbal->gimbal_pitch_target_ >= gimbal->highest_aimbot_pitch_angle_) {
    gimbal->scan_pitch_flag_ = true;
  } else if (gimbal->gimbal_pitch_target_ <= gimbal->lowest_pitch_angle_) {
    gimbal->scan_pitch_flag_ = false;
  }
  if (gimbal->scan_pitch_flag_) {
    gimbal->gimbal_pitch_target_ -= 0.004f;
  } else {
    gimbal->gimbal_pitch_target_ += 0.004f;
  }
  if (globals->NucControl.scan_mode) {
    gimbal->GimbalMove_ = kGbScan;
  } else {
    gimbal->GimbalMove_ = kGbNavigate;
  }
  if (gimbal->GimbalMove_ == kGbNavigate) {
    gimbal->gimbal_down_yaw_target_ +=
        rm::modules::Map(rm::modules::Clamp(globals->NucControl.yaw_speed, -1.0f, 1.0f), -1.0f, 1.0f, -0.01f, 0.01f);
  } else {
    gimbal->gimbal_down_yaw_target_ += 0.001f;
  }
  gimbal->gimbal_down_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_down_yaw_target_,  // 下部yaw轴周期限制
                                                      -static_cast<f32>(M_PI), M_PI);
}

void Gimbal::GimbalAimbotTargetUpdate() {
  if (globals->Aimbot.AimbotState >> 0 & 0x01) {
    gimbal->gimbal_up_yaw_target_ = rm::modules::Map(rm::modules::Wrap(globals->Aimbot.Yaw, -180.0f, 180.0f),  //
                                                     0.0f, 360.0f, 0.0f, 2.0f * static_cast<f32>(M_PI));
    gimbal->gimbal_pitch_target_ =
        rm::modules::Wrap(rm::modules::Map(-globals->Aimbot.Pitch, 0.0f, 360.0f, 0.0f, 2.0f * static_cast<f32>(M_PI)),
                          -static_cast<f32>(M_PI), M_PI);
    gimbal->GimbalDownYawFollow();
    gimbal->gimbal_down_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_down_yaw_target_,  // 下部yaw轴周期限制
                                                        -static_cast<f32>(M_PI), M_PI);
    gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_,  // pitch轴限位
                                                      gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);
  } else if (globals->can_communicator->aimbot_state() >> 0 & 0x01) {
    gimbal->gimbal_up_yaw_target_ =
        rm::modules::Map(rm::modules::Wrap(globals->can_communicator->yaw(), -180.0f, 180.0f),  //
                         0.0f, 360.0f, 0.0f, 2.0f * static_cast<f32>(M_PI));
    gimbal->gimbal_pitch_target_ = rm::modules::Wrap(
        rm::modules::Map(-globals->can_communicator->pitch(), 0.0f, 360.0f, 0.0f, 2.0f * static_cast<f32>(M_PI)),
        -static_cast<f32>(M_PI), M_PI);
    gimbal->GimbalDownYawFollow();
    gimbal->gimbal_down_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_down_yaw_target_,  // 下部yaw轴周期限制
                                                        -static_cast<f32>(M_PI), M_PI);
    gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_,  // pitch轴限位
                                                      gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);
  } else {
    gimbal->GimbalRCTargetUpdate();
  }
}

void Gimbal::GimbalDownYawFollow() {
  if ((globals->up_yaw_motor->encoder() >= gimbal->max_up_yaw_pos_ && globals->up_yaw_motor->encoder() < 4000 &&
       gimbal->gimbal_up_yaw_target_ >= globals->hipnuc_imu->yaw()) ||
      (globals->up_yaw_motor->encoder() <= gimbal->min_up_yaw_pos_ && globals->up_yaw_motor->encoder() > 4000 &&
       gimbal->gimbal_up_yaw_target_ <= globals->hipnuc_imu->yaw())) {
    gimbal->gimbal_up_yaw_target_ = globals->hipnuc_imu->yaw();
    gimbal->gimbal_down_yaw_target_ =
        gimbal->gimbal_up_yaw_target_ - globals->hipnuc_imu->yaw() + globals->ahrs.euler_angle().yaw;
  }
}

void Gimbal::GimbalMovePIDUpdate() {
  globals->gimbal_controller.SetTarget(gimbal->gimbal_up_yaw_target_, gimbal->gimbal_down_yaw_target_,  //
                                       gimbal->gimbal_pitch_target_);
  globals->gimbal_controller.Update(globals->hipnuc_imu->yaw(), globals->up_yaw_motor->rpm(),
                                    globals->ahrs.euler_angle().yaw, globals->down_yaw_motor->vel(),
                                    globals->hipnuc_imu->roll(), globals->pitch_motor->vel());
  const f32 gravity_compensation_ = 1.1f * std::cos(globals->hipnuc_imu->roll() + 0.377f);
  gimbal->pitch_torque_ = globals->gimbal_controller.output().pitch + gravity_compensation_;
  gimbal->pitch_torque_ = rm::modules::Clamp(pitch_torque_, -10.0f, 10.0f);
}

void Gimbal::GimbalMatchUpdate() {
  if (globals->Aimbot.AimbotState >> 0 & 0x01 || globals->can_communicator->aimbot_state() >> 0 & 0x01) {
    gimbal->GimbalMove_ = kGbAimbot;
  } else if (globals->NucControl.scan_mode) {
    gimbal->GimbalMove_ = kGbScan;
  } else {
    gimbal->GimbalMove_ = kGbNavigate;
  }
  gimbal->GimbalEnableUpdate();
}

void Gimbal::GimbalEnableUpdate() {
  gimbal->DaMiaoMotorEnable();
  globals->gimbal_controller.Enable(true);
  globals->GimbalData.aim_mode = 0x01;
  globals->aim_mode = 0x01;
  if (gimbal->GimbalMove_ == kGbRemote) {
    gimbal->GimbalRCTargetUpdate();
    gimbal->GimbalMovePIDUpdate();
  } else if (gimbal->GimbalMove_ == kGbScan || gimbal->GimbalMove_ == kGbNavigate) {
    gimbal->GimbalScanTargetUpdate();
    gimbal->GimbalMovePIDUpdate();
  } else if (gimbal->GimbalMove_ == kGbAimbot) {
    gimbal->GimbalAimbotTargetUpdate();
    gimbal->GimbalMovePIDUpdate();
  } else {
    globals->gimbal_controller.Enable(false);
  }
  gimbal->SetMotorCurrent();
}

void Gimbal::GimbalDisableUpdate() {
  gimbal->DaMiaoMotorDisable();
  globals->gimbal_controller.Enable(false);
  globals->GimbalData.aim_mode = 0x00;
  globals->aim_mode = 0x00;
  gimbal->gimbal_up_yaw_target_ = globals->hipnuc_imu->yaw();
  gimbal->gimbal_down_yaw_target_ = globals->ahrs.euler_angle().yaw;
  gimbal->gimbal_pitch_target_ = globals->hipnuc_imu->roll();
  gimbal->pitch_torque_ = 0.0f;
  gimbal->GimbalMovePIDUpdate();
  gimbal->SetMotorCurrent();
}

void Gimbal::DaMiaoMotorEnable() {
  if (gimbal->down_yaw_enable_flag_ == false && gimbal->pitch_enable_flag_ == true) {
    globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    gimbal->down_yaw_enable_flag_ = true;
  }
  if (gimbal->pitch_enable_flag_ == false) {
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    gimbal->pitch_enable_flag_ = true;
  }
}

void Gimbal::DaMiaoMotorDisable() {
  if (gimbal->down_yaw_enable_flag_ == true && gimbal->pitch_enable_flag_ == false) {
    globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    gimbal->down_yaw_enable_flag_ = false;
  }
  if (gimbal->pitch_enable_flag_ == true) {
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    gimbal->pitch_enable_flag_ = false;
  }
}

void Gimbal::ShootEnableUpdate() {
  globals->shoot_controller.Enable(true);
  globals->shoot_controller.Arm(true);
  // gimbal->AmmoSpeedUpdate();
  globals->shoot_controller.SetArmSpeed(ammo_speed_);
  globals->dail_encoder_counter.Update(globals->dial_motor->encoder());
  if (globals->rc->dial() <= -650
      // && heat_limit_ - heat_current_ > 100
  ) {
    if (!single_shoot_flag_) {
      globals->shoot_controller.SetMode(Shoot3Fric::kSingleShot);
      single_shoot_flag_ = true;
    } else {
      globals->shoot_controller.SetMode(Shoot3Fric::kStop);
    }
  } else if (globals->rc->dial() >= 650 || globals->Aimbot.AimbotState >> 1 & 0x01 ||
             globals->can_communicator->aimbot_state() >> 1 & 0x01) {
    globals->shoot_controller.SetMode(Shoot3Fric::kFullAuto);
    // if (heat_limit_ - heat_current_ > 100) {
    gimbal->shoot_frequency_ = -30.0f;
    // } else if (heat_limit_ - heat_current_ < 40) {
    //     gimbal->shoot_frequency_ = 0.0f;
    // } else {
    //     gimbal->shoot_frequency_ = -std::pow(static_cast<f32>(heat_limit_ - heat_current_) / 100.0f, 2.0f) * 20.0f;
    // }
    globals->shoot_controller.SetShootFrequency(shoot_frequency_);  // 负值为正转
  } else {
    globals->shoot_controller.SetMode(Shoot3Fric::kStop);
    single_shoot_flag_ = false;
  }
  globals->shoot_controller.Fire();
  globals->shoot_controller.Update(globals->friction_left->rpm(), globals->friction_right->rpm(), 0, 0,
                                   globals->dial_motor->rpm());
}

void Gimbal::ShootDisableUpdate() {
  globals->shoot_controller.SetMode(Shoot3Fric::kStop);
  if (globals->StateMachine_ == kUnable) {
    globals->shoot_controller.Enable(false);
    globals->shoot_controller.Arm(false);
  } else {
    globals->shoot_controller.Enable(true);
    globals->shoot_controller.SetArmSpeed(0.0f);
  }
  globals->shoot_controller.Fire();
  globals->dail_encoder_counter.Reset(globals->dial_motor->encoder());
  globals->shoot_controller.Update(globals->friction_left->rpm(), globals->friction_right->rpm(), 0, 0,
                                   globals->dial_motor->rpm());
}

void Gimbal::SetMotorCurrent() {
  globals->up_yaw_motor->SetCurrent(static_cast<i16>(globals->gimbal_controller.output().up_yaw));
  globals->friction_left->SetCurrent(static_cast<i16>(globals->shoot_controller.output().fric_1));
  globals->friction_right->SetCurrent(static_cast<i16>(globals->shoot_controller.output().fric_2));
  globals->dial_motor->SetCurrent(static_cast<i16>(globals->shoot_controller.output().loader));
}