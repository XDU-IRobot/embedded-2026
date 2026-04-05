#include "Gimbal.hpp"

void Gimbal::GimbalInit() {
  gimbal->gimbal_up_yaw_target_ = globals->hipnuc_imu->yaw();
  gimbal->gimbal_down_yaw_target_ = globals->ahrs.euler_angle().yaw;
  gimbal->gimbal_pitch_target_ = globals->hipnuc_imu->pitch();
  gimbal->up_yaw_move_limiter_.ResetAt(globals->hipnuc_imu->yaw());
  gimbal->down_yaw_move_limiter_.ResetAt(globals->ahrs.euler_angle().yaw);
  gimbal->pitch_torque_ = 0.0f;
}

void Gimbal::GimbalTask() {
  gimbal->GimbalStateUpdate();
  gimbal->heat_limit_ = globals->referee_data->data().robot_status.shooter_barrel_heat_limit;
  gimbal->heat_current_ = globals->referee_data->data().power_heat_data.shooter_17mm_1_barrel_heat;
}

void Gimbal::GimbalStateUpdate() {
  if (!globals->device_gimbal.all_device_ok()) {
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
  if (globals->referee_data->data().robot_status.power_management_shooter_output && !globals->last_shooter_power) {
    globals->shooter_init_time = 1500;
  }
  globals->last_shooter_power = globals->referee_data->data().robot_status.power_management_shooter_output;
  if (globals->shooter_init_time > 0) {
    globals->shooter_init_time--;
  }
  if (!globals->device_shoot.all_device_ok() || globals->shooter_init_time > 0 ||
      !globals->referee_data->data().robot_status.power_management_shooter_output) {
    gimbal->ShootDisableUpdate();  // 发射机构失能计算
  } else {
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
  }
}

void Gimbal::GimbalRCTargetUpdate() {
  gimbal->gimbal_up_yaw_target_ -= rm::modules::Map(globals->wfly_et16s->left_x(), -1, 1, -0.004f, 0.004f);
  // gimbal->gimbal_up_yaw_target_ -= rm::modules::Map(globals->wfly_et16s->left_x(), -1, 1, -0.004f, 0.004f);
  gimbal->gimbal_down_yaw_target_ -= rm::modules::Map(globals->wfly_et16s->left_x(), -1, 1, -0.004f, 0.004f);
  gimbal->gimbal_pitch_target_ -= rm::modules::Map(globals->wfly_et16s->left_y(), -1, 1, -0.004f, 0.004f);
  gimbal->gimbal_up_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_up_yaw_target_, -static_cast<f32>(M_PI), M_PI);
  gimbal->gimbal_down_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_down_yaw_target_, -static_cast<f32>(M_PI), M_PI);
  gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_,  // pitch轴限位
                                                    gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);
  gimbal->up_yaw_move_limiter_.ResetAt(globals->hipnuc_imu->yaw());
  gimbal->down_yaw_move_limiter_.ResetAt(globals->ahrs.euler_angle().yaw);
}

void Gimbal::GimbalScanTargetUpdate() {
  // 上部yaw轴扫描
  if (globals->navigate_communicator->aimbot_mode()) {
    if (globals->up_yaw_motor->encoder() >= gimbal->max_mechanism_up_yaw_pos_) {
      gimbal->scan_yaw_flag_ = true;
    } else if (globals->up_yaw_motor->encoder() <= gimbal->min_mechanism_up_yaw_pos_) {
      gimbal->scan_yaw_flag_ = false;
    }
  } else {
    if (globals->up_yaw_motor->encoder() >= gimbal->max_up_yaw_pos_) {
      gimbal->scan_yaw_flag_ = true;
    } else if (globals->up_yaw_motor->encoder() <= gimbal->min_up_yaw_pos_) {
      gimbal->scan_yaw_flag_ = false;
    }
  }
  if (gimbal->scan_yaw_flag_) {
    gimbal->gimbal_up_yaw_target_ -= 0.0025f;
  } else {
    gimbal->gimbal_up_yaw_target_ += 0.0025f;
  }
  // pitch轴扫描
  if (globals->navigate_communicator->aimbot_mode()) {
    if (gimbal->gimbal_pitch_target_ <= gimbal->lowest_mechanism_pitch_angle_) {
      gimbal->scan_pitch_flag_ = false;
    } else if (gimbal->gimbal_pitch_target_ >= gimbal->highest_mechanism_pitch_angle_) {
      gimbal->scan_pitch_flag_ = true;
    }
  } else {
    if (gimbal->gimbal_pitch_target_ <= gimbal->lowest_aimbot_pitch_angle_) {
      gimbal->scan_pitch_flag_ = false;
    } else if (gimbal->gimbal_pitch_target_ >= gimbal->highest_pitch_angle_) {
      gimbal->scan_pitch_flag_ = true;
    }
  }
  if (gimbal->scan_pitch_flag_) {
    gimbal->gimbal_pitch_target_ -= 0.005f;
  } else {
    gimbal->gimbal_pitch_target_ += 0.005f;
  }
  // 下部yaw轴扫描
  if (globals->navigate_communicator->scan_mode()) {
    gimbal->GimbalMove_ = kGbScan;
  } else {
    // gimbal->GimbalMove_ = kGbScan;
    gimbal->GimbalMove_ = kGbNavigate;
  }
  if (gimbal->perception_time_ > 0) {
    gimbal->perception_time_--;
  } else {
    if (gimbal->GimbalMove_ == kGbNavigate) {
      gimbal->gimbal_down_yaw_target_ +=
          rm::modules::Map(rm::modules::Clamp(globals->navigate_communicator->target_yaw_speed(), -1.0f, 1.0f), -1.0f,
                           1.0f, -0.01f, 0.01f);
    } else {
      gimbal->gimbal_down_yaw_target_ += 0.001f;
    }
  }
  // 基于下部yaw轴转速增减上部yaw轴转速
  if (gimbal->GimbalMove_ == kGbNavigate) {
    gimbal->gimbal_up_yaw_target_ +=
        rm::modules::Map(rm::modules::Clamp(globals->navigate_communicator->target_yaw_speed(), -1.0f, 1.0f), -1.0f,
                         1.0f, -0.01f, 0.01f);
  } else {
    gimbal->gimbal_up_yaw_target_ += 0.001f;
  }
  gimbal->gimbal_up_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_up_yaw_target_,  // 上部yaw轴周期限制
                                                    -static_cast<f32>(M_PI), M_PI);
  gimbal->gimbal_down_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_down_yaw_target_,  // 下部yaw轴周期限制
                                                      -static_cast<f32>(M_PI), M_PI);

  gimbal->up_yaw_move_limiter_.ResetAt(globals->hipnuc_imu->yaw());
  gimbal->down_yaw_move_limiter_.ResetAt(globals->ahrs.euler_angle().yaw);
}

void Gimbal::GimbalPerceptTargetUpdate() {
  if (gimbal->percept_move_complete_) {
    if (globals->navigate_communicator->perception_flag() >> 0 & 0x01) {
      gimbal->up_yaw_percept_target_ = globals->hipnuc_imu->yaw() + static_cast<f32>(M_PI) / 2.0f;
      gimbal->down_yaw_percept_target_ = globals->ahrs.euler_angle().yaw + static_cast<f32>(M_PI) / 2.0f;
    } else if (globals->navigate_communicator->perception_flag() >> 2 & 0x01) {
      gimbal->up_yaw_percept_target_ = globals->hipnuc_imu->yaw() - static_cast<f32>(M_PI) / 2.0f;
      gimbal->down_yaw_percept_target_ = globals->ahrs.euler_angle().yaw - static_cast<f32>(M_PI) / 2.0f;
    } else if (globals->navigate_communicator->perception_flag() >> 1 & 0x01) {
      gimbal->up_yaw_percept_target_ = globals->hipnuc_imu->yaw() + static_cast<f32>(M_PI);
      gimbal->down_yaw_percept_target_ = globals->ahrs.euler_angle().yaw + static_cast<f32>(M_PI);
    }
    gimbal->up_yaw_move_limiter_.SetTarget(gimbal->up_yaw_percept_target_);
    gimbal->down_yaw_move_limiter_.SetTarget(gimbal->down_yaw_percept_target_);
    gimbal->percept_move_complete_ = false;
  }
  gimbal->gimbal_up_yaw_target_ = gimbal->up_yaw_move_limiter_.Update(0.002f);
  gimbal->gimbal_down_yaw_target_ = gimbal->down_yaw_move_limiter_.Update(0.002f);
  gimbal->gimbal_up_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_up_yaw_target_,  // 上部yaw轴周期限制
                                                    -static_cast<f32>(M_PI), M_PI);
  gimbal->gimbal_down_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_down_yaw_target_,  // 下部yaw轴周期限制
                                                      -static_cast<f32>(M_PI), M_PI);
  gimbal->percept_move_complete_ = gimbal->up_yaw_move_limiter_.IsAtTarget(0.001f);
  if (gimbal->percept_move_complete_) {
    gimbal->up_yaw_move_limiter_.ResetAt(globals->hipnuc_imu->yaw());
    gimbal->down_yaw_move_limiter_.ResetAt(globals->ahrs.euler_angle().yaw);
    gimbal->perception_time_ = 1000;
  }
  // pitch轴扫描
  if (gimbal->gimbal_pitch_target_ <= gimbal->lowest_aimbot_pitch_angle_) {
    gimbal->scan_pitch_flag_ = false;
  } else if (gimbal->gimbal_pitch_target_ >= gimbal->highest_pitch_angle_) {
    gimbal->scan_pitch_flag_ = true;
  }
  if (gimbal->scan_pitch_flag_) {
    gimbal->gimbal_pitch_target_ -= 0.004f;
  } else {
    gimbal->gimbal_pitch_target_ += 0.004f;
  }
}

void Gimbal::GimbalAimbotTargetUpdate() {
  if (globals->aimbot_communicator->aimbot_state() >> 0 & 0x01) {
    if (globals->up_yaw_motor->encoder() >= gimbal->down_yaw_move_high_) {
      gimbal->gimbal_down_yaw_target_ += 0.001;
    } else if (globals->up_yaw_motor->encoder() <= gimbal->down_yaw_move_low_) {
      gimbal->gimbal_down_yaw_target_ -= 0.001;
    }
    auto aimbot_target_yaw = rm::modules::Map(rm::modules::Wrap(globals->aimbot_communicator->yaw(), -180.0f, 180.0f),
                                              0.0f, 360.0f, 0.0f, 2.0f * static_cast<f32>(M_PI));
    if ((globals->up_yaw_motor->encoder() >= gimbal->max_up_yaw_pos_ &&
         aimbot_target_yaw >= globals->hipnuc_imu->yaw()) ||
        (globals->up_yaw_motor->encoder() <= gimbal->min_up_yaw_pos_ &&
         aimbot_target_yaw <= globals->hipnuc_imu->yaw())) {
      gimbal->gimbal_up_yaw_target_ = aimbot_target_yaw;
      gimbal->up_yaw_move_limiter_.SetTarget(aimbot_target_yaw);
      gimbal->gimbal_up_yaw_target_ = gimbal->up_yaw_move_limiter_.Update(0.002f);
      gimbal->gimbal_up_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_up_yaw_target_, -static_cast<f32>(M_PI), M_PI);
      if (gimbal->up_yaw_move_limiter_.IsAtTarget(0.001f)) {
        gimbal->up_yaw_move_limiter_.ResetAt(globals->hipnuc_imu->yaw());
      }
      gimbal->gimbal_down_yaw_target_ =
          aimbot_target_yaw - globals->hipnuc_imu->yaw() + globals->ahrs.euler_angle().yaw;
      gimbal->down_yaw_move_limiter_.SetTarget(gimbal->gimbal_down_yaw_target_);
      gimbal->gimbal_down_yaw_target_ = gimbal->down_yaw_move_limiter_.Update(0.002f);
      gimbal->gimbal_down_yaw_target_ =
          rm::modules::Wrap(gimbal->gimbal_down_yaw_target_, -static_cast<f32>(M_PI), M_PI);
      if (gimbal->down_yaw_move_limiter_.IsAtTarget(0.001f)) {
        gimbal->down_yaw_move_limiter_.ResetAt(globals->ahrs.euler_angle().yaw);
      }
    } else {
      gimbal->gimbal_up_yaw_target_ = aimbot_target_yaw;
      gimbal->up_yaw_move_limiter_.ResetAt(globals->hipnuc_imu->yaw());
      gimbal->down_yaw_move_limiter_.ResetAt(globals->ahrs.euler_angle().yaw);
    }
    gimbal->gimbal_pitch_target_ = rm::modules::Wrap(
        rm::modules::Map(globals->aimbot_communicator->pitch(), 0.0f, 360.0f, 0.0f, 2.0f * static_cast<f32>(M_PI)),
        -static_cast<f32>(M_PI), M_PI);
    gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_,  // pitch轴限位
                                                      gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);
    gimbal->aimbot_time_ = 100;
  } else if (gimbal->aimbot_time_ > 0) {
    gimbal->aimbot_time_--;
  } else {
    gimbal->GimbalRCTargetUpdate();
  }
}

void Gimbal::GimbalMovePIDUpdate() {
  globals->gimbal_controller.SetTarget(gimbal->gimbal_up_yaw_target_, gimbal->gimbal_down_yaw_target_,  //
                                       gimbal->gimbal_pitch_target_);
  globals->gimbal_controller.Update(globals->hipnuc_imu->yaw(), globals->hipnuc_imu->gyro_z(),
                                    globals->ahrs.euler_angle().yaw, globals->imu->gyro_z(),
                                    globals->hipnuc_imu->pitch(), globals->hipnuc_imu->gyro_x(), 2.0f);
  // const f32 move_compensation_ = globals->down_yaw_motor->vel() / 9.0f;
  // gimbal->down_yaw_torque_ = globals->gimbal_controller.output().down_yaw + move_compensation_;
  // gimbal->down_yaw_torque_ = rm::modules::Clamp(gimbal->down_yaw_torque_, -10.0f, 10.0f);
  const f32 gravity_compensation_ = -1.82f * std::cos(globals->hipnuc_imu->pitch() + 0.2115f);
  gimbal->pitch_torque_ = globals->gimbal_controller.output().pitch + gravity_compensation_;
  gimbal->pitch_torque_ = rm::modules::Clamp(gimbal->pitch_torque_, -10.0f, 10.0f);
}

void Gimbal::GimbalMatchUpdate() {
  if (globals->aimbot_communicator->aimbot_state() >> 0 & 0x01) {
    gimbal->GimbalMove_ = kGbAimbot;
    gimbal->percept_move_complete_ = true;
    gimbal->perception_time_ = 0;
  } else if ((globals->navigate_communicator->perception_flag() != 0x00 || !gimbal->percept_move_complete_) &&
             gimbal->perception_time_ <= 0) {
    gimbal->GimbalMove_ = kGbPercept;
  } else if (globals->navigate_communicator->scan_mode()) {
    gimbal->GimbalMove_ = kGbScan;
  } else {
    gimbal->GimbalMove_ = kGbNavigate;
  }
  gimbal->GimbalEnableUpdate();
}

void Gimbal::GimbalEnableUpdate() {
  globals->gimbal_controller.Enable(true);
  if (gimbal->GimbalMove_ == kGbRemote) {
    gimbal->GimbalRCTargetUpdate();
  } else if (gimbal->GimbalMove_ == kGbPercept) {
    gimbal->GimbalPerceptTargetUpdate();
  } else if (gimbal->GimbalMove_ == kGbScan || gimbal->GimbalMove_ == kGbNavigate) {
    gimbal->GimbalScanTargetUpdate();
  } else if (gimbal->GimbalMove_ == kGbAimbot) {
    gimbal->GimbalAimbotTargetUpdate();
  } else {
    gimbal->GimbalDisableUpdate();
    return;
  }
  if (globals->navigate_communicator->aimbot_mode()) {
    if (globals->referee_data->data().game_status.SyncTimeStamp >= 240) {
      globals->aim_mode = 0x02;
    } else {
      globals->aim_mode = 0x03;
    }
  } else {
    globals->aim_mode = 0x01;
  }
  gimbal->GimbalMovePIDUpdate();
  gimbal->DaMiaoMotorEnable();
  gimbal->SetMotorCurrent();
}

void Gimbal::GimbalDisableUpdate() {
  globals->gimbal_controller.Enable(false);
  globals->aim_mode = 0x01;
  gimbal->gimbal_up_yaw_target_ = globals->hipnuc_imu->yaw();
  gimbal->gimbal_down_yaw_target_ = globals->ahrs.euler_angle().yaw;
  gimbal->gimbal_pitch_target_ = globals->hipnuc_imu->pitch();
  gimbal->up_yaw_move_limiter_.ResetAt(globals->hipnuc_imu->yaw());
  gimbal->down_yaw_move_limiter_.ResetAt(globals->ahrs.euler_angle().yaw);
  gimbal->GimbalMovePIDUpdate();
  gimbal->DaMiaoMotorDisable();
  gimbal->SetMotorCurrent();
  gimbal->pitch_torque_ = 0.0f;
}

void Gimbal::DaMiaoMotorEnable() {
  if (globals->down_yaw_motor->status() != 0x1F && globals->down_yaw_motor->status() != 0x0F) {
    globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  } else if (globals->pitch_motor->status() != 0x1F && globals->pitch_motor->status() != 0x0F) {
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  } else {
    if (globals->down_yaw_motor->status() == 0x0F) {
      globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    }
    if (globals->pitch_motor->status() == 0x0F) {
      globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    }
  }
}

void Gimbal::DaMiaoMotorDisable() {
  if (globals->down_yaw_motor->status() != 0x1F && globals->down_yaw_motor->status() != 0x0F) {
    globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  } else if (globals->pitch_motor->status() != 0x1F && globals->pitch_motor->status() != 0x0F) {
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  } else {
    if (globals->down_yaw_motor->status() == 0x1F) {
      globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    }
    if (globals->pitch_motor->status() == 0x1F) {
      globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    }
  }
}

void Gimbal::ShootEnableUpdate() {
  globals->shoot_controller.Enable(true);
  globals->shoot_controller.Arm(true);
  globals->shoot_controller.SetArmSpeed(gimbal->ammo_speed_);
  globals->dail_encoder_counter.Update(globals->dial_motor->encoder());
  if (globals->referee_data->data().shoot_data.initial_speed >= 22.0f ||
      (globals->referee_data->data().shoot_data.initial_speed >= 15.0f &&
       globals->referee_data->data().shoot_data.initial_speed <= 21.0f)) {
    gimbal->ammo_speed_ = 6200.0f * std::sqrt(22.0f / globals->referee_data->data().shoot_data.initial_speed);
  }
  if (((globals->wfly_et16s->wheel_position(rc_ch::LS) <= -650 &&
        globals->wfly_et16s->switch_position(rc_ch::SH) == SwitchPosition::kDown) ||
       (globals->StateMachine_ == kTest && globals->wfly_et16s->wheel_position(rc_ch::LS) <= -10 &&
        globals->wfly_et16s->switch_position(rc_ch::SH) == SwitchPosition::kDown &&
        globals->aimbot_communicator->aimbot_state() >> 1 & 0x01) ||
       (globals->StateMachine_ == kMatch && globals->navigate_communicator->aimbot_mode() &&
        globals->aimbot_communicator->aimbot_state() >> 1 & 0x01)) &&
      heat_limit_ - heat_current_ > 30) {
    if (!single_shoot_flag_) {
      globals->shoot_controller.SetMode(Shoot3Fric::kSingleShot);
      globals->shoot_controller.Fire();
      single_shoot_flag_ = true;
      gimbal->single_shoot_time_ = 200;
    } else if (gimbal->single_shoot_time_ > 0) {
      globals->shoot_controller.SetShootFrequency(0.0f);
      gimbal->single_shoot_time_--;
    } else if (gimbal->single_shoot_time_ == 0) {
      globals->shoot_controller.SetShootFrequency(0.0f);
      single_shoot_flag_ = false;
    }
  } else if (globals->wfly_et16s->wheel_position(rc_ch::LS) >= 650 ||
             (globals->StateMachine_ == kTest && globals->wfly_et16s->wheel_position(rc_ch::LS) >= 10 &&
              globals->aimbot_communicator->aimbot_state() >> 1 & 0x01) ||
             (globals->StateMachine_ == kMatch && !globals->navigate_communicator->aimbot_mode() &&
              globals->aimbot_communicator->aimbot_state() >> 1 & 0x01)) {
    globals->shoot_controller.SetMode(Shoot3Fric::kFullAuto);
    if (heat_limit_ - heat_current_ > 100) {
      globals->shoot_controller.SetShootFrequency(20.0f);
    } else if (heat_limit_ - heat_current_ < 20) {
      globals->shoot_controller.SetShootFrequency(0.0f);
    } else {
      globals->shoot_controller.SetShootFrequency(static_cast<f32>(heat_limit_ - heat_current_) / 6.0f + 5.0f);
    }
  } else {
    globals->shoot_controller.SetShootFrequency(0.0f);
    single_shoot_flag_ = false;
  }
  globals->shoot_controller.Update(globals->friction_left->rpm(), globals->friction_right->rpm(), 0,
                                   static_cast<f32>(globals->dail_encoder_counter.linear_ticks()),
                                   globals->dial_motor->rpm());
}

void Gimbal::ShootDisableUpdate() {
  globals->shoot_controller.SetMode(Shoot3Fric::kStop);
  if (!globals->referee_data->data().robot_status.power_management_shooter_output) {
    globals->shoot_controller.Enable(false);
    globals->shoot_controller.Arm(false);
  } else {
    globals->shoot_controller.Enable(true);
    globals->shoot_controller.Arm(true);
    globals->shoot_controller.SetArmSpeed(0.0f);
    globals->shoot_controller.SetShootFrequency(0.0f);
  }
  globals->dail_encoder_counter.Reset(0, globals->dial_motor->encoder());
  globals->shoot_controller.Update(globals->friction_left->rpm(), globals->friction_right->rpm(), 0,
                                   static_cast<f32>(globals->dail_encoder_counter.linear_ticks()),
                                   globals->dial_motor->rpm());
}

void Gimbal::SetMotorCurrent() {
  globals->up_yaw_motor->SetCurrent(static_cast<i16>(globals->gimbal_controller.output().up_yaw));
  globals->friction_left->SetCurrent(static_cast<i16>(globals->shoot_controller.output().fric_1));
  globals->friction_right->SetCurrent(static_cast<i16>(globals->shoot_controller.output().fric_2));
  globals->dial_motor->SetCurrent(static_cast<i16>(globals->shoot_controller.output().loader));
}