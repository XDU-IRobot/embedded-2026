#include "Gimbal.hpp"

float a, b, c, d;

void Gimbal::GimbalInit() {
  gimbal->gimbal_up_yaw_target_ = globals->up_yaw_motor->encoder();
  gimbal->gimbal_down_yaw_target_ = globals->ahrs.euler_angle().yaw;
  gimbal->gimbal_pitch_target_ = globals->pitch_motor->pos();
}

void Gimbal::GimbalTask() {
  gimbal->GimbalStateUpdate();
  gimbal->heat_limit_ = globals->referee_data_buffer->data().robot_status.shooter_barrel_heat_limit;
  gimbal->heat_current_ = globals->referee_data_buffer->data().power_heat_data.shooter_17mm_1_barrel_heat;
  f32 yaw = rm::modules::Map(globals->up_yaw_motor->encoder(), 0.0f, globals->GM6020_encoder_max_, 0.0f,
                             2.0f * static_cast<f32>(M_PI));
  yaw = rm::modules::Wrap(yaw, -static_cast<f32>(M_PI), M_PI);
  gimbal->EulerToQuaternion(yaw, -globals->pitch_motor->pos(), 0.0f);
  a = globals->NucControl.vx;
  b = globals->NucControl.vy;
  c = globals->NucControl.vw;
  d = globals->NucControl.yaw_speed;
}

void Gimbal::GimbalStateUpdate() {
  if (!globals->referee_data_buffer->data().robot_status.power_management_gimbal_output ||
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
  if (!globals->device_shoot.all_device_ok()) {
    gimbal->ShootDisableUpdate();  // 发射机构失能计算
  } else {
    switch (globals->StateMachine_) {
      case kMatch:                    // 比赛模式下，所有电机正常工作
        gimbal->ShootEnableUpdate();  // 发射机构使能计算
        break;
      case kTest:                      // 测试模式下，发射系统与拨盘电机失能
      case kNoForce:                   // 无力模式下，所有电机失能
      default:                         // 错误状态，所有电机失能
        gimbal->ShootDisableUpdate();  // 发射机构失能计算
        break;
    }
  }
}

void Gimbal::GimbalRCTargetUpdate() {
  if (gimbal->GimbalMove_ == kGbRemote) {
    // gimbal->gimbal_up_yaw_target_ -= rm::modules::Map(globals->rc->left_x(),  // 上部yaw轴目标值
    //                                                   -globals->rc_max_value_, globals->rc_max_value_,
    //                                                   -gimbal->sensitivity_up_yaw_, gimbal->sensitivity_up_yaw_);
    gimbal->gimbal_up_yaw_target_ = gimbal->mid_up_yaw_angle_;
  }
  gimbal->gimbal_down_yaw_target_ -= rm::modules::Map(globals->rc->left_x(),  // 上部yaw轴目标值
                                                      -globals->rc_max_value_, globals->rc_max_value_,
                                                      -gimbal->sensitivity_down_yaw_, gimbal->sensitivity_down_yaw_);
  gimbal->gimbal_pitch_target_ += rm::modules::Map(globals->rc->left_y(),  // pitch轴目标值
                                                   -globals->rc_max_value_, globals->rc_max_value_,
                                                   -gimbal->sensitivity_pitch_, gimbal->sensitivity_pitch_);
  // gimbal->GimbalDownYawFollow();
  gimbal->gimbal_up_yaw_target_ = rm::modules::Clamp(gimbal->gimbal_up_yaw_target_,  // 上部yaw轴限位
                                                     gimbal->min_up_yaw_angle_, gimbal->max_up_yaw_angle_);
  gimbal->gimbal_down_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_down_yaw_target_,  // 下部yaw轴周期限制
                                                      -static_cast<f32>(M_PI), M_PI);
  gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_,  // pitch轴限位
                                                    gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);
}

void Gimbal::GimbalScanTargetUpdate() {
  if (gimbal->gimbal_up_yaw_target_ >= gimbal->max_up_yaw_angle_) {
    gimbal->scan_yaw_flag_ = true;
  } else if (gimbal->gimbal_up_yaw_target_ <= gimbal->min_up_yaw_angle_) {
    gimbal->scan_yaw_flag_ = false;
  }
  if (gimbal->scan_yaw_flag_) {
    gimbal->gimbal_up_yaw_target_ -= 3.0f;
  } else {
    gimbal->gimbal_up_yaw_target_ += 3.0f;
  }
  if (gimbal->gimbal_pitch_target_ >= gimbal->highest_pitch_angle_) {
    gimbal->scan_pitch_flag_ = true;
  } else if (gimbal->gimbal_pitch_target_ <= gimbal->lowest_pitch_angle_) {
    gimbal->scan_pitch_flag_ = false;
  }
  if (gimbal->scan_pitch_flag_) {
    gimbal->gimbal_pitch_target_ -= 0.003f;
  } else {
    gimbal->gimbal_pitch_target_ += 0.003f;
  }
  // if (globals->NucControl.scan_mode) {
  gimbal->GimbalMove_ = kGbScan;
  // } else {
  //   gimbal->GimbalMove_ = kGbNavigate;
  // }
  if (gimbal->GimbalMove_ == kGbNavigate) {
    gimbal->gimbal_down_yaw_target_ += rm::modules::Map(globals->NucControl.yaw_speed, -1.0f, 1.0f, -0.0005f, 0.0005f);
  } else {
    gimbal->gimbal_down_yaw_target_ += 0.001f;
  }
  gimbal->gimbal_down_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_down_yaw_target_,  // 下部yaw轴周期限制
                                                      -static_cast<f32>(M_PI), M_PI);
}

void Gimbal::GimbalAimbotTargetUpdate() {
  if (globals->Aimbot.AimbotState >> 0 & 0x01) {
    gimbal->gimbal_up_yaw_target_ =
        rm::modules::Map(rm::modules::Wrap(globals->Aimbot.Yaw, 0.0f, 360.0f), 0.0f, 360.0f, 0.0f, 8191.0f);
    gimbal->gimbal_pitch_target_ =
        rm::modules::Wrap(rm::modules::Map(-globals->Aimbot.Pitch, 0.0f, 360.0f, 0.0f, 2.0f * static_cast<f32>(M_PI)),
                          -static_cast<f32>(M_PI), M_PI);
  } else {
    gimbal->GimbalRCTargetUpdate();
  }
  gimbal->GimbalDownYawFollow();
}

void Gimbal::GimbalDownYawFollow() {
  if (gimbal->gimbal_up_yaw_target_ > gimbal->down_yaw_move_high_) {
    // gimbal->gimbal_up_yaw_target_ = gimbal->down_yaw_move_high_;
    gimbal->gimbal_down_yaw_target_ += 0.002f;
  } else if (gimbal->gimbal_up_yaw_target_ < gimbal->down_yaw_move_low_) {
    // gimbal->gimbal_up_yaw_target_ = gimbal->down_yaw_move_low_;
    gimbal->gimbal_down_yaw_target_ -= 0.002f;
  }
  gimbal->gimbal_down_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_down_yaw_target_,  // 下部yaw轴周期限制
                                                      -static_cast<f32>(M_PI), M_PI);
}

void Gimbal::GimbalMovePIDUpdate() {
  globals->gimbal_controller.SetTarget(gimbal->gimbal_up_yaw_target_, gimbal->gimbal_down_yaw_target_,  //
                                       gimbal->gimbal_pitch_target_);
  globals->gimbal_controller.Update(globals->up_yaw_motor->encoder(), globals->up_yaw_motor->rpm(),
                                    globals->ahrs.euler_angle().yaw, globals->down_yaw_motor->vel(),
                                    globals->pitch_motor->pos(), globals->pitch_motor->vel());
  gimbal->gravity_compensation_ = gimbal->k_gravity_compensation_ * std::cos(globals->pitch_motor->pos());
  gimbal->pitch_torque_ = globals->gimbal_controller.output().pitch + gimbal->gravity_compensation_;
  gimbal->pitch_torque_ = rm::modules::Clamp(pitch_torque_, -10.0f, 10.0f);
}

void Gimbal::GimbalMatchUpdate() {
  if (globals->Aimbot.AimbotState >> 0 & 0x01) {
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
  gimbal->gimbal_up_yaw_target_ = globals->up_yaw_motor->encoder();
  gimbal->gimbal_down_yaw_target_ = globals->ahrs.euler_angle().yaw;
  gimbal->gimbal_pitch_target_ = globals->pitch_motor->pos();
  gimbal->gravity_compensation_ = 0.0f;
  gimbal->GimbalMovePIDUpdate();
  gimbal->SetMotorCurrent();
}

void Gimbal::DaMiaoMotorEnable() {
  if (gimbal->DM_enable_flag_ == false) {
    // 使达妙电机使能
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    gimbal->DM_enable_flag_ = true;
  }
}

void Gimbal::DaMiaoMotorDisable() {
  if (gimbal->DM_enable_flag_ == true) {
    // 使达妙电机失能
    globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    gimbal->DM_enable_flag_ = false;
  }
}

void Gimbal::ShootEnableUpdate() {
  globals->shoot_controller.Enable(true);
  globals->shoot_controller.Arm(true);
  // gimbal->AmmoSpeedUpdate();
  globals->shoot_controller.SetArmSpeed(ammo_speed_);
  globals->dail_position_counter.IncreaseUpdate(globals->dial_motor->encoder());
  if (globals->rc->dial() <= -650
      // && heat_limit_ - heat_current_ > 100
  ) {
    if (!single_shoot_flag_) {
      globals->shoot_controller.SetMode(Shoot3Fric::kSingleShot);
      single_shoot_flag_ = true;
    } else {
      globals->shoot_controller.SetMode(Shoot3Fric::kStop);
    }
  } else if (globals->rc->dial() >= 650 || globals->Aimbot.AimbotState >> 1 & 0x01) {
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
  globals->shoot_controller.Update(globals->friction_left->rpm(), globals->friction_right->rpm(), 0,
                                   globals->dail_position_counter.output(), globals->dial_motor->rpm());
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
  globals->dail_position_counter.Init(globals->dial_motor->encoder());
  globals->shoot_controller.Update(globals->friction_left->rpm(), globals->friction_right->rpm(), 0,
                                   globals->dail_position_counter.output(), globals->dial_motor->rpm());
}

void Gimbal::AmmoSpeedUpdate() {
  if (globals->referee_data_buffer->data().shoot_data.initial_speed > 20.0f &&
      globals->referee_data_buffer->data().shoot_data.initial_speed < 35.0f &&
      globals->referee_data_buffer->data().projectile_allowance.projectile_allowance_17mm !=
          gimbal->last_remain_bullet_) {
    if (gimbal->shoot_initial_speed_[0] == 0.0f) {
      for (float &i : gimbal->shoot_initial_speed_) {
        i = globals->referee_data_buffer->data().shoot_data.initial_speed;
      }
    } else {
      gimbal->shoot_initial_speed_[shoot_num_] = globals->referee_data_buffer->data().shoot_data.initial_speed;
      if (shoot_num_ < 10) {
        shoot_num_++;
      } else {
        shoot_num_ = 0;
      }
    }
    gimbal->last_remain_bullet_ = globals->referee_data_buffer->data().projectile_allowance.projectile_allowance_17mm;
  }
  gimbal->shoot_initial_average_speed_ /= 10.0f;
  if (gimbal->shoot_initial_speed_[0] == 0.0f) {
    gimbal->ammo_speed_ = gimbal->ammo_init_speed_;
  } else {
    gimbal->ammo_speed_ =
        gimbal->ammo_init_speed_ / gimbal->shoot_initial_average_speed_ * gimbal->target_shoot_initial_speed_;
  }
}

void Gimbal::SetMotorCurrent() {
  globals->up_yaw_motor->SetCurrent(static_cast<i16>(globals->gimbal_controller.output().up_yaw));
  globals->friction_left->SetCurrent(static_cast<i16>(globals->shoot_controller.output().fric_1));
  globals->friction_right->SetCurrent(static_cast<i16>(globals->shoot_controller.output().fric_2));
  globals->dial_motor->SetCurrent(static_cast<i16>(globals->shoot_controller.output().loader));
}

void Gimbal::EulerToQuaternion(f32 yaw, f32 pitch, f32 roll) {
  // 计算各角度的一半
  f32 halfYaw = yaw * 0.5f;
  f32 halfPitch = pitch * 0.5f;
  f32 halfRoll = roll * 0.5f;

  // 计算各角度一半的正弦和余弦值
  f32 cosYaw = cos(halfYaw);
  f32 sinYaw = sin(halfYaw);
  f32 cosPitch = cos(halfPitch);
  f32 sinPitch = sin(halfPitch);
  f32 cosRoll = cos(halfRoll);
  f32 sinRoll = sin(halfRoll);

  // 根据ZYX旋转顺序计算四元数分量
  globals->up_yaw_qw = cosYaw * cosPitch * cosRoll + sinYaw * sinPitch * sinRoll;
  globals->up_yaw_qx = -sinYaw * sinPitch * cosRoll + cosYaw * cosPitch * sinRoll;
  globals->up_yaw_qy = cosYaw * sinPitch * cosRoll + sinYaw * cosPitch * sinRoll;
  globals->up_yaw_qz = sinYaw * cosPitch * cosRoll - cosYaw * sinPitch * sinRoll;

  // 归一化四元数
  // 计算四元数的模长平方
  f32 norm = globals->up_yaw_qw * globals->up_yaw_qw + globals->up_yaw_qx * globals->up_yaw_qx +
             globals->up_yaw_qy * globals->up_yaw_qy + globals->up_yaw_qz * globals->up_yaw_qz;

  // 如果模长不为零，则进行归一化
  if (norm > 0.0f) {
    // 使用标准库函数计算归一化因子
    f32 invNorm = 1.0f / sqrt(norm);
    globals->up_yaw_qw *= invNorm;
    globals->up_yaw_qx *= invNorm;
    globals->up_yaw_qy *= invNorm;
    globals->up_yaw_qz *= invNorm;
  } else {
    // 如果模长为零，返回单位四元数
    globals->up_yaw_qw = 1.0f;
    globals->up_yaw_qx = 0.0f;
    globals->up_yaw_qy = 0.0f;
    globals->up_yaw_qz = 0.0f;
  }
}
