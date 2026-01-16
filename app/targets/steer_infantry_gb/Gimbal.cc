#include "Gimbal.hpp"

u8 a;

void Gimbal::GimbalInit() {
  gimbal->gimbal_yaw_target_ = globals->ahrs.euler_angle().yaw;
  gimbal->gimbal_pitch_target_ = -globals->ahrs.euler_angle().pitch;
}

void Gimbal::GimbalTask() { gimbal->GimbalStateUpdate(); }

void Gimbal::GimbalStateUpdate() {
  // if (!globals->device_gimbal.all_device_ok()) {
  //   gimbal->GimbalDisableUpdate();  // 云台电机失能计算
  // } else {
  switch (globals->StateMachine_) {
    case kNoForce:                    // 无力模式下，所有电机失能
      gimbal->GimbalDisableUpdate();  // 云台电机失能计算
      break;

    case kTest:                      // 测试模式下，发射系统与拨盘电机失能
      gimbal->GimbalEnableUpdate();  // 云台电机使能计算
      break;

    default:                          // 错误状态，所有电机失能
      gimbal->GimbalDisableUpdate();  // 云台电机失能计算
      break;
  }
  // }
  // if (!globals->device_shoot.all_device_ok()) {
  //   gimbal->ShootDisableUpdate();  // 发射机构失能计算
  // } else {
  switch (globals->StateMachine_) {
    case kTest:  // 测试模式下，发射系统与拨盘电机失能
      switch (gimbal->GimbalMove_) {
        case kGbAimbot:
          gimbal->ShootEnableUpdate();  // 发射机构使能计算
        case kGbRemote:
        default:
          gimbal->ShootDisableUpdate();  // 发射机构失能计算
      }
    case kNoForce:                   // 无力模式下，所有电机失能
    default:                         // 错误状态，所有电机失能
      gimbal->ShootDisableUpdate();  // 发射机构失能计算
      break;
  }
  // }
}

void Gimbal::GimbalRCTargetUpdate() {
  gimbal->gimbal_yaw_target_ -= rm::modules::Map(globals->rc->left_x(),  // 上部yaw轴目标值
                                                 -globals->rc_max_value_, globals->rc_max_value_,
                                                 -gimbal->sensitivity_yaw_, gimbal->sensitivity_yaw_);
  gimbal->gimbal_pitch_target_ -= rm::modules::Map(globals->rc->left_y(),  // pitch轴目标值
                                                   -globals->rc_max_value_, globals->rc_max_value_,
                                                   -gimbal->sensitivity_pitch_, gimbal->sensitivity_pitch_);
  gimbal->gimbal_yaw_target_ =
      rm::modules::Wrap(gimbal->gimbal_yaw_target_, 0.0f, 2.0f * static_cast<f32>(M_PI));  // yaw轴限位
  gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_,  // pitch轴限位
                                                    gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);
}

void Gimbal::GimbalAimbotTargetUpdate() {
  if (globals->can_communicator->aimbot_state() >> 0 & 0x01) {
    gimbal->gimbal_yaw_target_ = rm::modules::Map(rm::modules::Wrap(globals->can_communicator->yaw(), 0.0f, 360.0f),
                                                  0.0f, 360.0f, 0.0f, 8191.0f);
    gimbal->gimbal_pitch_target_ = rm::modules::Wrap(
        rm::modules::Map(-globals->can_communicator->pitch(), 0.0f, 360.0f, 0.0f, 2.0f * static_cast<f32>(M_PI)),
        -static_cast<f32>(M_PI), M_PI);
  } else {
    gimbal->GimbalRCTargetUpdate();
  }
}

void Gimbal::GimbalMovePIDUpdate() {
  globals->gimbal_controller.SetTarget(gimbal->gimbal_yaw_target_, gimbal->gimbal_pitch_target_);
  globals->gimbal_controller.Update(globals->ahrs.euler_angle().yaw, globals->yaw_motor->rpm(),
                                    -globals->ahrs.euler_angle().pitch, globals->pitch_motor->vel());
  gimbal->gravity_compensation_ = gimbal->k_gravity_compensation_ * std::cos(globals->ahrs.euler_angle().pitch);
  gimbal->pitch_torque_ = globals->gimbal_controller.output().pitch + gimbal->gravity_compensation_;
  gimbal->pitch_torque_ = rm::modules::Clamp(gimbal->pitch_torque_, -10.0f, 10.0f);
}

void Gimbal::GimbalEnableUpdate() {
  gimbal->DaMiaoMotorEnable();
  globals->gimbal_controller.Enable(true);
  globals->aim_mode = 0x01;
  if (gimbal->GimbalMove_ == kGbRemote) {
    gimbal->GimbalRCTargetUpdate();
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
  globals->aim_mode = 0x00;
  gimbal->gimbal_yaw_target_ = globals->ahrs.euler_angle().yaw;
  gimbal->gimbal_pitch_target_ = -globals->ahrs.euler_angle().pitch;
  gimbal->gravity_compensation_ = 0.0f;
  gimbal->GimbalMovePIDUpdate();
  gimbal->SetMotorCurrent();
}

void Gimbal::DaMiaoMotorEnable() {
  if (gimbal->DM_enable_flag_ == false) {
    // 使达妙电机使能
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    gimbal->DM_enable_flag_ = true;
  }
}

void Gimbal::DaMiaoMotorDisable() {
  if (gimbal->DM_enable_flag_ == true) {
    // 使达妙电机失能
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    gimbal->DM_enable_flag_ = false;
  }
}

void Gimbal::ShootEnableUpdate() {
  globals->shoot_controller.Enable(true);
  globals->shoot_controller.Arm(true);
  // gimbal->AmmoSpeedUpdate();
  globals->shoot_controller.SetArmSpeed(ammo_speed_);
  globals->dail_encoder_counter.Update(globals->dial_motor->encoder());
  if ((globals->can_communicator->aimbot_state() >> 0 & 0x01 &&
       globals->can_communicator->aimbot_state() >> 1 & 0x01) ||
      globals->rc->dial() >= 650) {
    globals->shoot_controller.SetMode(Shoot3Fric::kFullAuto);
    gimbal->shoot_frequency_ = 30.0f;
    globals->shoot_controller.SetShootFrequency(shoot_frequency_);
  } else {
    globals->shoot_controller.SetMode(Shoot3Fric::kStop);
    single_shoot_flag_ = false;
  }
  globals->shoot_controller.Fire();
  globals->shoot_controller.Update(
      globals->friction_left->rpm(), globals->friction_right->rpm(), 0,
      globals->dail_encoder_counter.revolutions() * 8191 + globals->dail_encoder_counter.last_ecd(),
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
  globals->shoot_controller.Update(
      globals->friction_left->rpm(), globals->friction_right->rpm(), 0,
      globals->dail_encoder_counter.revolutions() * 8191 + globals->dail_encoder_counter.last_ecd(),
      globals->dial_motor->rpm());
}

void Gimbal::SetMotorCurrent() {
  globals->yaw_motor->SetCurrent(static_cast<i16>(globals->gimbal_controller.output().yaw));
  globals->friction_left->SetCurrent(static_cast<i16>(globals->shoot_controller.output().fric_1));
  globals->friction_right->SetCurrent(static_cast<i16>(globals->shoot_controller.output().fric_2));
  globals->dial_motor->SetCurrent(static_cast<i16>(globals->shoot_controller.output().loader));
}