#include "Gimbal.hpp"

f32 a, b, c, d;

void Gimbal::GimbalInit() {
  gimbal->gimbal_yaw_target_ = globals->ahrs.euler_angle().yaw;
  gimbal->gimbal_pitch_target_ = globals->ahrs.euler_angle().pitch;
}

void Gimbal::GimbalTask() {
  gimbal->GimbalStateUpdate();
  a = gimbal->gimbal_yaw_target_;
  b = gimbal->gimbal_pitch_target_;
  c = globals->ahrs.euler_angle().yaw;
  d = globals->ahrs.euler_angle().pitch;
}

void Gimbal::GimbalStateUpdate() {
  if (!globals->device_gimbal.all_device_ok() || !globals->chassis_communicator->gimbal_power_state()) {
    globals->StateMachine_ = kUnable;  // 如果云台设备离线或云台供电异常，进入无力模式
    gimbal->GimbalDisableUpdate();     // 云台电机失能计算
  } else {
    switch (globals->StateMachine_) {
      case kNoForce:                    // 无力模式下，所有电机失能
        gimbal->GimbalDisableUpdate();  // 云台电机失能计算
        break;

      case kTest:                      // 测试模式下，发射系统与拨盘电机失能
        gimbal->GimbalEnableUpdate();  // 云台电机使能计算
        break;

      case kMatch:
        gimbal->GimbalMatchUpdate();
        break;

      default:                          // 错误状态，所有电机失能
        gimbal->GimbalDisableUpdate();  // 云台电机失能计算
        break;
    }
  }
  if (!globals->device_shoot.all_device_ok() || !globals->chassis_communicator->ammo_power_state()) {
    gimbal->ShootDisableUpdate();  // 发射机构失能计算
  } else {
    switch (globals->StateMachine_) {
      case kMatch:
        gimbal->ShootEnableUpdate();  // 发射机构使能计算
        break;
      case kTest:  // 测试模式下，发射系统与拨盘电机失能
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
  gimbal->gimbal_yaw_target_ -= rm::modules::Map(
      static_cast<f32>(globals->rc->left_x()) +
          30.0f * static_cast<f32>(globals->image_update_flag ? globals->image_data->data().mouse_x
                                                              : globals->rc->mouse_x()),  // 上部yaw轴目标值
      -660, 660, -gimbal->sensitivity_yaw_, gimbal->sensitivity_yaw_);
  gimbal->gimbal_pitch_target_ -= rm::modules::Map(
      static_cast<f32>(globals->rc->left_y()) +
          30.0f * static_cast<f32>(globals->image_update_flag ? globals->image_data->data().mouse_y
                                                              : globals->rc->mouse_y()),  // pitch轴目标值
      -660, 660, -gimbal->sensitivity_pitch_, gimbal->sensitivity_pitch_);
  gimbal->gimbal_yaw_target_ =
      rm::modules::Wrap(gimbal->gimbal_yaw_target_, 0.f, 2.f * static_cast<f32>(M_PI));  // yaw轴限位
  gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_,        // pitch轴限位
                                                    gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);
}

void Gimbal::GimbalAimbotTargetUpdate() {
  if ((globals->StateMachine_ == kTest && globals->aimbot_communicator->aimbot_state() >> 0 & 0x01) ||
      (globals->StateMachine_ == kMatch && (globals->image_update_flag ? globals->image_data->data().mouse_button_right
                                                                       : globals->rc->mouse_button_right()))) {
    gimbal->gimbal_yaw_target_ = globals->aimbot_communicator->yaw();
    gimbal->gimbal_pitch_target_ = globals->aimbot_communicator->pitch();
  } else {
    gimbal->GimbalRCTargetUpdate();
  }
}

void Gimbal::GimbalMovePIDUpdate() {
  gimbal->yaw_speed_ff = gimbal->Kf * (gimbal->gimbal_yaw_target_ - last_yaw_target) / Ts;
  gimbal->last_yaw_target = gimbal->gimbal_yaw_target_;

  globals->gimbal_controller.SetTarget(gimbal->gimbal_yaw_target_, gimbal->gimbal_pitch_target_, yaw_speed_ff);
  globals->gimbal_controller.Update(globals->ahrs.euler_angle().yaw, globals->yaw_motor->rpm(),
                                    globals->ahrs.euler_angle().pitch, globals->pitch_motor->vel());
  f32 gravity_compensation_ = -0.74f * std::cos(globals->ahrs.euler_angle().pitch - 0.25f);
  gimbal->pitch_torque_ = globals->gimbal_controller.output().pitch + gravity_compensation_;
  gimbal->pitch_torque_ = rm::modules::Clamp(gimbal->pitch_torque_, -10.f, 10.f);
}

void Gimbal::GimbalMatchUpdate() {
  if (globals->aimbot_communicator->aimbot_state() >> 0 & 0x01) {
    gimbal->GimbalMove_ = kGbAimbot;
  } else {
    gimbal->GimbalMove_ = kGbRemote;
  }
  gimbal->GimbalEnableUpdate();
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
  gimbal->gimbal_pitch_target_ = globals->ahrs.euler_angle().pitch;
  gimbal->GimbalMovePIDUpdate();
  gimbal->SetMotorCurrent();
  gimbal->pitch_torque_ = 0.f;
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
  globals->shoot_controller.SetArmSpeed(gimbal->ammo_speed_ - static_cast<f32>(globals->aim_speed_change) * 100.0f);
  globals->dail_encoder_counter.Update(globals->dial_motor->encoder());
  if (const u16 heat_delta = globals->chassis_communicator->heat_limit() - globals->chassis_communicator->heat_real();
      globals->rc->dial() <= -650 || (heat_delta > 30 && (globals->df_state || globals->xf_state) &&
                                      (globals->image_update_flag ? globals->image_data->data().mouse_button_left
                                                                  : globals->rc->mouse_button_left()))) {
    if (!gimbal->single_shoot_flag_) {
      globals->shoot_controller.SetMode(Shoot3Fric::kSingleShot);
      gimbal->single_shoot_flag_ = true;
    } else {
      globals->shoot_controller.SetMode(Shoot3Fric::kStop);
    }
  } else if ((globals->StateMachine_ == kTest &&
              ((globals->rc->dial() >= 10 && globals->aimbot_communicator->aimbot_state() >> 0 & 0x01 &&
                globals->aimbot_communicator->aimbot_state() >> 1 & 0x01) ||
               globals->rc->dial() >= 650)) ||
             (globals->StateMachine_ == kMatch &&
              (globals->image_update_flag ? globals->image_data->data().mouse_button_left
                                          : globals->rc->mouse_button_left()) &&  // 左键按下
              ((globals->image_update_flag ? !globals->image_data->data().mouse_button_right
                                           : !globals->rc->mouse_button_right()) ||  // 右键未按下
               ((globals->image_update_flag ? globals->image_data->data().mouse_button_right
                                            : globals->rc->mouse_button_right()) &&  // 右键按下且瞄到目标
                globals->aimbot_communicator->aimbot_state() >> 0 & 0x01 &&
                globals->aimbot_communicator->aimbot_state() >> 1 & 0x01)))) {
    globals->shoot_controller.SetMode(Shoot3Fric::kFullAuto);
    if (heat_delta > 100) {
      globals->shoot_controller.SetShootFrequency(20.0f);
    } else if (heat_delta < 20) {
      globals->shoot_controller.SetShootFrequency(0.0f);
    } else {
      globals->shoot_controller.SetShootFrequency(static_cast<f32>(heat_limit_ - heat_current_) / 6.0f + 5.0f);
    }
  } else {
    globals->shoot_controller.SetMode(Shoot3Fric::kStop);
    gimbal->single_shoot_flag_ = false;
  }
  globals->shoot_controller.Fire();
  globals->shoot_controller.Update(globals->friction_left->rpm(), globals->friction_right->rpm(), 0,
                                   static_cast<f32>(globals->dail_encoder_counter.linear_ticks()),
                                   globals->dial_motor->rpm());
}

void Gimbal::ShootDisableUpdate() {
  globals->shoot_controller.SetMode(Shoot3Fric::kStop);
  if (globals->StateMachine_ == kUnable) {
    globals->shoot_controller.Enable(false);
    globals->shoot_controller.Arm(false);
  } else {
    globals->shoot_controller.Enable(true);
    globals->shoot_controller.SetArmSpeed(0.f);
  }
  globals->shoot_controller.Fire();
  globals->dail_encoder_counter.Reset(0, globals->dial_motor->encoder());
  globals->shoot_controller.Update(globals->friction_left->rpm(), globals->friction_right->rpm(), 0,
                                   static_cast<f32>(globals->dail_encoder_counter.linear_ticks()),
                                   globals->dial_motor->rpm());
}

void Gimbal::SetMotorCurrent() {
  globals->yaw_motor->SetCurrent(static_cast<i16>(globals->gimbal_controller.output().yaw));
  globals->friction_left->SetCurrent(static_cast<i16>(globals->shoot_controller.output().fric_1));
  globals->friction_right->SetCurrent(static_cast<i16>(globals->shoot_controller.output().fric_2));
  globals->dial_motor->SetCurrent(static_cast<i16>(globals->shoot_controller.output().loader));
}