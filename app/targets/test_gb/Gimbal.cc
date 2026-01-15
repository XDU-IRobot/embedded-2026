#include "Gimbal.hpp"

void Gimbal::GimbalInit() {
  // gimbal->gimbal_yaw_target_ = globals->ahrs.euler_angle().yaw;
  // gimbal->gimbal_pitch_target_ = globals->ahrs.euler_angle().pitch;
  gimbal->gimbal_yaw_target_ = globals->hipnuc_imu->yaw();
  gimbal->gimbal_pitch_target_ = globals->hipnuc_imu->roll();
}

void Gimbal::GimbalTask() { gimbal->GimbalStateUpdate(); }

void Gimbal::GimbalStateUpdate() {
  if (!globals->device_gimbal.all_device_ok()) {
    gimbal->GimbalDisableUpdate();
  } else {
    switch (globals->StateMachine_) {
      case kNoForce:                    // 无力模式下，所有电机失能
        gimbal->GimbalDisableUpdate();  // 云台电机失能计算
        break;

      case kTest:  // 测试模式下，发射系统与拨盘电机失能
        switch (gimbal->GimbalMove_) {
          case kGbRemote:
          case kGbAimbot:
            gimbal->GimbalEnableUpdate();  // 云台电机使能计算
            break;

          default:
            gimbal->GimbalDisableUpdate();  // 云台电机失能计算
            break;
        }
        break;

      default:                          // 错误状态，所有电机失能
        gimbal->GimbalDisableUpdate();  // 云台电机失能计算
        break;
    }
  }
}

void Gimbal::GimbalRCTargetUpdate() {
  gimbal->gimbal_yaw_target_ -= rm::modules::Map(globals->rc->left_x(), -globals->rc_max_value_, globals->rc_max_value_,
                                                 -gimbal->sensitivity_, gimbal->sensitivity_);      // 上部yaw轴目标值
  gimbal->gimbal_pitch_target_ -= rm::modules::Map(globals->rc->left_y(), -globals->rc_max_value_,  // pitch轴目标值
                                                   globals->rc_max_value_, -gimbal->sensitivity_, gimbal->sensitivity_);
  gimbal->gimbal_yaw_target_ =
      rm::modules::Wrap(gimbal->gimbal_yaw_target_, -static_cast<f32>(M_PI), M_PI);  // yaw轴周期限位
  gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_,    // pitch轴限位
                                                    gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);
}

void Gimbal::GimbalAimbotTargetUpdate() {
  if (globals->can_communicator->aimbot_state() >> 0 & 0x01) {
    gimbal->gimbal_yaw_target_ = globals->can_communicator->yaw();
    gimbal->gimbal_pitch_target_ = globals->can_communicator->pitch();
  } else if (globals->Aimbot.AimbotState >> 0 & 0x01) {
    gimbal->gimbal_yaw_target_ = globals->Aimbot.Yaw;
    gimbal->gimbal_pitch_target_ = globals->Aimbot.Pitch;
  } else {
    gimbal->gimbal_yaw_target_ -=
        rm::modules::Map(globals->rc->left_x(), -globals->rc_max_value_, globals->rc_max_value_, -gimbal->sensitivity_,
                         gimbal->sensitivity_);  // 上部yaw轴目标值
    gimbal->gimbal_pitch_target_ -=
        rm::modules::Map(globals->rc->left_y(), -globals->rc_max_value_,  // pitch轴目标值
                         globals->rc_max_value_, -gimbal->sensitivity_, gimbal->sensitivity_);
  }
  gimbal->gimbal_yaw_target_ =
      rm::modules::Wrap(gimbal->gimbal_yaw_target_, -static_cast<f32>(M_PI), M_PI);  // yaw轴周期限位
  gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_,    // pitch轴限位
                                                    gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);
}

void Gimbal::GimbalMovePIDUpdate() {
  // globals->gimbal_controller.SetTarget(gimbal->gimbal_yaw_target_, gimbal->gimbal_pitch_target_);
  // globals->gimbal_controller.Update(globals->ahrs.euler_angle().yaw, globals->yaw_motor->vel(),
  //                                   globals->ahrs.euler_angle().pitch, globals->pitch_motor->vel());
  globals->gimbal_controller.SetTarget(gimbal->gimbal_yaw_target_, gimbal->gimbal_pitch_target_);
  globals->gimbal_controller.Update(globals->hipnuc_imu->yaw(), globals->yaw_motor->vel(),
                                    globals->hipnuc_imu->roll(), globals->pitch_motor->vel());
  // gimbal->gravity_compensation_ = gimbal->k_gravity_compensation_ * std::cos(globals->pitch_motor->pos());
  // gimbal->pitch_torque_ = globals->gimbal_controller.output().pitch + gimbal->gravity_compensation_;
  // gimbal->pitch_torque_ = rm::modules::Clamp(pitch_torque_, -10.0f, 10.0f);
}

void Gimbal::GimbalEnableUpdate() {
  gimbal->DaMiaoMotorEnable();
  globals->gimbal_controller.Enable(true);
  if (gimbal->GimbalMove_ == kGbRemote) {
    globals->GimbalData.aim_mode = 0x00;
    globals->aim_mode = 0x00;
    gimbal->GimbalRCTargetUpdate();
    gimbal->GimbalMovePIDUpdate();
  } else if (gimbal->GimbalMove_ == kGbAimbot) {
    globals->GimbalData.aim_mode = 0x01;
    globals->aim_mode = 0x01;
    gimbal->GimbalAimbotTargetUpdate();
    gimbal->GimbalMovePIDUpdate();
  } else {
    globals->GimbalData.aim_mode = 0x00;
    globals->aim_mode = 0x00;
    globals->gimbal_controller.Enable(false);
  }
}

void Gimbal::GimbalDisableUpdate() {
  gimbal->DaMiaoMotorDisable();
  globals->gimbal_controller.Enable(false);
  globals->GimbalData.aim_mode = 0x00;
  // gimbal->gimbal_yaw_target_ = globals->ahrs.euler_angle().yaw;
  // gimbal->gimbal_pitch_target_ = globals->ahrs.euler_angle().pitch;
  gimbal->gimbal_yaw_target_ = globals->hipnuc_imu->yaw();
  gimbal->gimbal_pitch_target_ = globals->hipnuc_imu->roll();
  gimbal->GimbalMovePIDUpdate();
}

void Gimbal::DaMiaoMotorEnable() {
  if (gimbal->DM_enable_flag_ == false) {
    // 使达妙电机使能
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    globals->yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    gimbal->DM_enable_flag_ = true;
  }
}

void Gimbal::DaMiaoMotorDisable() {
  if (gimbal->DM_enable_flag_ == true) {
    // 使达妙电机失能
    globals->yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    gimbal->DM_enable_flag_ = false;
  }
}