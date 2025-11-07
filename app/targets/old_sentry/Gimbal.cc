#include "Gimbal.hpp"
void Gimbal::GimbalTask() {
    gimbal->GimbalStateUpdate();
}

void Gimbal::GimbalInit() {
    gimbal->gimbal_up_yaw_target_ = globals->up_yaw_motor->encoder();
    gimbal->gimbal_down_yaw_target_ = globals->ahrs.euler_angle().yaw;
    gimbal->gimbal_pitch_target_ = globals->pitch_motor->pos();
}

void Gimbal::GimbalStateUpdate() {
    switch (globals->StateMachine_) {
        case NO_FORCE: // 无力模式下，所有电机失能
            GimbalDisableUpdate(); // 云台电机失能计算
            AmmoDisableUpdate(); // 摩擦轮机构失能计算
            RotorDisableUpdate(); // 拨盘失能计算
            break;

        case TEST: // 测试模式下，发射系统与拨盘电机失能
            switch (gimbal->GimbalMove_) {
                case GB_REMOTE:
                    GimbalEnableUpdate(); // 云台电机使能计算
                    AmmoDisableUpdate(); // 摩擦轮机构失能计算
                    RotorDisableUpdate(); // 拨盘失能计算
                    break;

                case GB_AIMBOT:
                case GB_SCAN:
                    GimbalEnableUpdate(); // 云台电机使能计算
                    AmmoEnableUpdate(); // 摩擦轮机构使能计算
                    RotorEnableUpdate(); // 拨盘使能计算
                    break;

                default:
                    GimbalDisableUpdate(); // 云台电机失能计算
                    AmmoDisableUpdate(); // 摩擦轮机构失能计算
                    RotorDisableUpdate(); // 拨盘失能计算
                    break;
            }
            break;

        case MATCH: // 比赛模式下，所有电机正常工作
            GimbalMatchUpdate(); // 云台电机使能计算
            // AmmoEnableUpdate();                  // 摩擦轮机构使能计算
            // RotorEnableUpdate();                 // 拨盘使能计算
            break;

        default: // 错误状态，所有电机失能
            GimbalDisableUpdate(); // 云台电机失能计算
            AmmoDisableUpdate(); // 摩擦轮机构失能计算
            RotorDisableUpdate(); // 拨盘失能计算
            break;
    }
}

void Gimbal::GimbalRCTargetUpdate() {
    gimbal->gimbal_up_yaw_target_ -= rm::modules::Map(globals->rc->left_x(), -660, 660, //
                                                      -gimbal->sensitivity_up_yaw_, gimbal->sensitivity_up_yaw_);
    gimbal->gimbal_pitch_target_ += rm::modules::Map(globals->rc->left_y(), -660, 660, //
                                                     -gimbal->sensitivity_pitch_, gimbal->sensitivity_pitch_);
    gimbal->GimbalDownYawFollow();
    // gimbal->gimbal_up_yaw_target_ = rm::modules::Clamp(gimbal->gimbal_up_yaw_target_, // 上部yaw轴限位
    //                                                    gimbal->min_up_yaw_angle_, gimbal->max_up_yaw_angle_);
    gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_, // pitch轴限位
                                                      gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);
}

void Gimbal::GimbalScanTargetUpdate() {
    if (gimbal->gimbal_up_yaw_target_ >= gimbal->max_up_yaw_angle_) {
        gimbal->scan_yaw_flag_ = true;
    } else if (gimbal->gimbal_up_yaw_target_ <= gimbal->min_up_yaw_angle_) {
        gimbal->scan_yaw_flag_ = false;
    }
    if (gimbal->scan_yaw_flag_) {
        gimbal->gimbal_up_yaw_target_ -= 1.0f;
    } else {
        gimbal->gimbal_up_yaw_target_ += 1.0f;
    }
    if (gimbal->gimbal_pitch_target_ >= gimbal->highest_pitch_angle_) {
        gimbal->scan_pitch_flag_ = true;
    } else if (gimbal->gimbal_pitch_target_ <= gimbal->lowest_pitch_angle_) {
        gimbal->scan_pitch_flag_ = false;
    }
    if (gimbal->scan_pitch_flag_) {
        gimbal->gimbal_pitch_target_ -= 0.0006f;
    } else {
        gimbal->gimbal_pitch_target_ += 0.0006f;
    }
    gimbal->gimbal_down_yaw_target_ -= 0.0002f;
    gimbal->gimbal_down_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_down_yaw_target_, // 下部yaw轴周期限制
                                                        -static_cast<f32>(M_PI), M_PI);
}

void Gimbal::GimbalAimbotTargetUpdate() {
    if ((globals->Aimbot.AimbotState >> 0) & 0x01) {
        gimbal->gimbal_up_yaw_target_ = globals->Aimbot.Yaw;
        gimbal->gimbal_pitch_target_ = globals->Aimbot.Pitch;
    }
    gimbal->GimbalDownYawFollow();
}

void Gimbal::GimbalDownYawFollow() {
    if (gimbal->gimbal_up_yaw_target_ > gimbal->down_yaw_move_high_) {
        gimbal->gimbal_up_yaw_target_ = gimbal->down_yaw_move_high_;
        gimbal->gimbal_down_yaw_target_ += 0.001f;
        gimbal->down_yaw_target_refresh_flag_ = true;
    } else if (gimbal->gimbal_up_yaw_target_ < gimbal->down_yaw_move_low_) {
        gimbal->gimbal_up_yaw_target_ = gimbal->down_yaw_move_low_;
        gimbal->gimbal_down_yaw_target_ -= 0.001f;
        gimbal->down_yaw_target_refresh_flag_ = true;
    } else {
        if (gimbal->down_yaw_target_refresh_flag_) {
        gimbal->gimbal_down_yaw_target_ = globals->ahrs.euler_angle().yaw;
            gimbal->down_yaw_target_refresh_flag_ = false;
        }
    }
    gimbal->gimbal_down_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_down_yaw_target_, // 下部yaw轴周期限制
                                                    -static_cast<f32>(M_PI), M_PI);
}

void Gimbal::GimbalEnableUpdate() {
    DaMiaoMotorEnable();
    globals->gimbal_controller.Enable(true);
    // // GimbalImu.mode = 0x00;
    if (gimbal->GimbalMove_ == GB_REMOTE) {
        GimbalRCTargetUpdate();
        GimbalMovePIDUpdate();
    } else if (gimbal->GimbalMove_ == GB_SCAN) {
        GimbalScanTargetUpdate();
        GimbalMovePIDUpdate();
    } else if (gimbal->GimbalMove_ == GB_AIMBOT) {
        GimbalAimbotTargetUpdate();
        GimbalMovePIDUpdate();
    } else {
        globals->gimbal_controller.Enable(false);
    }
    auto pitch_torque = globals->gimbal_controller.output().pitch + gimbal->gravity_compensation_;
    pitch_torque = rm::modules::Clamp(pitch_torque, -10.0f, 10.0f);
    globals->up_yaw_motor->SetCurrent(static_cast<i16>(globals->gimbal_controller.output().up_yaw));
    globals->down_yaw_motor->SetPosition(0, 0, globals->gimbal_controller.output().down_yaw, 0, 0);
    globals->pitch_motor->SetPosition(0, 0, pitch_torque, 0, 0);
    rm::device::DjiMotor<>::SendCommand(); // 发送电流
}

void Gimbal::GimbalMatchUpdate() {
}

void Gimbal::GimbalDisableUpdate() {
    DaMiaoMotorDisable();
    globals->gimbal_controller.Enable(false);
    gimbal->gimbal_up_yaw_target_ = globals->up_yaw_motor->encoder();
    gimbal->gimbal_down_yaw_target_ = globals->ahrs.euler_angle().yaw;
    gimbal->gimbal_pitch_target_ = globals->pitch_motor->pos();
    gimbal->gravity_compensation_ = 0.0f;
    globals->up_yaw_motor->SetCurrent(0);
    globals->down_yaw_motor->SetPosition(0, 0, 0, 0, 0);
    globals->pitch_motor->SetPosition(0, 0, 0, 0, 0);
    rm::device::DjiMotor<>::SendCommand(); // 发送电流
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

void Gimbal::GimbalMovePIDUpdate() {
    globals->gimbal_controller.SetTarget(gimbal->gimbal_up_yaw_target_, gimbal->gimbal_down_yaw_target_, //
                                         gimbal->gimbal_pitch_target_);
    globals->gimbal_controller.Update(globals->up_yaw_motor->encoder(), globals->up_yaw_motor->rpm(),
                                      globals->ahrs.euler_angle().yaw, globals->down_yaw_motor->vel(),
                                      globals->pitch_motor->pos(), globals->pitch_motor->vel());
    gimbal->gravity_compensation_ = gimbal->k_gravity_compensation_ * std::cos(globals->pitch_motor->pos());
}

void Gimbal::AmmoEnableUpdate() {
}

void Gimbal::AmmoDisableUpdate() {
}

void Gimbal::RotorEnableUpdate() {
}

void Gimbal::RotorDisableUpdate() {
}
