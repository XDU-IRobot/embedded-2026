#include "dart_statemachine.hpp"
#include <math.h>

void DartStateMachineUpdate(DartState &state) {
    // 根据遥控器左拨杆位置设置状态
    if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kDown) {
        // 左拨杆向下，无力状态
        state.unable = AbleState::kOn;
        state.manual_mode.enabled = AbleState::kOff;
        state.auto_mode.enabled = AbleState::kOff;
        state.adjust_mode.enabled = AbleState::kOff;
    } else if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kMid) {
        // 左拨杆向中，手动模式
        state.unable = AbleState::kOff;
        state.manual_mode.enabled = AbleState::kOn;
        state.auto_mode.enabled = AbleState::kOff;
        state.adjust_mode.enabled = AbleState::kOff;
    } else if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kUp &&
               dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kUp) {
        // 左右拨杆向上，自动模式
        state.unable = AbleState::kOff;
        state.manual_mode.enabled = AbleState::kOff;
        state.auto_mode.enabled = AbleState::kOn;
        state.adjust_mode.enabled = AbleState::kOff;
    } else if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kUp &&
               dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kMid) {
        state.unable = AbleState::kOff;
        state.manual_mode.enabled = AbleState::kOff;
        state.auto_mode.enabled = AbleState::kOff;
        state.adjust_mode.enabled = AbleState::kOn;
    }
    // 状态机处理逻辑
    if (state.unable == AbleState::kOn) {
        DartStateClear(state);
        DartStateUnableUpdate();
        return;
    } else if (state.manual_mode.enabled == AbleState::kOn) {
        DartStateManualUpdate();
    } else if (state.auto_mode.enabled == AbleState::kOn) {
    } else if (state.adjust_mode.enabled == AbleState::kOn) {
        DartStateAdjustUpdate();
    } else {
        DartStateClear(state);
    }
}

// 手动模式状态机处理
void DartStateManualUpdate() {
    switch (dart_rack->state_.manual_mode.mode) {
        case ModeState::kUnable:
            // 等待进入初始化阶段
            if (dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kUp) {
                dart_rack->state_.manual_mode.mode = ModeState::kInit;
            } else {
                DartStateUnableUpdate();
            }
            break;
        case ModeState::kInit:
            // 初始化逻辑
            if (dart_rack->state_.manual_mode.init == PhaseState::kUncomplete) {
                DartStateInitUpdate();
            } else if (dart_rack->state_.manual_mode.init == PhaseState::kDone) {
                // 初始化完成，进入下一个阶段
                if (dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kDown) {
                    dart_rack->state_.manual_mode.mode = ModeState::kload;
                }
            }
            break;
        case ModeState::kload:
            if (dart_rack->state_.manual_mode.load == PhaseState::kUncomplete) {
                DartStateLoadUpdate(); //... 装填操作
            } else if (dart_rack->state_.manual_mode.load == PhaseState::kDone) {
                if (dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kMid) {
                    dart_rack->state_.manual_mode.mode = ModeState::kAdd;
                }
            }
            // 装填逻辑
            break;

        case ModeState::kAdd:
            if (dart_rack->state_.manual_mode.add == PhaseState::kUncomplete) {
                DartStateAddUpdate();
            } else if (dart_rack->state_.manual_mode.add == PhaseState::kDone) {
                // 加弹完成，进入下一个阶段
                if (dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kMid) {
                    dart_rack->state_.manual_mode.mode = ModeState::kAim;
                }
            }
            break;

        case ModeState::kAim:
            if (dart_rack->state_.manual_mode.aim == PhaseState::kUncomplete) {
                DartStateAimUpdate();
            } else if (dart_rack->state_.manual_mode.aim == PhaseState::kDone) {
                // 瞄准完成，进入下一个阶段
                dart_rack->state_.manual_mode.mode = ModeState::kFire;
            }
            break;
        case ModeState::kFire:
            // 发射逻辑
            if (dart_rack->state_.manual_mode.fire == PhaseState::kUncomplete && dart_rack->rc_->left_x() == 660 &&
                dart_rack->rc_->right_x() == -660) {
                DartStateFireUpdate();
            } else if (dart_rack->state_.manual_mode.fire == PhaseState::kDone) {
                // 发射完成，返回待机状态
                dart_rack->state_.manual_mode.mode = ModeState::kUnable;
                dart_rack->dart_count_ = static_cast<DartCount>(static_cast<uint8_t>(dart_rack->dart_count_) + 1);
                DartManualModeClear(dart_rack->state_.manual_mode);
            }
            break;
        default:
            break;
    }
}

void DartStateAdjustUpdate() {
    // Yaw轴调节
    if (dart_rack->rc_->right_x() > 330) {
        if (dart_rack->yaw_encoder_->angle_deg() <= DartRack::kYawEcdMin) {
            dart_rack->yaw_motor_->SetCurrent(0);
        } else {
            dart_rack->yaw_motor_speed_pid_.Update(-2000.0f, dart_rack->yaw_motor_->rpm(), 1.0f);
            dart_rack->yaw_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->yaw_motor_speed_pid_.out()));
        }
    } else if (dart_rack->rc_->right_x() < -330) {
        if (dart_rack->yaw_encoder_->angle_deg() >= DartRack::kYawEcdMax) {
            dart_rack->yaw_motor_->SetCurrent(0);
        } else {
            dart_rack->yaw_motor_speed_pid_.Update(2000.0f, dart_rack->yaw_motor_->rpm(), 1.0f);
            dart_rack->yaw_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->yaw_motor_speed_pid_.out()));
        }
    } else {
        dart_rack->yaw_motor_->SetCurrent(0);
    }
    // 扳机位置调节
    if (dart_rack->rc_->right_y() > 330) {
        if (dart_rack->trigger_motor_odometer_.stall_time() <= 100) {
            dart_rack->trigger_motor_speed_pid_.Update(-8000.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
            dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
        } else {
            dart_rack->trigger_motor_->SetCurrent(0);
        }
    } else if (dart_rack->rc_->right_y() < -330) {
        if (dart_rack->trigger_motor_odometer_.stall_time() <= 100) {
            dart_rack->trigger_motor_speed_pid_.Update(8000.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
            dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
        } else {
            dart_rack->trigger_motor_->SetCurrent(0);
        }
    } else {
        dart_rack->trigger_motor_speed_pid_.Update(.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
        dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
    }
    // 上膛调节
    if (dart_rack->rc_->left_y() > 330) {
        dart_rack->add_plate_servo_->SetServoAngle(
593, 0, 0);
        if (dart_rack->load_motor_l_odometer_.stall_time() <= 100 &&
            dart_rack->load_motor_r_odometer_.stall_time() <= 100) {
            dart_rack->load_motor_l_speed_pid_.Update(3000.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
            dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
            dart_rack->load_motor_r_speed_pid_.Update(-3000.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
            dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
        } else {
            dart_rack->load_motor_l_speed_pid_.Update(.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
            dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
            dart_rack->load_motor_r_speed_pid_.Update(.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
            dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
        }
    } else if (dart_rack->rc_->left_y() < -330) {
        dart_rack->add_plate_servo_->SetServoAngle(
    204, 1, 0);

        if (dart_rack->load_motor_l_odometer_.stall_time() <= 100 &&
            dart_rack->load_motor_r_odometer_.stall_time() <= 100) {
            dart_rack->load_motor_l_speed_pid_.Update(-3000.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
            dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
            dart_rack->load_motor_r_speed_pid_.Update(3000.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
            dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
        } else {
            dart_rack->load_motor_l_speed_pid_.Update(.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
            dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
            dart_rack->load_motor_r_speed_pid_.Update(.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
            dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
        }
    } else {
        dart_rack->add_plate_servo_->SetServoAngle(
    214, 2, 0);

        dart_rack->load_motor_l_speed_pid_.Update(0.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
        dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
        dart_rack->load_motor_r_speed_pid_.Update(0.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
        dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
    }
    // 扳机触发调节
    if (dart_rack->rc_->left_x() > 330) {
        if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 100) {
            if (dart_rack->trigger_motor_force_->encoder() <= 8000) {
                dart_rack->trigger_motor_force_pid_.Update(1000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
                dart_rack->trigger_motor_force_->SetCurrent(
                    static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
            } else {
                dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
                dart_rack->trigger_motor_force_->SetCurrent(
                    static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
            }
        } else {
            dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
            dart_rack->trigger_motor_force_->SetCurrent(
                static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
        }
    } else if (dart_rack->rc_->left_x() < -330) {
        if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 100) {
            if (dart_rack->trigger_motor_force_->encoder() >= 5000) {
                dart_rack->trigger_motor_force_pid_.Update(-1000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
                dart_rack->trigger_motor_force_->SetCurrent(
                    static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
            } else {
                dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
                dart_rack->trigger_motor_force_->SetCurrent(
                    static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
            }
        } else {
            dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
            dart_rack->trigger_motor_force_->SetCurrent(
                static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
        }
    } else {
        dart_rack->trigger_motor_force_->SetCurrent(0);
    }
    if (dart_rack->rc_->dial() < -330) {
        dart_rack->add_motor_speed_pid_.Update(300.0f, dart_rack->add_motor_->rpm(), 1.0f);
        dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
        dart_rack->add_plate_servo_->SetServoAngle(866, 0, 0);
    } else if (dart_rack->rc_->dial() > 330) {
        dart_rack->add_motor_speed_pid_.Update(-300.0f, dart_rack->add_motor_->rpm(), 1.0f);
        dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
        dart_rack->add_plate_servo_->SetServoAngle(593, 0, 0);
    } else {
        dart_rack->add_motor_speed_pid_.Update(.0f, dart_rack->add_motor_->rpm(), 1.0f);
        dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
    }
}

void DartStateUnableUpdate() {
    dart_rack->yaw_motor_->SetCurrent(0);
    dart_rack->load_motor_l_->SetCurrent(0);
    dart_rack->load_motor_r_->SetCurrent(0);
    dart_rack->trigger_motor_->SetCurrent(0);
    dart_rack->trigger_motor_force_->SetCurrent(0);
    dart_rack->add_motor_->SetCurrent(0);
}

void DartStateInitUpdate() {
    // Yaw轴根据是第几发镖初始化
    if (dart_rack->yaw_encoder_->angle_deg() - DartRack::kYawEcd[static_cast<uint8_t>(dart_rack->dart_count_)] > 0.05) {
        dart_rack->yaw_motor_speed_pid_.Update(-1000.0f, dart_rack->yaw_motor_->rpm(), 1.0f);
        dart_rack->yaw_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->yaw_motor_speed_pid_.out()));
    } else if (dart_rack->yaw_encoder_->angle_deg() - DartRack::kYawEcd[static_cast<uint8_t>(dart_rack->dart_count_)] <
               -0.05) {
        dart_rack->yaw_motor_speed_pid_.Update(1000.0f, dart_rack->yaw_motor_->rpm(), 1.0f);
        dart_rack->yaw_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->yaw_motor_speed_pid_.out()));
    } else {
        dart_rack->yaw_motor_->SetCurrent(0);
        dart_rack->state_.manual_mode.is_yaw_init_done = true;
    }

    // 如果是第一发镖，首先全部转到限位并清除计圈器
    // 上膛电机初始化
    if (dart_rack->dart_count_ == DartCount::kFirst && dart_rack->state_.manual_mode.is_load_reset_done == false) {
        // if (dart_rack->load_motor_l_odometer_.stall_time() <= 100 ||
        //     dart_rack->load_motor_r_odometer_.stall_time() <= 100) {
        //   dart_rack->load_motor_l_speed_pid_.Update(3000.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
        //   dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
        //   dart_rack->load_motor_r_speed_pid_.Update(-3000.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
        //   dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
        // } else {
        //   dart_rack->load_motor_l_speed_pid_.Update(.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
        //   dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
        //   dart_rack->load_motor_r_speed_pid_.Update(.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
        //   dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
        //   dart_rack->state_.manual_mode.is_load_reset_done = true;
        //   dart_rack->load_motor_l_odometer_.Reset();
        //   dart_rack->load_motor_r_odometer_.Reset();
        // }
        //直接清除计圈器
        dart_rack->load_motor_l_speed_pid_.Update(.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
        dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
        dart_rack->load_motor_r_speed_pid_.Update(.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
        dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
        dart_rack->state_.manual_mode.is_load_reset_done = true;
        dart_rack->load_motor_l_odometer_.Reset();
        dart_rack->load_motor_r_odometer_.Reset();
        // // 扳机位置初始化
        // if (dart_rack->trigger_motor_odometer_.stall_time() <= 100 &&
        //     dart_rack->state_.manual_mode.is_trigger_reset_done == false) {
        //     dart_rack->trigger_motor_speed_pid_.Update(-8000.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
        //     dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
        // } else {
        //     dart_rack->trigger_motor_->SetCurrent(0);
        //     dart_rack->state_.manual_mode.is_trigger_reset_done = true;
        //     dart_rack->trigger_motor_odometer_.Reset();
        // }
        dart_rack->state_.manual_mode.is_trigger_reset_done = true;
        dart_rack->trigger_motor_odometer_.Reset();
    } else {
        dart_rack->load_motor_l_speed_pid_.Update(.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
        dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
        dart_rack->load_motor_r_speed_pid_.Update(.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
        dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
        dart_rack->state_.manual_mode.is_load_reset_done = true;
        dart_rack->state_.manual_mode.is_trigger_reset_done = true;
    }

    // // 根据当前发镖数确定扳机位置
    // if ( dart_rack->state_.manual_mode.is_trigger_reset_done
    //     == false) {
    //     if (dart_rack->trigger_motor_odometer_.linear_ticks() >
    //         DartRack::kTriggerEcd[static_cast<uint8_t>(dart_rack->dart_count_)]) {
    //         dart_rack->trigger_motor_speed_pid_.Update(-1000.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
    //         dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
    //     } else if (dart_rack->trigger_motor_odometer_.linear_ticks() <
    //                DartRack::kTriggerEcd[static_cast<uint8_t>(dart_rack->dart_count_)]) {
    //         dart_rack->trigger_motor_speed_pid_.Update(1000.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
    //         dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
    //     } else {
    //         dart_rack->trigger_motor_speed_pid_.Update(.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
    //         dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
    //         dart_rack->state_.manual_mode.is_trigger_init_done = true;
    //     }
    // }
    // 根据扳机位置计算滑台里程


    //  让换弹电机撞限位
    // if (dart_rack->state_.manual_mode.is_add_init_done == false) {
    //     if (dart_rack->add_motor_odometer_.stall_time() <= 100) {
    //         dart_rack->add_motor_speed_pid_.Update(300.0f, dart_rack->add_motor_->rpm(), 1.0f);
    //         dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
    //     } else {
    //         dart_rack->add_motor_speed_pid_.Update(.0f, dart_rack->add_motor_->rpm(), 1.0f);
    //         dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
    //         dart_rack->state_.manual_mode.is_add_init_done = true;
    //         dart_rack->add_motor_odometer_.Reset();
    //     }
    // }
    dart_rack->state_.manual_mode.is_add_init_done = true;
    // 打开撒放器
    if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 50 &&
        dart_rack->state_.manual_mode.is_trigger_force_init_done == false) {
        dart_rack->trigger_motor_force_pid_.Update(1000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
        dart_rack->trigger_motor_force_->SetCurrent(
            static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
    } else {
        dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
        dart_rack->trigger_motor_force_->SetCurrent(
            static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
        dart_rack->state_.manual_mode.is_trigger_force_init_done = true;
    }
    // 全部检查完成后，初始化完成
    if (dart_rack->state_.manual_mode.is_yaw_init_done == true &&
        dart_rack->state_.manual_mode.is_load_reset_done == true &&
        dart_rack->state_.manual_mode.is_trigger_reset_done == true &&
        dart_rack->state_.manual_mode.is_trigger_force_init_done == true && dart_rack->state_.manual_mode.
        is_add_init_done == true) {
        dart_rack->state_.manual_mode.init = PhaseState::kDone;
        dart_rack->yaw_motor_speed_pid_.Update(.0f, dart_rack->yaw_motor_->rpm(), 1.0f);
        dart_rack->yaw_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->yaw_motor_speed_pid_.out()));
        dart_rack->load_motor_l_speed_pid_.Update(0.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
        dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
        dart_rack->load_motor_r_speed_pid_.Update(0.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
        dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
        dart_rack->trigger_motor_speed_pid_.Update(.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
        dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
        dart_rack->trigger_motor_force_pid_.Update(.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
        dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
        dart_rack->add_motor_speed_pid_.Update(.0f, dart_rack->add_motor_->rpm(), 1.0f);
        dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
    }
}

void DartStateLoadUpdate() {
    // 上膛逻辑
    // 滑台下拉
    if (dart_rack->load_motor_l_odometer_.stall_time() <= 100 &&
        dart_rack->load_motor_r_odometer_.stall_time() <= 100 && dart_rack->state_.manual_mode.is_load_down_done ==
        false) {
        if (dart_rack->load_motor_r_odometer_.linear_ticks()>DartRack::kTriggerEcdMax||dart_rack->load_motor_l_odometer_.linear_ticks()<-DartRack::kTriggerEcdMax) {

            dart_rack->load_motor_l_speed_pid_.Update(-1500.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
            dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
            dart_rack->load_motor_r_speed_pid_.Update(1500.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
            dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
        }
        else {

            dart_rack->load_motor_l_speed_pid_.Update(-4000.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
            dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
            dart_rack->load_motor_r_speed_pid_.Update(4000.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
            dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));

        }





    } else {
        dart_rack->load_motor_l_speed_pid_.Update(.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
        dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
        dart_rack->load_motor_r_speed_pid_.Update(.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
        dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
        dart_rack->state_.manual_mode.is_load_down_done = true;
    }

    // 撒放器锁定
    if (dart_rack->state_.manual_mode.is_load_down_done == true) {
        if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 100 &&
            dart_rack->state_.manual_mode.is_trigger_lock_done == false) {
            dart_rack->trigger_motor_force_pid_.Update(-1000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
            dart_rack->trigger_motor_force_->SetCurrent(
                static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
        } else {
            dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
            dart_rack->trigger_motor_force_->SetCurrent(
                static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
            dart_rack->state_.manual_mode.is_trigger_lock_done = true;
        }
    } else {
        dart_rack->trigger_motor_force_pid_.Update(0.f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
        dart_rack->trigger_motor_force_->SetCurrent(
            static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
    }
    if (dart_rack->state_.manual_mode.is_trigger_lock_done == true) {
        // 滑台还原
        if (dart_rack->load_motor_l_odometer_.linear_ticks() <= 0 ||
            dart_rack->load_motor_r_odometer_.linear_ticks() >= 0 ||
            dart_rack->state_.manual_mode.is_load_reset_done == false) {
            if (dart_rack->load_motor_l_odometer_.stall_time() <= 100 &&
                dart_rack->load_motor_r_odometer_.stall_time() <= 100) {
                dart_rack->load_motor_l_speed_pid_.Update(4000.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
                dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
                dart_rack->load_motor_r_speed_pid_.Update(-4000.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
                dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
            } else {
                dart_rack->load_motor_l_speed_pid_.Update(.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
                dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
                dart_rack->load_motor_r_speed_pid_.Update(.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
                dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
            }
        } else if (abs(dart_rack->load_motor_l_odometer_.linear_ticks() +
                       dart_rack->load_motor_r_odometer_.linear_ticks()) >= 100000) {
            if (dart_rack->load_motor_l_odometer_.linear_ticks() >= 0) {
                dart_rack->load_motor_l_speed_pid_.Update(-3000.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
                dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
            } else if (dart_rack->load_motor_r_odometer_.linear_ticks() <= 0) {
                dart_rack->load_motor_r_speed_pid_.Update(3000.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
                dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
            } else {
                dart_rack->load_motor_l_speed_pid_.Update(.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
                dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
                dart_rack->load_motor_r_speed_pid_.Update(.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
                dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
            }
        } else {
            dart_rack->state_.manual_mode.is_load_up_done = true;
            dart_rack->load_motor_l_speed_pid_.Update(.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
            dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
            dart_rack->load_motor_r_speed_pid_.Update(.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
            dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
        }
    }
    if (dart_rack->state_.manual_mode.is_trigger_lock_done == true &&
        dart_rack->state_.manual_mode.is_load_up_done == true &&
        dart_rack->state_.manual_mode.is_load_down_done == true) {
        dart_rack->state_.manual_mode.load = PhaseState::kDone;

        dart_rack->load_motor_l_->SetCurrent(0);

        dart_rack->load_motor_r_->SetCurrent(0);
    }
}

void DartStateAddUpdate() {
    // 加弹逻辑
    if (dart_rack->dart_count_ == DartCount::kFirst) {
        dart_rack->state_.manual_mode.add = PhaseState::kDone; // 第一发不用换弹
    }

    if (dart_rack->state_.manual_mode.add == PhaseState::kUncomplete) {
        const auto add_index = static_cast<uint8_t>(dart_rack->dart_count_) - 1;
        const auto target_ticks = DartRack::kAddEcd[add_index];
        if (dart_rack->state_.manual_mode.is_add_down_done == false) {
            if (dart_rack->state_.manual_mode.is_add_down_done == false &&
                dart_rack->add_motor_odometer_.linear_ticks() > target_ticks) {
                dart_rack->add_motor_speed_pid_.Update(-300.0f, dart_rack->add_motor_->rpm(), 1.0f);
                dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
                } else if (dart_rack->state_.manual_mode.is_add_down_done == false &&
                           dart_rack->add_motor_odometer_.linear_ticks() <= target_ticks) {
                    dart_rack->state_.manual_mode.is_add_down_done = true;
                    dart_rack->add_motor_speed_pid_.Update(.0f, dart_rack->add_motor_->rpm(), 1.0f);
                    dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
                           } else {
                               dart_rack->add_motor_speed_pid_.Update(.0f, dart_rack->add_motor_->rpm(), 1.0f);
                               dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
                           }
        }
        if (dart_rack->state_.manual_mode.is_add_down_done == true && dart_rack->state_.manual_mode.is_add_plate_done ==
            false) {
            if (dart_rack->ticks <= 1000) {
                dart_rack->add_plate_servo_->SetServoAngle(
                    DartRack::kAddPlateUnlockEcd[add_index], add_index, 0);
                dart_rack->ticks++;
                dart_rack->add_motor_speed_pid_.Update(.0f, dart_rack->add_motor_->rpm(), 1.0f);
                dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
            } else {
                dart_rack->state_.manual_mode.is_add_plate_done = true;
                dart_rack->ticks = 0;
                dart_rack->add_motor_speed_pid_.Update(.0f, dart_rack->add_motor_->rpm(), 1.0f);
                dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
            }
        }
        if (dart_rack->state_.manual_mode.is_add_plate_done == true && dart_rack->state_.manual_mode.is_add_up_done ==
            false) {
            if (dart_rack->add_motor_odometer_.linear_ticks() < 0&&dart_rack->add_motor_odometer_.stall_time()<=100) {
                dart_rack->add_motor_speed_pid_.Update(300.0f, dart_rack->add_motor_->rpm(), 1.0f);
                dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
            } else {
                dart_rack->state_.manual_mode.is_add_up_done = true;
                dart_rack->add_motor_speed_pid_.Update(.0f, dart_rack->add_motor_->rpm(), 1.0f);
                dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
                // dart_rack->add_plate_servo_->SetServoAngle(
                //     DartRack::[add_index], add_index, 0);
            }
        }
        if (dart_rack->state_.manual_mode.is_add_plate_done == true && dart_rack->state_.manual_mode.is_add_up_done ==
            true && dart_rack->state_.manual_mode.is_add_down_done == true) {
            dart_rack->state_.manual_mode.add = PhaseState::kDone;
        }
    }
    if (dart_rack->state_.manual_mode.add == PhaseState::kDone) {
        dart_rack->add_motor_->SetCurrent(0);
    }
}

void DartStateAimUpdate() {
    dart_rack->state_.manual_mode.aim = PhaseState::kDone; // 瞄准待实现
}

void DartStateFireUpdate() {
    // 释放扳机即可
    if (dart_rack->state_.manual_mode.fire == PhaseState::kUncomplete) {
        if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 100) {
            dart_rack->trigger_motor_force_pid_.Update(1000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
            dart_rack->trigger_motor_force_->SetCurrent(
                static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
        } else {
            dart_rack->trigger_motor_force_->SetCurrent(0);
            dart_rack->state_.manual_mode.fire = PhaseState::kDone;
        }
    } else {
        dart_rack->trigger_motor_force_->SetCurrent(0);
        dart_rack->state_.manual_mode.fire = PhaseState::kDone;
    }
}
