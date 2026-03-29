#include "dart_statemachine.hpp"
#include <cmath>

int32_t debug_trigger_force_encoder = 0;
uint32_t debug_trigger_force_stall_time = 0;
uint32_t debug_adjust_motor_running_time = 0;

void DartStateMachineUpdate(DartState &state) {
  // 更新用于 FreeMASTER 观测的调试变量
  debug_trigger_force_encoder = dart_rack->trigger_motor_force_->encoder();
  debug_trigger_force_stall_time = dart_rack->trigger_motor_force_odometer_.stall_time();

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
  static bool wait_for_release = false; // 用于防止长按遥控器导致的连续触发下一发

  switch (dart_rack->state_.manual_mode.mode) {
    case ModeState::kUnable:
      // 如果遥控器松开（不在kUp），则解除锁定
      if (dart_rack->rc_->switch_r() != rm::device::DR16::SwitchPosition::kUp) {
        wait_for_release = false;
      }

      // 等待进入初始化阶段：必须在解除锁定后，再次拨到 kUp 才开始下一发
      if (!wait_for_release && dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kUp) {
        dart_rack->state_.manual_mode.mode = ModeState::kInit;
        wait_for_release = true; // 锁定，直到下一次松开
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
        dart_rack->state_.manual_mode.mode = ModeState::kload;
      }
      break;
    case ModeState::kload:
      if (dart_rack->state_.manual_mode.load == PhaseState::kUncomplete) {
        DartStateLoadUpdate();  //... 装填操作
      } else if (dart_rack->state_.manual_mode.load == PhaseState::kDone) {
        dart_rack->state_.manual_mode.mode = ModeState::kAdd;
      }
      // 装填逻辑
      break;

    case ModeState::kAdd:
      if (dart_rack->state_.manual_mode.add == PhaseState::kUncomplete) {
        DartStateAddUpdate();
      } else if (dart_rack->state_.manual_mode.add == PhaseState::kDone) {
        // 加弹完成，进入下一个阶段
        dart_rack->state_.manual_mode.mode = ModeState::kAim;
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
      if (dart_rack->state_.manual_mode.fire == PhaseState::kUncomplete) {
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
  if (!dart_rack->state_.manual_mode.is_yaw_init_done) {
    float yaw_target = DartRack::kYawEcd[static_cast<uint8_t>(dart_rack->dart_count_)];
    float yaw_current = dart_rack->yaw_encoder_->angle_deg();
    float yaw_error = yaw_target - yaw_current;

    // 误差大于1度时继续进行调节
    if (std::abs(yaw_error) > 1.0f) {
      // 降低P系数，让运动更柔和平缓，防止冲过头
      float target_speed = yaw_error * 150.0f;

      // 限制最大旋转速度
      if (target_speed > 800.0f) target_speed = 800.0f;
      else if (target_speed < -800.0f) target_speed = -800.0f;

      // 设置最低起步指令克服静摩擦力
      if (target_speed > 0.0f && target_speed < 80.0f) target_speed = 80.0f;
      else if (target_speed < 0.0f && target_speed > -80.0f) target_speed = -80.0f;

      dart_rack->yaw_motor_speed_pid_.Update(target_speed, dart_rack->yaw_motor_->rpm(), 1.0f);
      dart_rack->yaw_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->yaw_motor_speed_pid_.out()));
    } else {
      // 到达目标范围后，直接断电清零防止由刹车引起的反转/抽搔
      dart_rack->yaw_motor_speed_pid_.Update(0.0f, dart_rack->yaw_motor_->rpm(), 1.0f);
      dart_rack->yaw_motor_->SetCurrent(0);
      dart_rack->state_.manual_mode.is_yaw_init_done = true;
    }
  } else {
    // 已经满足过一次条件就将其锁在停止状态，避免云台惯性越界导致反复抽搐判断
    dart_rack->yaw_motor_->SetCurrent(0);
  }

  // 如果是第一发镖，首先全部转到限位并清除计圈器
  // 上膛电机初始化
  if (dart_rack->dart_count_ == DartCount::kFirst && dart_rack->state_.manual_mode.is_load_reset_done == false) {
    // 直接清除计圈器
    dart_rack->load_motor_l_speed_pid_.Update(.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
    dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
    dart_rack->load_motor_r_speed_pid_.Update(.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
    dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
    dart_rack->state_.manual_mode.is_load_reset_done = true;
    dart_rack->load_motor_l_odometer_.Reset();
    dart_rack->load_motor_r_odometer_.Reset();

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

  dart_rack->state_.manual_mode.is_add_init_done = true;

  // 全部检查完成后，初始化完成 (移除了 is_trigger_force_init_done，将其放到 load 中执行)
  if (dart_rack->state_.manual_mode.is_yaw_init_done == true &&
      dart_rack->state_.manual_mode.is_load_reset_done == true &&
      dart_rack->state_.manual_mode.is_trigger_reset_done == true &&
      dart_rack->state_.manual_mode.is_add_init_done == true) {
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
  bool target_trigger_active = false;
  float target_trigger_speed = 0.0f;

  bool target_load_active = false;
  float target_load_speed_l = 0.0f;
  float target_load_speed_r = 0.0f;

  // Phase 1: 打开撒放器 + 滑台下拉，同时进行
  if (dart_rack->state_.manual_mode.is_trigger_force_init_done == false) {
    if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 50) {
      target_trigger_active = true;
      target_trigger_speed = 1000.0f;
    } else {
      dart_rack->state_.manual_mode.is_trigger_force_init_done = true;
    }
  }

  if (dart_rack->state_.manual_mode.is_load_down_done == false) {
    if (dart_rack->load_motor_l_odometer_.stall_time() <= 100 && dart_rack->load_motor_r_odometer_.stall_time() <= 100) {
      target_load_active = true;
      if (dart_rack->load_motor_r_odometer_.linear_ticks() > DartRack::kTriggerEcdMax ||
          dart_rack->load_motor_l_odometer_.linear_ticks() < -DartRack::kTriggerEcdMax) {
        target_load_speed_l = -1500.0f;
        target_load_speed_r = 1500.0f;
      } else {
        target_load_speed_l = -3000.0f;
        target_load_speed_r = 3000.0f;
      }
    } else {
      dart_rack->state_.manual_mode.is_load_down_done = true;
    }
  }

  // Phase 2 & 3: 两者均完成后，依次锁定撒放器并上拉还原
  if (dart_rack->state_.manual_mode.is_load_down_done == true &&
      dart_rack->state_.manual_mode.is_trigger_force_init_done == true) {

    // Phase 2: 锁定撒放器
    if (dart_rack->state_.manual_mode.is_trigger_lock_done == false) {
      if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 100) {
        target_trigger_active = true;
        target_trigger_speed = -1000.0f;
      } else {
        dart_rack->state_.manual_mode.is_trigger_lock_done = true;
      }
    }
    // Phase 3: 滑台还原
    else if (dart_rack->state_.manual_mode.is_load_up_done == false) {
      if (dart_rack->load_motor_l_odometer_.linear_ticks() <= 0 ||
          dart_rack->load_motor_r_odometer_.linear_ticks() >= 0 ||
          dart_rack->state_.manual_mode.is_load_reset_done == false) {

        if (dart_rack->load_motor_l_odometer_.stall_time() <= 100 &&
            dart_rack->load_motor_r_odometer_.stall_time() <= 100) {
          target_load_active = true;
          target_load_speed_l = 3000.0f;
          target_load_speed_r = -3000.0f;
        }
      } else if (abs(dart_rack->load_motor_l_odometer_.linear_ticks() +
                     dart_rack->load_motor_r_odometer_.linear_ticks()) >= 100000) {
        if (dart_rack->load_motor_l_odometer_.linear_ticks() >= 0) {
          target_load_active = true;
          target_load_speed_l = -3000.0f;
          target_load_speed_r = 0.0f;
        } else if (dart_rack->load_motor_r_odometer_.linear_ticks() <= 0) {
          target_load_active = true;
          target_load_speed_l = 0.0f;
          target_load_speed_r = 3000.0f;
        }
      } else {
        dart_rack->state_.manual_mode.is_load_up_done = true;
      }
    }
  }

  // 统一输出电机指令，避免函数内发生覆盖
  if (target_trigger_active) {
    dart_rack->trigger_motor_force_pid_.Update(target_trigger_speed, dart_rack->trigger_motor_force_->rpm(), 1.0f);
    dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
  } else {
    dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
    dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
  }

  if (target_load_active) {
    dart_rack->load_motor_l_speed_pid_.Update(target_load_speed_l, dart_rack->load_motor_l_->rpm(), 1.0f);
    dart_rack->load_motor_r_speed_pid_.Update(target_load_speed_r, dart_rack->load_motor_r_->rpm(), 1.0f);
    dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
    dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
  } else {
    dart_rack->load_motor_l_speed_pid_.Update(0.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
    dart_rack->load_motor_r_speed_pid_.Update(0.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
    dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
    dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
  }

  if (dart_rack->state_.manual_mode.is_trigger_lock_done == true &&
      dart_rack->state_.manual_mode.is_load_up_done == true &&
      dart_rack->state_.manual_mode.is_load_down_done == true &&
      dart_rack->state_.manual_mode.is_trigger_force_init_done == true) {
    dart_rack->state_.manual_mode.load = PhaseState::kDone;
  }
}

void DartStateAddUpdate() {
  // 加弹逻辑
  if (dart_rack->dart_count_ == DartCount::kFirst) {
    dart_rack->state_.manual_mode.add = PhaseState::kDone;  // 第一发不用换弹
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
    if (dart_rack->state_.manual_mode.is_add_down_done == true &&
        dart_rack->state_.manual_mode.is_add_plate_done == false) {
      if (dart_rack->ticks <= 1000) {
        dart_rack->add_plate_servo_->SetServoAngle(DartRack::kAddPlateUnlockEcd[add_index], add_index, 0);
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
    if (dart_rack->state_.manual_mode.is_add_plate_done == true &&
        dart_rack->state_.manual_mode.is_add_up_done == false) {
      if (dart_rack->add_motor_odometer_.linear_ticks() < 0 && dart_rack->add_motor_odometer_.stall_time() <= 100) {
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
    if (dart_rack->state_.manual_mode.is_add_plate_done == true &&
        dart_rack->state_.manual_mode.is_add_up_done == true &&
        dart_rack->state_.manual_mode.is_add_down_done == true) {
      dart_rack->state_.manual_mode.add = PhaseState::kDone;
    }
  }
  if (dart_rack->state_.manual_mode.add == PhaseState::kDone) {
    dart_rack->add_motor_->SetCurrent(0);
  }
}

void DartStateAimUpdate() {
  dart_rack->state_.manual_mode.aim = PhaseState::kDone;  // 瞄准待实现
}

void DartStateFireUpdate() {
  static uint32_t fire_running_time = 0;
  // 释放扳机即可
  if (dart_rack->state_.manual_mode.fire == PhaseState::kUncomplete) {
    // 恢复使用测量的 running_time 防止堵转保护电机
    if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 100 && fire_running_time < 150) {
      dart_rack->trigger_motor_force_pid_.Update(1000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
      fire_running_time++;
    } else {
      dart_rack->trigger_motor_force_->SetCurrent(0);
      dart_rack->state_.manual_mode.fire = PhaseState::kDone;
      fire_running_time = 0;
    }
  } else {
    dart_rack->trigger_motor_force_->SetCurrent(0);
    dart_rack->state_.manual_mode.fire = PhaseState::kDone;
    fire_running_time = 0;
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
    dart_rack->add_plate_servo_->SetServoAngle(593, 0, 0);
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
    dart_rack->add_plate_servo_->SetServoAngle(204, 1, 0);

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
    dart_rack->add_plate_servo_->SetServoAngle(214, 2, 0);

    dart_rack->load_motor_l_speed_pid_.Update(0.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
    dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
    dart_rack->load_motor_r_speed_pid_.Update(0.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
    dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
  }
  // 扳机触发调节
  if (dart_rack->rc_->left_x() > 330) {
    debug_adjust_motor_running_time++;
    if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 100) {
      if (dart_rack->trigger_motor_force_->encoder() <= 8000) {
        dart_rack->trigger_motor_force_pid_.Update(1000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
        dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
      } else {
        dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
        dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
      }
    } else {
      dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
    }
  } else if (dart_rack->rc_->left_x() < -330) {
    debug_adjust_motor_running_time++;
    if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 100) {
      if (dart_rack->trigger_motor_force_->encoder() >= 5000) {
        dart_rack->trigger_motor_force_pid_.Update(-1000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
        dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
      } else {
        dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
        dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
      }
    } else {
      dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
    }
  } else {
    debug_adjust_motor_running_time = 0;
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