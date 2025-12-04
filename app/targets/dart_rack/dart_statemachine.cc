//
// Created by 34236 on 2025/11/25.
//

#include "dart_statemachine.hpp"

void DartStateMachineUpdate(DartState &state) {
  // 根据遥控器左拨杆位置设置状态
  if (dart_rack->rc->switch_l() == rm::device::DR16::SwitchPosition::kDown) {  // 左拨杆向下，无力状态
    state.unable = AbleState::kOn;
    state.manual_mode.enabled = AbleState::kOff;
    state.auto_mode.enabled = AbleState::kOff;
    state.adjust_mode.enabled = AbleState::kOff;
  } else if (dart_rack->rc->switch_l() == rm::device::DR16::SwitchPosition::kMid) {  // 左拨杆向中，手动模式
    state.unable = AbleState::kOff;
    state.manual_mode.enabled = AbleState::kOn;
    state.auto_mode.enabled = AbleState::kOff;
    state.adjust_mode.enabled = AbleState::kOff;
  } else if (dart_rack->rc->switch_l() == rm::device::DR16::SwitchPosition::kUp &&
             dart_rack->rc->switch_r() == rm::device::DR16::SwitchPosition::kUp) {  // 左右拨杆向上，自动模式
    state.unable = AbleState::kOff;
    state.manual_mode.enabled = AbleState::kOff;
    state.auto_mode.enabled = AbleState::kOn;
    state.adjust_mode.enabled = AbleState::kOff;
  } else if (dart_rack->rc->switch_l() == rm::device::DR16::SwitchPosition::kUp &&
             dart_rack->rc->switch_r() == rm::device::DR16::SwitchPosition::kMid) {
    state.unable = AbleState::kOff;
    state.manual_mode.enabled = AbleState::kOff;
    state.auto_mode.enabled = AbleState::kOff;
    state.adjust_mode.enabled = AbleState::kOn;
  }
  // 状态机处理逻辑
  if (state.unable == AbleState::kOn) {
    DartStateClear(state);
    dart_rack->yaw_motor->SetCurrent(0);
    dart_rack->load_motor_l->SetCurrent(0);
    dart_rack->load_motor_r->SetCurrent(0);
    dart_rack->trigger_motor->SetCurrent(0);
    dart_rack->trigger_motor_force->SetCurrent(0);
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
  switch (dart_rack->state->manual_mode.mode) {
    case ModeState::kUnable:
      // 等待进入初始化阶段
      if (dart_rack->rc->switch_r() == rm::device::DR16::SwitchPosition::kUp) {
        dart_rack->state->manual_mode.mode = ModeState::kInit;
      }
      break;
    case ModeState::kInit:
      // 初始化逻辑
      if (dart_rack->state->manual_mode.init == PhaseState::kUncomplete) {
        if (dart_rack->rc->switch_r() ==
            rm::device::DR16::SwitchPosition::kUp)  //...
                                                    // 初始化操作 1.拨杆位于最上方 2.扳机到达初始位置
                                                    // 3.上膛机构复位到初始位置
        {
          dart_rack->state->manual_mode.init = PhaseState::kDone;  // 初始化完成
        } else {
          break;
        }
      } else if (dart_rack->state->manual_mode.init == PhaseState::kDone) {
        // 初始化完成，进入下一个阶段
        if (dart_rack->rc->switch_r() == rm::device::DR16::SwitchPosition::kDown) {
          dart_rack->state->manual_mode.mode = ModeState::kAdd;
        }
      }
    case ModeState::kAdd:
      if (dart_rack->state->manual_mode.add == PhaseState::kUncomplete) {
        //... 加弹操作
      } else if (dart_rack->state->manual_mode.add == PhaseState::kDone) {
        // 加弹完成，进入下一个阶段
      }

    case ModeState::kReload:
      if (dart_rack->state->manual_mode.reload == PhaseState::kUncomplete) {
        //... 装填操作
      } else if (dart_rack->state->manual_mode.reload == PhaseState::kDone) {
      }
      // 装填逻辑
      break;
    case ModeState::kChamber:
      if (dart_rack->state->manual_mode.chamber == PhaseState::kUncomplete) {
        //... 上膛操作
      } else if (dart_rack->state->manual_mode.chamber == PhaseState::kDone) {
        // 上膛完成，进入下一个阶段
        dart_rack->state->manual_mode.mode = ModeState::kAim;
      }
      break;
    case ModeState::kAim:
      if (dart_rack->state->manual_mode.aim == PhaseState::kUncomplete) {
        //... 瞄准操作
      } else if (dart_rack->state->manual_mode.aim == PhaseState::kDone) {
        // 瞄准完成，进入下一个阶段
        dart_rack->state->manual_mode.mode = ModeState::kFire;
      }
      break;
    case ModeState::kFire:
      // 发射逻辑
      if (dart_rack->state->manual_mode.fire == PhaseState::kUncomplete) {
        //... 发射操作
      } else if (dart_rack->state->manual_mode.fire == PhaseState::kDone) {
        // 发射完成，返回待机状态
        dart_rack->state->manual_mode.mode = ModeState::kUnable;
      }
      break;
    default:
      break;
  }
}

void DartStateAdjustUpdate() {
  // Yaw轴调节
  if (dart_rack->rc->right_x() > 330) {
    if (dart_rack->yaw_encoder->angle_deg() <= DartRack::kYawEcdMin) {
      dart_rack->yaw_motor->SetCurrent(0);

    } else {
      dart_rack->yaw_motor_speed_pid->Update(-4000.0f, dart_rack->yaw_motor->rpm(), 1.0f);
      dart_rack->yaw_motor->SetCurrent(static_cast<rm::i16>(dart_rack->yaw_motor_speed_pid->out()));
    }
  } else if (dart_rack->rc->right_x() < -330) {
    if (dart_rack->yaw_encoder->angle_deg() >= DartRack::kYawEcdMax) {
      dart_rack->yaw_motor->SetCurrent(0);
    } else {
      dart_rack->yaw_motor_speed_pid->Update(4000.0f, dart_rack->yaw_motor->rpm(), 1.0f);
      dart_rack->yaw_motor->SetCurrent(static_cast<rm::i16>(dart_rack->yaw_motor_speed_pid->out()));
    }
  } else {
    dart_rack->yaw_motor->SetCurrent(0);
  }
  // 扳机位置调节
  if (dart_rack->rc->right_y() > 330) {
    if (dart_rack->trigger_motor_odometer->stall_time() <= 100) {
      dart_rack->trigger_motor_speed_pid->Update(-8000.0f, dart_rack->trigger_motor->rpm(), 1.0f);
      dart_rack->trigger_motor->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid->out()));
    } else {
      dart_rack->trigger_motor->SetCurrent(0);
    }
  } else if (dart_rack->rc->right_y() < -330) {
    if (dart_rack->trigger_motor_odometer->stall_time() <= 100) {
      dart_rack->trigger_motor_speed_pid->Update(8000.0f, dart_rack->trigger_motor->rpm(), 1.0f);
      dart_rack->trigger_motor->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid->out()));
    } else {
      dart_rack->trigger_motor->SetCurrent(0);
    }
  } else {
    dart_rack->trigger_motor_speed_pid->Update(.0f, dart_rack->trigger_motor->rpm(), 1.0f);
    dart_rack->trigger_motor->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid->out()));
  }
  // 上膛调节
  if (dart_rack->rc->left_y() > 330) {
    if (dart_rack->load_motor_l_odometer->stall_time() <= 100 &&
        dart_rack->load_motor_r_odometer->stall_time() <= 100) {
      dart_rack->load_motor_l_speed_pid->Update(3000.0f, dart_rack->load_motor_l->rpm(), 1.0f);
      dart_rack->load_motor_l->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid->out()));
      dart_rack->load_motor_r_speed_pid->Update(-3000.0f, dart_rack->load_motor_r->rpm(), 1.0f);
      dart_rack->load_motor_r->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid->out()));
    } else {
      dart_rack->load_motor_l_speed_pid->Update(.0f, dart_rack->load_motor_l->rpm(), 1.0f);
      dart_rack->load_motor_l->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid->out()));
      dart_rack->load_motor_r_speed_pid->Update(.0f, dart_rack->load_motor_r->rpm(), 1.0f);
      dart_rack->load_motor_r->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid->out()));
    }
  } else if (dart_rack->rc->left_y() < -330) {
    if (dart_rack->load_motor_l_odometer->stall_time() <= 100 &&
        dart_rack->load_motor_r_odometer->stall_time() <= 100) {
      dart_rack->load_motor_l_speed_pid->Update(-3000.0f, dart_rack->load_motor_l->rpm(), 1.0f);
      dart_rack->load_motor_l->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid->out()));
      dart_rack->load_motor_r_speed_pid->Update(3000.0f, dart_rack->load_motor_r->rpm(), 1.0f);
      dart_rack->load_motor_r->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid->out()));
    } else {
      dart_rack->load_motor_l_speed_pid->Update(.0f, dart_rack->load_motor_l->rpm(), 1.0f);
      dart_rack->load_motor_l->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid->out()));
      dart_rack->load_motor_r_speed_pid->Update(.0f, dart_rack->load_motor_r->rpm(), 1.0f);
      dart_rack->load_motor_r->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid->out()));
    }
  } else {
    dart_rack->load_motor_l_speed_pid->Update(.0f, dart_rack->load_motor_l->rpm(), 1.0f);
    dart_rack->load_motor_l->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid->out()));
    dart_rack->load_motor_r_speed_pid->Update(.0f, dart_rack->load_motor_r->rpm(), 1.0f);
    dart_rack->load_motor_r->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid->out()));
  }
  // 扳机触发调节
  if (dart_rack->rc->left_x() > 330) {
    if (dart_rack->trigger_motor_force_odometer->stall_time() <= 100) {
      if (dart_rack->trigger_motor_force->encoder() <= 8000) {
        dart_rack->trigger_motor_force_pid->Update(1000.0f, dart_rack->trigger_motor_force->rpm(), 1.0f);
        dart_rack->trigger_motor_force->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid->out()));
      } else {
        dart_rack->trigger_motor_force_pid->Update(0.0f, dart_rack->trigger_motor_force->rpm(), 1.0f);
        dart_rack->trigger_motor_force->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid->out()));
      }
    } else {
      dart_rack->trigger_motor_force_pid->Update(0.0f, dart_rack->trigger_motor_force->rpm(), 1.0f);
      dart_rack->trigger_motor_force->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid->out()));
    }

  } else if (dart_rack->rc->left_x() < -330) {
    if (dart_rack->trigger_motor_force_odometer->stall_time() <= 100) {
      if (dart_rack->trigger_motor_force->encoder() >= 5000) {
        dart_rack->trigger_motor_force_pid->Update(-1000.0f, dart_rack->trigger_motor_force->rpm(), 1.0f);
        dart_rack->trigger_motor_force->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid->out()));
      } else {
        dart_rack->trigger_motor_force_pid->Update(0.0f, dart_rack->trigger_motor_force->rpm(), 1.0f);
        dart_rack->trigger_motor_force->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid->out()));
      }
    } else {
      dart_rack->trigger_motor_force_pid->Update(0.0f, dart_rack->trigger_motor_force->rpm(), 1.0f);
      dart_rack->trigger_motor_force->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid->out()));
    }
  } else {
    dart_rack->trigger_motor_force->SetCurrent(0);
  }
}