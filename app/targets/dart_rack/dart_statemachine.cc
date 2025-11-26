//
// Created by 34236 on 2025/11/25.
//

#include "dart_statemachine.hpp"

void DartStateMachineUpdate(DartState &state) {
  // 状态机处理逻辑
  if (state.unable == AbleState::kOn) {
    DartStateClear(state);
    return;
  } else if (state.manual_mode.enabled == AbleState::kOn) {
    DartStateManualUpadte();
  } else if (state.auto_mode.enabled == AbleState::kOn) {
  } else {
    DartStateClear(state);
  }
}
// 手动模式状态机处理
void DartStateManualUpadte() {
  switch (dart_rack->state->manual_mode.mode) {
    case ModeState::kInit:
      // 初始化逻辑
      if (dart_rack->state->manual_mode.init == PhaseState::kUncomplete) {
        //... 初始化操作
      } else if (dart_rack->state->manual_mode.init == PhaseState::kDone) {
        // 初始化完成，进入下一个阶段
        dart_rack->state->manual_mode.mode = ModeState::kReload;
      }
      break;
    case ModeState::kReload:
      if (dart_rack->state->manual_mode.reload == PhaseState::kUncomplete) {
        //... 装填操作
      } else if (dart_rack->state->manual_mode.reload == PhaseState::kDone) {
        // 装填完成，进入下一个阶段
        dart_rack->state->manual_mode.mode = ModeState::kChamber;
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