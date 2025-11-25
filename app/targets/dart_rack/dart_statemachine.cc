//
// Created by 34236 on 2025/11/25.
//

#include "dart_statemachine.hpp"

void DartStateMachineUpdate(DartState &state) {
  // 状态机处理逻辑
  if (state.unable == AbleState::kOn) {
    DartStateClear(state);
    return;
  }
  else if (state.manual_mode.enabled== AbleState::kOn) {
    DartStateManualUpadte();
  }
  else if (state.auto_mode.enabled == AbleState::kOn) {

  }
  else{
    DartStateClear(state);}
  }
// 手动模式状态机处理
void DartStateManualUpadte() {
  switch (dart_rack->state->manual_mode.mode) {
        case ModeState::kInit:
          // 初始化逻辑
          break;
        case ModeState::kReload:
          // 装填逻辑
          break;
        case ModeState::kChamber:
          // 上膛逻辑
          break;
        case ModeState::kAim:
          // 瞄准逻辑
          break;
        case ModeState::kFire:
          // 发射逻辑
          break;
        default:
          break;
  }

}