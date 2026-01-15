#include "fsm.hpp"

#include <librm.hpp>

#include "global.hpp"

void Fsm::Transit(State new_mode) {
  // 输入新状态
  if (new_mode != mode_) {
    if (new_mode == State::kNoForce) {
      global.motor->DMDisable();
      global.motor->ShootEnable(false);
    } else {
      global.motor->DMEnable();
      // global.motor->ShootEnable(true);
    }
  }
  // 替换现有状态
  mode_ = new_mode;
}

void Fsm::Update() {
  // 根据遥控器切换状态
  if (global.bc->rc->switch_r() == DR16::SwitchPosition::kDown) {
    Transit(State::kNoForce);
  } else if (global.bc->rc->switch_r() == DR16::SwitchPosition::kMid) {
    Transit(State::kTest);
  } else {
    Transit(State::kNoForce);
  }

  switch (mode_) {
    case State::kNoForce:
      break;
    case State::kTest:
      if (init_count_ < 300) {
        init_count_++;
        global.motor->DMInit();
      } else {
        global.motor->DMControl();
      }
  }
}