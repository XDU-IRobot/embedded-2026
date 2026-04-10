#include "State.hpp"
#include "librm.hpp"
#include "main.hpp"

/**
 * @brief:
 * @note:
 **/
void StateMachine::StateUpdate() {
  VT03::SwitchPosition tc_switch_position = globals->tc->data().switch_position;
  DR16::SwitchPosition rc_switch_position_r = globals->rc->switch_r();
  DR16::SwitchPosition rc_switch_position_l = globals->rc->switch_l();

  last_sub_state_ = current_sub_state_;
  last_main_state_ = current_main_state_;
  //图传最高优先级
  if (tc_switch_position == VT03::SwitchPosition::C) {
    current_main_state_ = MainState::kOffline;
    return;
  } else if (tc_switch_position == VT03::SwitchPosition::S) {
    current_main_state_ = MainState::kGame;
    return;
  }
  //DT7控制模式
  switch (rc_switch_position_r) {
    case DR16::SwitchPosition::kUnknown:
      current_main_state_ = MainState::kOffline;
      break;

    case DR16::SwitchPosition::kDown:
      current_main_state_ = MainState::kOffline;
      break;

    case DR16::SwitchPosition::kMid:
      switch (current_main_state_) {
        case MainState::kOffline:
          current_main_state_ = MainState::kWaiting;
          break;
        case MainState::kWaiting:
          if (count > 0) {
            count--;
          } else {
            current_main_state_ = MainState::kTest;
            count = waiting_count_;
            break;
          }
        case MainState::kTest:
          switch (current_sub_state_) {

          }
        default:
          current_main_state_ = MainState::kTest;
          break;
      }
      break;

    case DR16::SwitchPosition::kUp:
      switch (current_main_state_) {
        case MainState::kOffline:
          current_main_state_ = MainState::kWaiting;
          break;
        case MainState::kTest:
          current_main_state_ = MainState::kWaiting;
          break;
        case MainState::kWaiting:
          if (count > 0) {
            count--;
          } else {
            current_main_state_ = MainState::kGame;
            count = waiting_count_;
            break;
          }
        default:
          current_main_state_ = MainState::kGame;
          break;
      }
      break;
    default:
      current_main_state_ = MainState::kOffline;
      break;
  }
}