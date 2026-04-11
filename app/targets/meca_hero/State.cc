#include "State.hpp"
#include "librm.hpp"
#include "main.hpp"

/**
 * @brief:
 * @note:
 **/
void StateMachine::StateUpdate() {
  //
  VT03::SwitchPosition tc_switch_position = globals->tc->data().switch_position;
  DR16::SwitchPosition rc_switch_position_r = globals->rc->switch_r();
  DR16::SwitchPosition rc_switch_position_l = globals->rc->switch_l();

  // 子模式状态位
  static bool follow{true};  // 随动标志位
  if (key_once_tc(VT03::KeyboardKey::kC)) follow = !follow;
  static bool snipe{false};  // 部署标志位
  if (key_once_tc(VT03::KeyboardKey::kG)) snipe = !snipe;
  bool radar_aimbot = globals->tc->data().right_button;  // 部署模式下的雷达自瞄
  bool overpower = globals->tc->data().keyboard_key & static_cast<int16_t>(VT03::KeyboardKey::kShift);  // 爬坡-自动触发
  bool aimbot = globals->tc->data().keyboard_key & static_cast<int16_t>(VT03::KeyboardKey::kCtrl) |
                globals->tc->data().right_button;  // 常态模式下的装甲板自瞄
  bool autofire =
      globals->tc->data().keyboard_key & static_cast<int16_t>(VT03::KeyboardKey::kCtrl);  // 自瞄决定开火标志位

  //
  last_sub_state_ = current_sub_state_;
  last_main_state_ = current_main_state_;
  last_chassis_state_ = current_chassis_state_;
  last_ammo_state_ = current_ammo_state_;
  last_gimbal_state_ = current_gimbal_state_;

  // 图传最高优先级，图传在C强制失能，S强制进入比赛并进入子模式
  if (tc_switch_position == VT03::SwitchPosition::C) {
    current_main_state_ = MainState::kOffline;
    return;
  } else if (tc_switch_position == VT03::SwitchPosition::S) {
    current_main_state_ = MainState::kGame;
    if (overpower) {
      current_sub_state_ = SubState::kOverPower;
    } else if (snipe) {
      current_sub_state_ = SubState::kSnipe;
      if (radar_aimbot) {
      }
    } else {
      current_sub_state_ = SubState::kNoAct;
    }
    return;
  }
  // DT7控制模式
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
          switch (rc_switch_position_l) {
            // case DR16::SwitchPosition::kDown: //遥控模式，随动默认开启
            //   current_sub_state_ = SubState::kFollow;
            //   break;
            // case DR16::SwitchPosition::kMid:
            //   current_sub_state_ = SubState::
          };
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