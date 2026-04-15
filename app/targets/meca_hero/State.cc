#include "State.hpp"
#include "librm.hpp"
#include "main.hpp"

/**
 *@brief:DT7遥控器切换状态
 */
void StateMachine::DT7Switch() {
  switch (StateMachine::rc_switch_position_r) {
    case rm::device::DR16::SwitchPosition::kUnknown: /// 保险
      current_main_state_ = MainState::kOffline;
      break;

    case rm::device::DR16::SwitchPosition::kDown: /// 失能
      current_main_state_ = MainState::kOffline;
      break;

    case rm::device::DR16::SwitchPosition::kMid: /// 调试
      switch (current_main_state_) {
        case MainState::kOffline:
          current_main_state_ = MainState::kWaiting;
          break;
        case MainState::kWaiting:
          if (Waiting(waiting_count_)) current_main_state_ = MainState::kTest;
          break;
        case MainState::kTest:
          switch (rc_switch_position_l) {
            case rm::device::DR16::SwitchPosition::kDown: /// 普通遥控
              current_sub_state_ = SubState::kNormal;
              /// 随动切换
              TestFollowSwitch();
              break;
            case rm::device::DR16::SwitchPosition::kMid: /// 超功率+上坡（应该直接加pitch无力
              current_sub_state_ = SubState::kOverPower;
              break;
            case rm::device::DR16::SwitchPosition::kUp:
              current_sub_state_ = SubState::kSnipe;
              break;
            default:
              current_sub_state_ = SubState::kNormal;
          };
          break;
        default:
          current_main_state_ = MainState::kTest;
          break;
      }
      break;

    case rm::device::DR16::SwitchPosition::kUp:
      switch (current_main_state_) {
        case MainState::kOffline: //保险
          current_main_state_ = MainState::kWaiting;
          break;
        case MainState::kTest:
          current_main_state_ = MainState::kWaiting;
          break;
        case MainState::kWaiting:
          if (Waiting(waiting_count_)) current_main_state_ = MainState::kGame;
          break;
        case MainState::kGame:

          break;
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

/**
 * @brief:状态机更新函数
 * @note:
 **/
void StateMachine::StateUpdate() {
  //
  rc_switch_position_l = globals->rc->switch_l();
  tc_switch_position = globals->tc->data().switch_position;
  rc_switch_position_r = globals->rc->switch_r();

  // 子模式状态位
  static bool follow{true}; // 随动标志位
  if (key_once_tc(VT03::KeyboardKey::kC)) follow = !follow;
  static bool snipe{false}; // 部署标志位
  if (key_once_tc(VT03::KeyboardKey::kG)) snipe = !snipe;
  bool radar_aimbot = globals->tc->data().right_button; // 部署模式下的雷达自瞄
  bool overpower = globals->tc->data().keyboard_key & static_cast<int16_t>(VT03::KeyboardKey::kShift); // 爬坡-自动触发
  bool aimbot = globals->tc->data().keyboard_key & static_cast<int16_t>(VT03::KeyboardKey::kCtrl) |
                globals->tc->data().right_button; // 常态模式下的装甲板自瞄
  bool autofire =
      globals->tc->data().keyboard_key & static_cast<int16_t>(VT03::KeyboardKey::kCtrl); // 自瞄决定开火标志位

  // 图传最高优先级，图传在C强制失能，S强制进入比赛并进入子模式
  if (tc_switch_position == VT03::SwitchPosition::C) {
    current_main_state_ = MainState::kOffline;
    return;
  } else if (tc_switch_position == VT03::SwitchPosition::S) {
    switch (current_main_state_) {
      case MainState::kOffline:
        current_main_state_ = MainState::kWaiting;
        break;
      case MainState::kWaiting:
        if (Waiting()) current_main_state_ = MainState::kGame;
        break;
      case MainState::kGame:
        if (snipe) {
          current_sub_state_ = SubState::kSnipe;
        } else if (overpower) {
          current_sub_state_ = SubState::kOverPower;
        } else {
          current_sub_state_ = SubState::kNormal;
        }
        // 模块状态
        if (follow) {
          current_chassis_state_ = ChassisState::kFollow;
        } else {
          current_chassis_state_ = ChassisState::kNormal;
        }

        if () break;
      default:
        current_main_state_ = MainState::kGame;
        break;
    }
    current_main_state_ = MainState::kGame;

    return;
  }

  DT7Switch();
}