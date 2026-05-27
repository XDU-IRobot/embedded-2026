//
// Created by Jason on 26-5-27.
//

#include "UIDrone.hpp"
#include "librm.hpp"
#include "../main.hpp"

using namespace rm;
using namespace rm::device;

extern u16 robotID;
extern u8 dataBox[128];

static float cmd_yaw = 0.0f;
static i16 cmd_ammo = 0;
static u8 cmd_fire = 0;

// 无人机显示英雄UI
void UIDroneHero_add() {
  UIFigure7 UIGroup1;
  UIGroup1.figure1.fillFloat("yaw", UIFigure::Operation::Add, 0, UIFigure::Color::Yellow, 5, 1460, 470, 20,
                             gimbal->referee_user.data().hero_2_drone.hero_yaw_angle * 1000);
  UIGroup1.figure2.fillFloat("pit", UIFigure::Operation::Add, 0, UIFigure::Color::Black, 5, 1620, 470, 20,
                             gimbal->referee_user.data().hero_2_drone.hero_pitch_angle * 1000);
  UIGroup1.figure3.fillIntegrate("adj", UIFigure::Operation::Add, 0, UIFigure::Color::Green, 5, 1650, 580, 22,
                                 gimbal->referee_user.data().hero_2_drone.hero_ammo_adjust);
  UIGroup1.figure4.fillFloat("isp", UIFigure::Operation::Add, 0, UIFigure::Color::Cyan, 5, 1460, 580, 22,
                             gimbal->referee_user.data().hero_2_drone.hero_initial_speed * 1000);
  UIGroup1.figure5.fillFloat("cya", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 5, 1460, 520, 24,
                             cmd_yaw * 1000);
  UIGroup1.figure6.fillFloat("dyw", UIFigure::Operation::Add, 0, UIFigure::Color::Yellow, 5, 1460, 420, 20,
                             gimbal->yaw_ * 1000);
  UIGroup1.figure7.fillFloat("dpt", UIFigure::Operation::Add, 0, UIFigure::Color::Black, 5, 1620, 420, 20,
                             gimbal->pitch_ * 1000);

  const auto dataLen = Referee0x301Prepare(dataBox, 0, UIGroup1, robotID, robotID + 256);
  gimbal->referee_uart->Write(dataBox, dataLen);
}

void UIDroneHero_edit() {
  UIFigure7 UIGroup1;
  UIGroup1.figure1.fillFloat("yaw", UIFigure::Operation::Edit, 0, UIFigure::Color::Yellow, 5, 1460, 470, 20,
                             gimbal->referee_user.data().hero_2_drone.hero_yaw_angle * 180 / 3.1415 * 1000);
  UIGroup1.figure2.fillFloat("pit", UIFigure::Operation::Edit, 0, UIFigure::Color::Magenta, 5, 1620, 470, 20,
                             gimbal->referee_user.data().hero_2_drone.hero_pitch_angle * 1000);
  UIGroup1.figure3.fillIntegrate("adj", UIFigure::Operation::Edit, 0, UIFigure::Color::Green, 5, 1650, 580, 22,
                                 gimbal->referee_user.data().hero_2_drone.hero_ammo_adjust);
  UIGroup1.figure4.fillFloat("isp", UIFigure::Operation::Edit, 0, UIFigure::Color::Cyan, 5, 1460, 580, 22,
                             gimbal->referee_user.data().hero_2_drone.hero_initial_speed * 1000);
  UIGroup1.figure5.fillFloat("cya", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 5, 1460, 520, 24,
                             cmd_yaw * 180 / 3.1415 * 1000);
  UIGroup1.figure6.fillFloat("dyw", UIFigure::Operation::Edit, 0, UIFigure::Color::Yellow, 5, 1460, 420, 20,
                             gimbal->yaw_ * 1000);
  UIGroup1.figure7.fillFloat("dpt", UIFigure::Operation::Edit, 0, UIFigure::Color::Black, 5, 1620, 420, 20,
                             gimbal->pitch_ * 1000);
  const auto dataLen = Referee0x301Prepare(dataBox, 0, UIGroup1, robotID, robotID + 256);
  gimbal->referee_uart->Write(dataBox, dataLen);
}

void D2H_func() {
  static auto last_rc_key_x = false;
  static auto last_rc_key_c = false;
  static auto now_rc_key_x = false;
  static auto now_rc_key_c = false;
  static auto state_ = false;
  if (gimbal->Rcchoose() == 2) {
    now_rc_key_x = gimbal->vt03->data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kX);
    now_rc_key_c = gimbal->vt03->data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kC);
    state_ = gimbal->vt03->data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kCtrl);
  } else if (gimbal->Rcchoose() == 1) {
    now_rc_key_x = gimbal->rc->key(DR16::Key::kX);
    now_rc_key_c = gimbal->rc->key(DR16::Key::kC);
    state_ = gimbal->rc->key(DR16::Key::kCtrl);
  } else {
    now_rc_key_x = false;
    now_rc_key_c = false;
    state_ = false;
  }
  if (!state_) {
    cmd_yaw = gimbal->referee_user.data().hero_2_drone.hero_yaw_angle;
    cmd_ammo = gimbal->referee_user.data().hero_2_drone.hero_ammo_adjust;
  } else {
    if (gimbal->Rcchoose() == 2) {
      if (gimbal->vt03->data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kA))
        cmd_yaw += 0.000125f;
      if (gimbal->vt03->data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kD))
        cmd_yaw -= 0.000125f;
    } else if (gimbal->Rcchoose() == 1) {
      if (gimbal->rc->key(DR16::Key::kA)) cmd_yaw += 0.000125f;
      if (gimbal->rc->key(DR16::Key::kD)) cmd_yaw -= 0.000125f;
    }
    if (last_rc_key_x == false && now_rc_key_x) {
      cmd_ammo += 10;
    } else if (last_rc_key_c == false && now_rc_key_c) {
      cmd_ammo -= 10;
    }
  }
  last_rc_key_x = now_rc_key_x;
  last_rc_key_c = now_rc_key_c;

  if (gimbal->Rcchoose() == 2) {
    cmd_fire = gimbal->vt03->data().mouse_button_left;
  } else if (gimbal->Rcchoose() == 1) {
    cmd_fire = gimbal->rc->mouse_button_left();
  } else {
    cmd_fire = false;
  }
  Drone2Hero d2h{cmd_yaw, cmd_ammo, cmd_fire};
  const auto size = Referee0x301Prepare(dataBox, 0, d2h, robotID, robotID - 5);
  gimbal->referee_uart->Write(dataBox, size);
}