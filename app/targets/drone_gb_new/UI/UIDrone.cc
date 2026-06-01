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
  UIGroup1.figure6.fillFloat("dyw", UIFigure::Operation::Edit, 0, UIFigure::Color::Yellow, 2, 1460, 420, 20,
                             gimbal->yaw_ * 180 / 3.1415 * 1000);
  UIGroup1.figure7.fillFloat("dpt", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 2, 1620, 420, 20,
                             gimbal->pitch_ * 180 / 3.1415 * 1000);
  const auto dataLen = Referee0x301Prepare(dataBox, 0, UIGroup1, robotID, robotID + 256);
  gimbal->referee_uart->Write(dataBox, dataLen);
}

void D2H_func() {
  static auto last_rc_key_x = false;
  static auto last_rc_key_c = false;
  static auto now_rc_key_x = false;
  static auto now_rc_key_c = false;
  static auto state_ = false;

  now_rc_key_x = gimbal->control_rc->key(DR16::Key::kX);
  now_rc_key_c = gimbal->control_rc->key(DR16::Key::kC);
  state_ = gimbal->control_rc->key(DR16::Key::kCtrl);

  if (!state_) {
    cmd_yaw = gimbal->referee_user.data().hero_2_drone.hero_yaw_angle;
    cmd_ammo = gimbal->referee_user.data().hero_2_drone.hero_ammo_adjust;
  } else {
    if (gimbal->control_rc->key(DR16::Key::kA)) cmd_yaw += 0.005f;
    if (gimbal->control_rc->key(DR16::Key::kD)) cmd_yaw -= 0.005f;

    if (last_rc_key_x == false && now_rc_key_x) {
      cmd_ammo += 10;
    } else if (last_rc_key_c == false && now_rc_key_c) {
      cmd_ammo -= 10;
    }
  }
  last_rc_key_x = now_rc_key_x;
  last_rc_key_c = now_rc_key_c;
  cmd_fire = gimbal->control_rc->mouse_button_left();
  Drone2Hero d2h{cmd_yaw, cmd_ammo, cmd_fire};
  const auto size = Referee0x301Prepare(dataBox, 0, d2h, robotID, robotID - 5);
  gimbal->referee_uart->Write(dataBox, size);
}

void drone_state_1_add() {
  UIFigure7 UIGroup1;
  UIGroup1.figure1.fillIntegrate("gim", UIFigure::Operation::Add, 0, UIFigure::Color::Magenta, 5, 300, 760, 20, 0);
  UIGroup1.figure2.fillIntegrate("fir", UIFigure::Operation::Add, 0, UIFigure::Color::Green, 5, 300, 730, 20, 0);
  UIGroup1.figure3.fillIntegrate("shr", UIFigure::Operation::Add, 0, UIFigure::Color::White, 5, 320, 700, 20, 0);
  UIGroup1.figure4.fillRec("aut", UIFigure::Operation::Add, 0, UIFigure::Color::Green, 3, 794, 422, 1096, 665);
  const auto dataLen = Referee0x301Prepare(dataBox, 0, UIGroup1, robotID, robotID + 256);
  gimbal->referee_uart->Write(dataBox, dataLen);
}

void base_line_add_1() {
  UIFigure7 UIGroup1;
  UIGroup1.figure1.fillLine("ba1", UIFigure::Operation::Add, 0, UIFigure::Color::Orange, 1, 933, 522, 951, 468);
  UIGroup1.figure2.fillLine("ba2", UIFigure::Operation::Add, 0, UIFigure::Color::Orange, 1, 983, 522, 962, 468);
  UIGroup1.figure3.fillLine("li2", UIFigure::Operation::Add, 0, UIFigure::Color::Orange, 2, 933, 522, 983, 522);
  UIGroup1.figure4.fillLine("li4", UIFigure::Operation::Add, 0, UIFigure::Color::Orange, 2, 941, 515, 968, 515);
  UIGroup1.figure5.fillLine("li6", UIFigure::Operation::Add, 0, UIFigure::Color::Orange, 2, 945, 507, 963, 507);
  UIGroup1.figure6.fillLine("li8", UIFigure::Operation::Add, 0, UIFigure::Color::Orange, 2, 947, 493, 961, 493);
  UIGroup1.figure7.fillLine("l10", UIFigure::Operation::Add, 0, UIFigure::Color::Orange, 2, 951, 468, 962, 468);
  const auto dataLen = Referee0x301Prepare(dataBox, 0, UIGroup1, robotID, robotID + 256);
  gimbal->referee_uart->Write(dataBox, dataLen);
}

void base_line_add_2() {
  UIFigure7 UIGroup1;
  UIGroup1.figure1.fillLine("bb1", UIFigure::Operation::Add, 0, UIFigure::Color::White, 3, 887, 209, 900, 13);
  UIGroup1.figure2.fillLine("bb2", UIFigure::Operation::Add, 0, UIFigure::Color::White, 3, 1019, 209, 985, 13);
  UIGroup1.figure3.fillLine("bl6", UIFigure::Operation::Add, 0, UIFigure::Color::White, 3, 887, 209, 1019, 209);
  UIGroup1.figure4.fillLine("bl8", UIFigure::Operation::Add, 0, UIFigure::Color::White, 3, 890, 50, 1000, 50);
  UIGroup1.figure5.fillLine("b10", UIFigure::Operation::Add, 0, UIFigure::Color::White, 3, 900, 13, 985, 13);
  const auto dataLen = Referee0x301Prepare(dataBox, 0, UIGroup1, robotID, robotID + 256);
  gimbal->referee_uart->Write(dataBox, dataLen);
}

void drone_state_1_edit() {
  UIFigure7 UIGroup1;
  UIGroup1.figure1.fillIntegrate("gim", UIFigure::Operation::Edit, 0, UIFigure::Color::Magenta, 5, 300, 760, 20,
                                 gimbal->AmmoState_);
  UIGroup1.figure2.fillIntegrate("fir", UIFigure::Operation::Edit, 0, UIFigure::Color::Green, 5, 300, 730, 20,
                                 gimbal->GimbalState_);
  UIGroup1.figure3.fillIntegrate("shr", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 5, 320, 700, 20,
                                 gimbal->shootcnt);
  if (Aimbot.AimbotState == 4 || Aimbot.AimbotState == 2)
    UIGroup1.figure4.fillRec("aut", UIFigure::Operation::Edit, 0, UIFigure::Color::Magenta, 3, 794, 422, 1096, 665);
  else
    UIGroup1.figure4.fillRec("aut", UIFigure::Operation::Edit, 0, UIFigure::Color::Green, 3, 794, 422, 1096, 665);

  const auto dataLen = Referee0x301Prepare(dataBox, 0, UIGroup1, robotID, robotID + 256);
  gimbal->referee_uart->Write(dataBox, dataLen);
}