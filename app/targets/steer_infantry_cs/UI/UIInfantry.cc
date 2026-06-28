//
// Created by Jason on 26-5-27.
//

#include "UIInfantry.hpp"
#include "librm.hpp"
#include "../main.hpp"
#include "../Chassis.hpp"

using namespace rm;
using namespace rm::device;

extern u16 robotID;

// 无人机显示英雄UI
void UIInfantryAdd1() {
  UIFigure7 UIGroup1;
  UIGroup1.figure1.fillLine("xxx", UIFigure::Operation::Add, 0, UIFigure::Color::Orange, 2, 918, 515, 978, 515);
  UIGroup1.figure2.fillLine("yyy", UIFigure::Operation::Add, 0, UIFigure::Color::Orange, 2, 948, 465, 948, 565);

  UIGroup1.figure3.fillRec("gtf", UIFigure::Operation::Add, 0, UIFigure::Color::Magenta, 5, 150, 727, 380, 688);
  UIGroup1.figure4.fillRec("sff", UIFigure::Operation::Add, 0, UIFigure::Color::Magenta, 5, 150, 686, 430, 651);
  UIGroup1.figure5.fillRec("cmf", UIFigure::Operation::Add, 0, UIFigure::Color::Magenta, 5, 1592, 728, 1622, 690);
  UIGroup1.figure6.fillRec("bmf", UIFigure::Operation::Add, 0, UIFigure::Color::Magenta, 5, 1742, 728, 1772, 690);
  UIGroup1.figure7.fillRec("smf", UIFigure::Operation::Add, 0, UIFigure::Color::Magenta, 5, 1542, 686, 1572, 648);
  const auto dataLen = Referee0x301Prepare(globals->dataBox, 0, UIGroup1, robotID, robotID + 256);
  globals->referee_uart->Write(globals->dataBox, dataLen, 500);
}
void UIInfantryAdd2() {
  UIFigure2 UIGroup1;
  UIGroup1.figure1.fillFloat("cms", UIFigure::Operation::Add, 0, UIFigure::Color::Green, 5, 900, 270, 27,
                             static_cast<f32>(globals->super_cap->GetCapEnergy()) * 1000.0f);
  UIGroup1.figure2.fillFloat("asj", UIFigure::Operation::Add, 0, UIFigure::Color::White, 2, 100, 720, 25,
                             static_cast<f32>(globals->gimbal_communicator->aim_speed_change()) * 1000.0f);
  const auto dataLen = Referee0x301Prepare(globals->dataBox, 0, UIGroup1, robotID, robotID + 256);
  globals->referee_uart->Write(globals->dataBox, dataLen, 500);
}
void UIInfantryAdd3() {
  UICharacter UITextHeader;
  UITextHeader.character.fillCharacter("aim", UIFigure::Operation::Add, 0, UIFigure::Color::Green, 2, 160, 720, 25, 21);
  memcpy(UITextHeader.data, "GETTARGET\nSUGGESTFIRE", 21);
  const auto dataLen = Referee0x301Prepare(globals->dataBox, 0, UITextHeader, robotID, robotID + 256);
  globals->referee_uart->Write(globals->dataBox, dataLen, 500);
}
void UIInfantryAdd4() {
  UICharacter UITextHeader;
  UITextHeader.character.fillCharacter("mod", UIFigure::Operation::Add, 0, UIFigure::Color::Green, 2, 1500, 720, 25,
                                       17);
  memcpy(UITextHeader.data, "F R N U D X\nH N S", 17);
  const auto dataLen = Referee0x301Prepare(globals->dataBox, 0, UITextHeader, robotID, robotID + 256);
  globals->referee_uart->Write(globals->dataBox, dataLen, 500);
}

void UIInfantryEdit() {
  UIFigure7 UIGroup1;
  // 电容电压
  if (chassis->speed_mode_ == kHighSpeed) {
    UIGroup1.figure1.fillFloat("cms", UIFigure::Operation::Edit, 0, UIFigure::Color::Green, 5, 900, 270, 27,
                               static_cast<f32>(globals->super_cap->GetCapEnergy()) * 1000.0f);
  } else {
    UIGroup1.figure1.fillFloat("cms", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 5, 900, 270, 27,
                               static_cast<f32>(globals->super_cap->GetCapEnergy()) * 1000.0f);
  }
  // 弹速调节
  if (globals->gimbal_communicator->aim_speed_change() > 0) {
    UIGroup1.figure2.fillFloat("asj", UIFigure::Operation::Edit, 0, UIFigure::Color::Green, 2, 100, 720, 25,
                               static_cast<f32>(globals->gimbal_communicator->aim_speed_change()) * 1000.0f);
  } else if (globals->gimbal_communicator->aim_speed_change() < 0) {
    UIGroup1.figure2.fillFloat("asj", UIFigure::Operation::Edit, 0, UIFigure::Color::Pink, 2, 100, 720, 25,
                               static_cast<f32>(globals->gimbal_communicator->aim_speed_change()) * 1000.0f);
  } else {
    UIGroup1.figure2.fillFloat("asj", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 2, 100, 720, 25,
                               static_cast<f32>(globals->gimbal_communicator->aim_speed_change()) * 1000.0f);
  }
  // // 自瞄模式
  if (globals->gimbal_communicator->get_target_flag() == 1) {
    UIGroup1.figure3.fillRec("gtf", UIFigure::Operation::Edit, 0, UIFigure::Color::Magenta, 3, 150, 727, 380, 688);
  } else {
    UIGroup1.figure3.fillRec("gtf", UIFigure::Operation::Edit, 0, UIFigure::Color::Magenta, 0, 150, 727, 380, 688);
  }

  if (globals->gimbal_communicator->suggest_fire_flag() == 1) {
    UIGroup1.figure4.fillRec("sff", UIFigure::Operation::Edit, 0, UIFigure::Color::Magenta, 3, 150, 686, 430, 651);
  } else {
    UIGroup1.figure4.fillRec("sff", UIFigure::Operation::Edit, 0, UIFigure::Color::Magenta, 0, 150, 686, 430, 651);
  }

  // 底盘模式
  if (chassis->ChassisMove_ == kFollow) {
    UIGroup1.figure5.fillRec("cmf", UIFigure::Operation::Edit, 0, UIFigure::Color::Magenta, 3, 1492, 728, 1522, 690);
  } else if (chassis->ChassisMove_ == kRotate || chassis->ChassisMove_ == kReRotate) {
    UIGroup1.figure5.fillRec("cmf", UIFigure::Operation::Edit, 0, UIFigure::Color::Magenta, 3, 1542, 728, 1572, 690);
  } else if (chassis->ChassisMove_ == kNoForce) {
    UIGroup1.figure5.fillRec("cmf", UIFigure::Operation::Edit, 0, UIFigure::Color::Magenta, 3, 1592, 728, 1622, 690);
  } else {
    UIGroup1.figure5.fillRec("cmf", UIFigure::Operation::Edit, 0, UIFigure::Color::Magenta, 3, 1642, 728, 1672, 690);
  }
  if (chassis->buff_state_ == kDaFu) {
    UIGroup1.figure6.fillRec("bmf", UIFigure::Operation::Edit, 0, UIFigure::Color::Magenta, 3, 1692, 728, 1722, 690);
  } else if (chassis->buff_state_ == kXiaoFu) {
    UIGroup1.figure6.fillRec("bmf", UIFigure::Operation::Edit, 0, UIFigure::Color::Magenta, 3, 1742, 728, 1772, 690);
  } else {
    UIGroup1.figure6.fillRec("bmf", UIFigure::Operation::Edit, 0, UIFigure::Color::Magenta, 0, 1742, 728, 1772, 690);
  }

  // 底盘速度模式
  if (chassis->speed_mode_ == kHighSpeed) {
    UIGroup1.figure7.fillRec("smf", UIFigure::Operation::Edit, 0, UIFigure::Color::Magenta, 3, 1492, 686, 1522, 648);
  } else {
    UIGroup1.figure7.fillRec("smf", UIFigure::Operation::Edit, 0, UIFigure::Color::Magenta, 3, 1542, 686, 1572, 648);
  }
  if (globals->referee_data->data().power_heat_data.buffer_energy < 40) {
    UIGroup1.figure7.fillRec("smf", UIFigure::Operation::Edit, 0, UIFigure::Color::Magenta, 3, 1592, 686, 1622, 648);
  }
  const auto dataLen = Referee0x301Prepare(globals->dataBox, 0, UIGroup1, robotID, robotID + 256);
  globals->referee_uart->Write(globals->dataBox, dataLen, 500);
}