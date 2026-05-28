//
// Created by Jason on 26-5-27.
//

#include "UIDrone.hpp"
#include "librm.hpp"
#include "../main.hpp"
#include "../Chassis.hpp"

using namespace rm;
using namespace rm::device;

extern u16 robotID;

// 无人机显示英雄UI
void UIInfantryAdd() {
  UIFigure7 UIGroup1;
  UIGroup1.figure1.fillLine("xxx", UIFigure::Operation::Add, 0, UIFigure::Color::Orange, 2, 918, 515, 978, 515);
  UIGroup1.figure2.fillLine("yyy", UIFigure::Operation::Add, 0, UIFigure::Color::Orange, 2, 948, 465, 948, 565);

  UIGroup1.figure3.fillRec("gtf", UIFigure::Operation::Add, 0, UIFigure::Color::Magenta, 5, 350, 807, 580, 768);
  UIGroup1.figure4.fillRec("sff", UIFigure::Operation::Add, 0, UIFigure::Color::Magenta, 5, 350, 766, 630, 731);
  UIGroup1.figure5.fillRec("cmf", UIFigure::Operation::Add, 0, UIFigure::Color::Magenta, 5, 1392, 808, 1422, 770);
  UIGroup1.figure6.fillRec("bmf", UIFigure::Operation::Add, 0, UIFigure::Color::Magenta, 5, 1542, 808, 1572, 770);
  UIGroup1.figure7.fillRec("smf", UIFigure::Operation::Add, 0, UIFigure::Color::Magenta, 5, 1342, 766, 1372, 728);
  UIFigure2 UIGroup2;
  UIGroup2.figure1.fillFloat("cms", UIFigure::Operation::Add, 0, UIFigure::Color::Green, 5, 900, 270, 27,
                             static_cast<f32>(globals->super_cap->CapEnergy()) * 1000.0f);
  UIGroup2.figure2.fillFloat("asj", UIFigure::Operation::Add, 0, UIFigure::Color::White, 2, 360, 850, 25,
                             static_cast<f32>(globals->gimbal_communicator->aim_speed_change()) * 1000.0f);
  const auto dataLen1 = Referee0x301Prepare(globals->dataBox, 0, UIGroup1, robotID, robotID + 256);
  globals->referee_uart->Write(globals->dataBox, dataLen1, 500);
  const auto dataLen2 = Referee0x301Prepare(globals->dataBox, 0, UIGroup2, robotID, robotID + 256);
  globals->referee_uart->Write(globals->dataBox, dataLen2, 500);
  UICharacter UITextHeader1;
  UITextHeader1.character.fillCharacter("Hed", UIFigure::Operation::Add, 0, UIFigure::Color::Orange, 6, 55, 890, 24,
                                        29);
  memcpy(UITextHeader1.data, "GETTARGET\nSUGGESTFIRE", 21);
  const auto dataLen3 = Referee0x301Prepare(globals->dataBox, 0, UITextHeader1, robotID, robotID + 256);
  globals->referee_uart->Write(globals->dataBox, dataLen3, 500);
  UICharacter UITextHeader2;
  UITextHeader2.character.fillCharacter("Hed", UIFigure::Operation::Add, 0, UIFigure::Color::Orange, 6, 55, 890, 24,
                                        29);
  memcpy(UITextHeader2.data, "F R N U D X\nH N S", 17);
  const auto dataLen4 = Referee0x301Prepare(globals->dataBox, 0, UITextHeader2, robotID, robotID + 256);
  globals->referee_uart->Write(globals->dataBox, dataLen4, 500);
}

void UIInfantryEdit() {
  UIFigure7 UIGroup1;
  // 电容电压
  if (chassis->speed_mode_ == kHighSpeed) {
    UIGroup1.figure1.fillFloat("cms", UIFigure::Operation::Edit, 0, UIFigure::Color::Green, 5, 900, 270, 27,
                               static_cast<f32>(globals->super_cap->CapEnergy()) * 1000.0f);
  } else {
    UIGroup1.figure1.fillFloat("cms", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 5, 900, 270, 27,
                               static_cast<f32>(globals->super_cap->CapEnergy()) * 1000.0f);
  }
  // 弹速调节
  if (globals->gimbal_communicator->aim_speed_change() > 0) {
    UIGroup1.figure2.fillFloat("asj", UIFigure::Operation::Edit, 2, UIFigure::Color::Green, 2, 360, 850, 25,
                               static_cast<f32>(globals->gimbal_communicator->aim_speed_change()) * 1000.0f);
  } else if (globals->gimbal_communicator->aim_speed_change() < 0) {
    UIGroup1.figure2.fillFloat("asj", UIFigure::Operation::Edit, 2, UIFigure::Color::Pink, 2, 360, 850, 25,
                               static_cast<f32>(globals->gimbal_communicator->aim_speed_change()) * 1000.0f);
  } else {
    UIGroup1.figure2.fillFloat("asj", UIFigure::Operation::Edit, 2, UIFigure::Color::White, 2, 360, 850, 25,
                               static_cast<f32>(globals->gimbal_communicator->aim_speed_change()) * 1000.0f);
  }
  // // 自瞄模式
  if (globals->gimbal_communicator->get_target_flag() == 1) {
    UIGroup1.figure3.fillRec("gtf", UIFigure::Operation::Edit, 2, UIFigure::Color::Magenta, 3, 350, 807, 580, 768);
  } else {
    UIGroup1.figure3.fillRec("gtf", UIFigure::Operation::Edit, 2, UIFigure::Color::Magenta, 0, 350, 807, 580, 768);
  }

  if (globals->gimbal_communicator->suggest_fire_flag() == 1) {
    UIGroup1.figure4.fillRec("sff", UIFigure::Operation::Edit, 2, UIFigure::Color::Magenta, 3, 350, 766, 630, 731);
  } else {
    UIGroup1.figure4.fillRec("sff", UIFigure::Operation::Edit, 2, UIFigure::Color::Magenta, 0, 350, 766, 630, 731);
  }

  // 底盘模式
  if (chassis->ChassisMove_ == kFollow) {
    UIGroup1.figure5.fillRec("cmf", UIFigure::Operation::Edit, 2, UIFigure::Color::Magenta, 3, 1292, 808, 1322, 770);
  } else if (chassis->ChassisMove_ == kRotate) {
    UIGroup1.figure5.fillRec("cmf", UIFigure::Operation::Edit, 2, UIFigure::Color::Magenta, 3, 1342, 808, 1372, 770);
  } else if (chassis->ChassisMove_ == kNoForce) {
    UIGroup1.figure5.fillRec("cmf", UIFigure::Operation::Edit, 2, UIFigure::Color::Magenta, 3, 1392, 808, 1422, 770);
  } else {
    UIGroup1.figure5.fillRec("cmf", UIFigure::Operation::Edit, 2, UIFigure::Color::Magenta, 3, 1492, 808, 1522, 770);
  }
  if (chassis->buff_state_ == kDaFu) {
    UIGroup1.figure6.fillRec("bmf", UIFigure::Operation::Edit, 2, UIFigure::Color::Magenta, 3, 1492, 808, 1522, 770);
  } else if (chassis->buff_state_ == kXiaoFu) {
    UIGroup1.figure6.fillRec("bmf", UIFigure::Operation::Edit, 2, UIFigure::Color::Magenta, 3, 1542, 808, 1572, 770);
  } else {
    UIGroup1.figure6.fillRec("bmf", UIFigure::Operation::Edit, 2, UIFigure::Color::Magenta, 0, 1542, 808, 1572, 770);
  }

  // 底盘速度模式
  if (chassis->speed_mode_ == kHighSpeed) {
    UIGroup1.figure7.fillRec("smf", UIFigure::Operation::Edit, 2, UIFigure::Color::Magenta, 3, 1292, 766, 1322, 728);
  } else {
    UIGroup1.figure7.fillRec("smf", UIFigure::Operation::Edit, 2, UIFigure::Color::Magenta, 3, 1342, 766, 1372, 728);
  }
  if (globals->referee_data->data().power_heat_data.buffer_energy < 40) {
    UIGroup1.figure7.fillRec("smf", UIFigure::Operation::Edit, 2, UIFigure::Color::Magenta, 3, 1392, 766, 1422, 728);
  }
  const auto dataLen = Referee0x301Prepare(globals->dataBox, 0, UIGroup1, robotID, robotID + 256);
  globals->referee_uart->Write(globals->dataBox, dataLen, 500);
}