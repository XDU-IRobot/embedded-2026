//
// Created by Jason on 26-5-27.
//

#include "UIuser1.hpp"
#include "librm.hpp"
#include "../main.hpp"
#include "../subReferee/protocol_user.hpp"
#include "../subReferee/referee_user.hpp"

using namespace rm;
using namespace rm::device;

struct EmyRobotHP {
  u16 hero_1_HP;
  u16 engineer_2_HP;
  u16 standard_3_HP;
  u16 standard_4_HP;
  u16 sentry_7_HP;
} robotHP;

u8 dataBox[128] = {};  // 发送缓冲区
u16 robotID = 106;     // 在主函数中设置为裁判系统的机器人ID

void UITextHeaderRobotRed_add() {
  UICharacter UITextHeader;
  UITextHeader.character.fillCharacter("Hed", UIFigure::Operation::Add, 0, UIFigure::Color::Orange, 6, 1170, 890, 24,
                                       29);
  memcpy(UITextHeader.data, "HRO1 ENG2 STD3 STD4 DRO6 SEN7", 29);
  const auto dataLen = Referee0x301Prepare(dataBox, 0, UITextHeader, robotID, robotID + 256);
  globals->referee_uart->Write(dataBox, dataLen, 500);
}

void UITextHeaderHPRed_add() {
  UIFigure7 UITextHeaderHP;
  UITextHeaderHP.figure1.fillIntegrate("HP1", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 1170, 850, 20,
                                       0);
  UITextHeaderHP.figure2.fillIntegrate("HP2", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 1290, 850, 20,
                                       0);
  UITextHeaderHP.figure3.fillIntegrate("HP3", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 1410, 850, 20,
                                       0);
  UITextHeaderHP.figure4.fillIntegrate("HP4", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 1530, 850, 20,
                                       0);
  UITextHeaderHP.figure5.fillIntegrate("HP5", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 1770, 850, 20,
                                       0);
  UITextHeaderHP.figure6.fillIntegrate("sco", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 988, 900, 18,
                                       0);
  UITextHeaderHP.figure7.fillIntegrate("cco", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 988, 865, 24, 0);
  const auto dataLen = Referee0x301Prepare(dataBox, 0, UITextHeaderHP, robotID, robotID + 256);
  globals->referee_uart->Write(dataBox, dataLen, 500);
}

void UITextHeaderHPRed_edit() {
  static UIFigure7 UITextHeaderHP;
  // memcpy(&robotHP, globals->referee_data->data().robot_custom_data_3.data,10);
  if (globals->subReferee->data().enemy_robot_buff.hero.defense >= 100) {
    UITextHeaderHP.figure1.fillIntegrate("HP1", UIFigure::Operation::Edit, 0, UIFigure::Color::Yellow, 4, 1170, 850, 20,
                                         robotHP.hero_1_HP);
  } else {
    UITextHeaderHP.figure1.fillIntegrate("HP1", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 4, 1170, 850,
                                         20, robotHP.hero_1_HP);
  }

  if (globals->subReferee->data().enemy_robot_buff.engineer.defense >= 100) {
    UITextHeaderHP.figure2.fillIntegrate("HP2", UIFigure::Operation::Edit, 0, UIFigure::Color::Yellow, 4, 1290, 850, 20,
                                         robotHP.engineer_2_HP);
  } else {
    UITextHeaderHP.figure2.fillIntegrate("HP2", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 4, 1290, 850,
                                         20, robotHP.engineer_2_HP);
  }

  if (globals->subReferee->data().enemy_robot_buff.infantry3.defense >= 100) {
    UITextHeaderHP.figure3.fillIntegrate("HP3", UIFigure::Operation::Edit, 0, UIFigure::Color::Yellow, 4, 1410, 850, 20,
                                         robotHP.standard_3_HP);
  } else {
    UITextHeaderHP.figure3.fillIntegrate("HP3", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 4, 1410, 850,
                                         20, robotHP.standard_3_HP);
  }

  if (globals->subReferee->data().enemy_robot_buff.infantry4.defense >= 100) {
    UITextHeaderHP.figure4.fillIntegrate("HP4", UIFigure::Operation::Edit, 0, UIFigure::Color::Yellow, 4, 1530, 850, 20,
                                         robotHP.standard_4_HP);
  } else {
    UITextHeaderHP.figure4.fillIntegrate("HP4", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 4, 1530, 850,
                                         20, robotHP.standard_4_HP);
  }

  if (globals->subReferee->data().enemy_robot_buff.sentry.defense >= 100) {
    UITextHeaderHP.figure5.fillIntegrate("HP5", UIFigure::Operation::Edit, 0, UIFigure::Color::Yellow, 4, 1770, 850, 20,
                                         robotHP.sentry_7_HP);
  } else {
    UITextHeaderHP.figure5.fillIntegrate("HP5", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 4, 1770, 850,
                                         20, robotHP.sentry_7_HP);
  }
  UITextHeaderHP.figure6.fillIntegrate("sco", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 4, 988, 900, 18,
                                       globals->subReferee->data().enemy_gold_coin_RFID.enemy_gold_total);
  UITextHeaderHP.figure7.fillIntegrate("cco", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 988, 865, 24,
                                       globals->subReferee->data().enemy_gold_coin_RFID.enemy_gold_remaining);
  const auto dataLen = Referee0x301Prepare(dataBox, 0, UITextHeaderHP, robotID, robotID + 256);
  globals->referee_uart->Write(dataBox, dataLen, 500);
}

void UITextHeaderAllowRed_add() {
  UIFigure5 UITextHeaderAllow;
  UITextHeaderAllow.figure1.fillIntegrate("AL1", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 1170, 810, 16,
                                          0);
  UITextHeaderAllow.figure2.fillIntegrate("AL2", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 1410, 810, 16,
                                          0);
  UITextHeaderAllow.figure3.fillIntegrate("AL3", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 1530, 810, 16,
                                          0);
  UITextHeaderAllow.figure4.fillIntegrate("AL4", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 1650, 810, 16,
                                          0);
  UITextHeaderAllow.figure5.fillIntegrate("AL5", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 1770, 810, 16,
                                          0);
  const auto dataLen = Referee0x301Prepare(dataBox, 0, UITextHeaderAllow, robotID, robotID + 256);
  globals->referee_uart->Write(dataBox, dataLen, 500);
}

void UITextHeaderAllowRed_edit() {
  static UIFigure5 UITextHeaderAllow;
  UITextHeaderAllow.figure1.fillIntegrate(
      "AL1", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 1170, 810, 16,
      globals->subReferee->data().enemy_robot_projectile_allowance.hero_1_projectile_allowance);
  UITextHeaderAllow.figure2.fillIntegrate(
      "AL2", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 1410, 810, 16,
      globals->subReferee->data().enemy_robot_projectile_allowance.standard_3_projectile_allowance);
  UITextHeaderAllow.figure3.fillIntegrate(
      "AL3", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 1530, 810, 16,
      globals->subReferee->data().enemy_robot_projectile_allowance.standard_4_projectile_allowance);
  UITextHeaderAllow.figure4.fillIntegrate(
      "AL4", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 1650, 810, 16,
      globals->subReferee->data().enemy_robot_projectile_allowance.drone_6_projectile_allowance);
  UITextHeaderAllow.figure5.fillIntegrate(
      "AL5", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 1770, 810, 16,
      globals->subReferee->data().enemy_robot_projectile_allowance.sentry_7_projectile_allowance);
  const auto dataLen = Referee0x301Prepare(dataBox, 0, UITextHeaderAllow, robotID, robotID + 256);
  globals->referee_uart->Write(dataBox, dataLen, 500);
}

void UITextHeaderRobotBlue_add() {
  UICharacter UITextHeader;
  UITextHeader.character.fillCharacter("Hed", UIFigure::Operation::Add, 0, UIFigure::Color::Orange, 6, 55, 890, 24, 29);
  memcpy(UITextHeader.data, "SEN7 DRO6 STD4 STD3 ENG2 HRO1", 29);
  const auto dataLen = Referee0x301Prepare(dataBox, 0, UITextHeader, robotID, robotID + 256);
  globals->referee_uart->Write(dataBox, dataLen, 500);
}

void UITextHeaderHPBlue_add() {
  UIFigure7 UITextHeaderHP;
  UITextHeaderHP.figure1.fillIntegrate("HP1", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 55, 850, 20, 0);
  UITextHeaderHP.figure2.fillIntegrate("HP2", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 295, 850, 20,
                                       0);
  UITextHeaderHP.figure3.fillIntegrate("HP3", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 415, 850, 20,
                                       0);
  UITextHeaderHP.figure4.fillIntegrate("HP4", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 535, 850, 20,
                                       0);
  UITextHeaderHP.figure5.fillIntegrate("HP5", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 655, 850, 20,
                                       0);
  UITextHeaderHP.figure6.fillIntegrate("sco", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 860, 900, 18,
                                       0);
  UITextHeaderHP.figure7.fillIntegrate("cco", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 860, 865, 24, 0);
  const auto dataLen = Referee0x301Prepare(dataBox, 0, UITextHeaderHP, robotID, robotID + 256);
  globals->referee_uart->Write(dataBox, dataLen, 500);
}

void UITextHeaderHPBlue_edit() {
  static UIFigure7 UITextHeaderHP;
  memcpy(&robotHP, globals->referee_data->data().robot_custom_data_3.data, 10);
  if (globals->subReferee->data().enemy_robot_buff.hero.defense >= 100) {
    UITextHeaderHP.figure1.fillIntegrate("HP1", UIFigure::Operation::Edit, 0, UIFigure::Color::Yellow, 4, 55, 850, 20,
                                         robotHP.sentry_7_HP);
  } else {
    UITextHeaderHP.figure1.fillIntegrate("HP1", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 4, 55, 850, 20,
                                         robotHP.sentry_7_HP);
  }

  if (globals->subReferee->data().enemy_robot_buff.engineer.defense >= 100) {
    UITextHeaderHP.figure2.fillIntegrate("HP2", UIFigure::Operation::Edit, 0, UIFigure::Color::Yellow, 4, 295, 850, 20,
                                         robotHP.standard_4_HP);
  } else {
    UITextHeaderHP.figure2.fillIntegrate("HP2", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 4, 295, 850, 20,
                                         robotHP.standard_4_HP);
  }

  if (globals->subReferee->data().enemy_robot_buff.infantry3.defense >= 100) {
    UITextHeaderHP.figure3.fillIntegrate("HP3", UIFigure::Operation::Edit, 0, UIFigure::Color::Yellow, 4, 415, 850, 20,
                                         robotHP.standard_3_HP);
  } else {
    UITextHeaderHP.figure3.fillIntegrate("HP3", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 4, 415, 850, 20,
                                         robotHP.standard_3_HP);
  }

  if (globals->subReferee->data().enemy_robot_buff.infantry4.defense >= 100) {
    UITextHeaderHP.figure4.fillIntegrate("HP4", UIFigure::Operation::Edit, 0, UIFigure::Color::Yellow, 4, 535, 850, 20,
                                         robotHP.engineer_2_HP);
  } else {
    UITextHeaderHP.figure4.fillIntegrate("HP4", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 4, 535, 850, 20,
                                         robotHP.engineer_2_HP);
  }

  if (globals->subReferee->data().enemy_robot_buff.sentry.defense >= 100) {
    UITextHeaderHP.figure5.fillIntegrate("HP5", UIFigure::Operation::Edit, 0, UIFigure::Color::Yellow, 4, 655, 850, 20,
                                         robotHP.hero_1_HP);
  } else {
    UITextHeaderHP.figure5.fillIntegrate("HP5", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 4, 655, 850, 20,
                                         robotHP.hero_1_HP);
  }
  UITextHeaderHP.figure6.fillIntegrate("sco", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 4, 860, 900, 18,
                                       globals->subReferee->data().enemy_gold_coin_RFID.enemy_gold_total);
  UITextHeaderHP.figure7.fillIntegrate("cco", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 860, 865, 24,
                                       globals->subReferee->data().enemy_gold_coin_RFID.enemy_gold_remaining);
  const auto dataLen = Referee0x301Prepare(dataBox, 0, UITextHeaderHP, robotID, robotID + 256);
  globals->referee_uart->Write(dataBox, dataLen, 500);
}

void UITextHeaderAllowBlue_add() {
  UIFigure5 UITextHeaderAllow;
  UITextHeaderAllow.figure1.fillIntegrate("AL1", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 55, 810, 16,
                                          0);
  UITextHeaderAllow.figure2.fillIntegrate("AL2", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 175, 810, 16,
                                          0);
  UITextHeaderAllow.figure3.fillIntegrate("AL3", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 295, 810, 16,
                                          0);
  UITextHeaderAllow.figure4.fillIntegrate("AL4", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 415, 810, 16,
                                          0);
  UITextHeaderAllow.figure5.fillIntegrate("AL5", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 655, 810, 16,
                                          0);
  const auto dataLen = Referee0x301Prepare(dataBox, 0, UITextHeaderAllow, robotID, robotID + 256);
  globals->referee_uart->Write(dataBox, dataLen, 500);
}

void UITextHeaderAllowBlue_edit() {
  static UIFigure5 UITextHeaderAllow;
  UITextHeaderAllow.figure1.fillIntegrate(
      "AL1", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 55, 810, 16,
      globals->subReferee->data().enemy_robot_projectile_allowance.hero_1_projectile_allowance);
  UITextHeaderAllow.figure2.fillIntegrate(
      "AL2", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 175, 810, 16,
      globals->subReferee->data().enemy_robot_projectile_allowance.standard_3_projectile_allowance);
  UITextHeaderAllow.figure3.fillIntegrate(
      "AL3", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 295, 810, 16,
      globals->subReferee->data().enemy_robot_projectile_allowance.standard_4_projectile_allowance);
  UITextHeaderAllow.figure4.fillIntegrate(
      "AL4", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 415, 810, 16,
      globals->subReferee->data().enemy_robot_projectile_allowance.drone_6_projectile_allowance);
  UITextHeaderAllow.figure5.fillIntegrate(
      "AL5", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 655, 810, 16,
      globals->subReferee->data().enemy_robot_projectile_allowance.sentry_7_projectile_allowance);
  const auto dataLen = Referee0x301Prepare(dataBox, 0, UITextHeaderAllow, robotID, robotID + 256);
  globals->referee_uart->Write(dataBox, dataLen, 500);
}

using namespace rm;