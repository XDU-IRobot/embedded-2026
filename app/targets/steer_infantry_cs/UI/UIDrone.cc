//
// Created by Jason on 26-5-27.
//

#include "UIDrone.hpp"
#include "librm.hpp"
#include "../main.hpp"

using namespace rm;
using namespace rm::device;

extern u16 robotID;

static float cmd_yaw = 0.0f;
static i16 cmd_ammo = 0;
static u8 cmd_fire = 0;

// 无人机显示英雄UI
void UIDroneHero_add() {
  UIFigure5 UIGroup1;
  UIGroup1.figure1.fillFloat("yaw", UIFigure::Operation::Add, 0, UIFigure::Color::Yellow, 5, 1460, 470, 20,
                             globals->subReferee->data().hero_2_drone.hero_yaw_angle * 1000);
  UIGroup1.figure2.fillFloat("pit", UIFigure::Operation::Add, 0, UIFigure::Color::Black, 5, 1620, 470, 20,
                             globals->subReferee->data().hero_2_drone.hero_pitch_angle * 1000);
  UIGroup1.figure3.fillIntegrate("adj", UIFigure::Operation::Add, 0, UIFigure::Color::Green, 5, 1650, 580, 22,
                                 globals->subReferee->data().hero_2_drone.hero_ammo_adjust);
  UIGroup1.figure4.fillFloat("isp", UIFigure::Operation::Add, 0, UIFigure::Color::Cyan, 5, 1460, 580, 22,
                             globals->subReferee->data().hero_2_drone.hero_initial_speed * 1000);
  UIGroup1.figure5.fillFloat("cya", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 5, 1460, 520, 24,
                             cmd_yaw * 1000);

  const auto dataLen = Referee0x301Prepare(globals->dataBox, 0, UIGroup1, robotID, robotID + 256);
  globals->referee_uart->Write(globals->dataBox, dataLen, 500);
}

void UIDroneHero_edit() {
  UIFigure5 UIGroup1;
  UIGroup1.figure1.fillFloat("yaw", UIFigure::Operation::Edit, 0, UIFigure::Color::Yellow, 5, 1460, 470, 20,
                             globals->subReferee->data().hero_2_drone.hero_yaw_angle * 1000);
  UIGroup1.figure2.fillFloat("pit", UIFigure::Operation::Edit, 0, UIFigure::Color::Black, 5, 1620, 470, 20,
                             globals->subReferee->data().hero_2_drone.hero_pitch_angle * 1000);
  UIGroup1.figure3.fillIntegrate("adj", UIFigure::Operation::Edit, 0, UIFigure::Color::Green, 5, 1650, 580, 22,
                                 globals->subReferee->data().hero_2_drone.hero_ammo_adjust);
  UIGroup1.figure4.fillFloat("isp", UIFigure::Operation::Edit, 0, UIFigure::Color::Cyan, 5, 1460, 580, 22,
                             globals->subReferee->data().hero_2_drone.hero_initial_speed * 1000);
  UIGroup1.figure5.fillFloat("cya", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 5, 1460, 520, 24,
                             cmd_yaw * 1000);
  const auto dataLen = Referee0x301Prepare(globals->dataBox, 0, UIGroup1, robotID, robotID + 256);
  globals->referee_uart->Write(globals->dataBox, dataLen, 500);
}

void D2H_func() {

  Drone2Hero d2h{cmd_yaw, cmd_ammo, cmd_fire};
  const auto size = Referee0x301Prepare(globals->dataBox, 0, d2h, robotID, robotID - 5);
  globals->referee_uart->Write(globals->dataBox, size, 500);
}