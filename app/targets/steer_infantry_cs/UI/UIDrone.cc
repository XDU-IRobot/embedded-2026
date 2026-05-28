//
// Created by Jason on 26-5-27.
//

#include "UIDrone.hpp"
#include "librm.hpp"
#include "../main.hpp"

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

  UIGroup1.figure1.fillFloat("cms", UIFigure::Operation::Add, 0, UIFigure::Color::Green, 27, 900, 270, 2,
                             static_cast<f32>(globals->super_cap->CapEnergy()) * 1000.0f);
  UIGroup1.figure1.fillFloat("asj", UIFigure::Operation::Add, 0, UIFigure::Color::White, 25, 360, 850, 2,
                             static_cast<f32>(globals->gimbal_communicator->aim_speed_change()) * 1000.0f);

  Char_Draw(&aimbot, (char *)"aim", UI_Graph_ADD, 1, UI_Color_Green, 25, 22, 2, 360, 800,
            (char *)"GETTARGET\nSUGGESTFIRE");
  Char_Draw(&mode, (char *)"mod", UI_Graph_ADD, 1, UI_Color_Green, 25, 20, 2, 1300, 800, (char *)"F R N U D X\nH N S");
  UIGroup1.figure1.fillFloat("yaw", UIFigure::Operation::Add, 0, UIFigure::Color::Yellow, 5, 1460, 470, 20,
                             globals->subReferee->data().hero_2_drone.hero_yaw_angle * 1000);
  UIGroup1.figure2.fillFloat("pit", UIFigure::Operation::Add, 0, UIFigure::Color::Black, 5, 1620, 470, 20,
                             globals->subReferee->data().hero_2_drone.hero_pitch_angle * 1000);
  UIGroup1.figure3.fillIntegrate("adj", UIFigure::Operation::Add, 0, UIFigure::Color::Green, 5, 1650, 580, 22,
                                 globals->subReferee->data().hero_2_drone.hero_ammo_adjust);
  UIGroup1.figure4.fillFloat("isp", UIFigure::Operation::Add, 0, UIFigure::Color::Cyan, 5, 1460, 580, 22,
                             globals->subReferee->data().hero_2_drone.hero_initial_speed * 1000);

  const auto dataLen = Referee0x301Prepare(globals->dataBox, 0, UIGroup1, robotID, robotID + 256);
  globals->referee_uart->Write(globals->dataBox, dataLen, 500);
}

void UIInfantryEdit() {
  UIFigure5 UIGroup1;
  UIGroup1.figure1.fillFloat("yaw", UIFigure::Operation::Edit, 0, UIFigure::Color::Yellow, 5, 1460, 470, 20,
                             globals->subReferee->data().hero_2_drone.hero_yaw_angle * 1000);
  UIGroup1.figure2.fillFloat("pit", UIFigure::Operation::Edit, 0, UIFigure::Color::Black, 5, 1620, 470, 20,
                             globals->subReferee->data().hero_2_drone.hero_pitch_angle * 1000);
  UIGroup1.figure3.fillIntegrate("adj", UIFigure::Operation::Edit, 0, UIFigure::Color::Green, 5, 1650, 580, 22,
                                 globals->subReferee->data().hero_2_drone.hero_ammo_adjust);
  UIGroup1.figure4.fillFloat("isp", UIFigure::Operation::Edit, 0, UIFigure::Color::Cyan, 5, 1460, 580, 22,
                             globals->subReferee->data().hero_2_drone.hero_initial_speed * 1000);
  const auto dataLen = Referee0x301Prepare(globals->dataBox, 0, UIGroup1, robotID, robotID + 256);
  globals->referee_uart->Write(globals->dataBox, dataLen, 500);
}