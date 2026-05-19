#include "gimbal.hpp"
using namespace rm;
void Layer0_func() {
  struct rm::device::UIFigure2 UIGroup0;
  if (Aimbot.AimbotState == 2||Aimbot.AimbotState == 4) {
    UIGroup0.figure1.fillRec("AimbotState", rm::device::UIFigure1::Operation::Add, 0, rm::device::UIFigure1::Color::Magenta, 3,
                                   766, 343, 1144, 721);
  }
  else {
    UIGroup0.figure1.fillRec("AimbotState", rm::device::UIFigure1::Operation::Add, 0, rm::device::UIFigure1::Color::Green, 3,
                               766, 343, 1144, 721);
  }

  float heat_percentage=gimbal->referee_data_buffer.data().power_heat_data.shooter_17mm_1_barrel_heat/gimbal->referee_data_buffer.data().robot_status.shooter_barrel_heat_limit==0?gimbal->referee_data_buffer.data().robot_status.shooter_barrel_heat_limit:1;
  if (heat_percentage>0.8f) {
    UIGroup0.figure2.fillArc("heat_state", rm::device::UIFigure1::Operation::Add, 0, rm::device::UIFigure1::Color::Magenta, 3,
                               961, 537, 0,360*heat_percentage,77, 77);
  }
  else if (0.8f>=heat_percentage&&heat_percentage>0.6f) {
    UIGroup0.figure2.fillArc("heat_state", rm::device::UIFigure1::Operation::Add, 0, rm::device::UIFigure1::Color::Yellow, 3,
                               961, 537, 0,360*heat_percentage,77, 77);
  }
  else{
    UIGroup0.figure2.fillArc("heat_state", rm::device::UIFigure1::Operation::Add, 0, rm::device::UIFigure1::Color::Green, 3,
                               961, 537, 0,360*heat_percentage,77, 77);
  }

  u8 len = device::Referee0x301Prepare(gimbal->dataBox, 0, UIGroup0, 0x001, 0x001 + 256);
  gimbal->refereeUart->Write(gimbal->dataBox, len);
}

void Layer1_func() {

}