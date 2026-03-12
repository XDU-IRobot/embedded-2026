#include "Aimbot.h"

#include "librm.hpp"
#include "usbd_cdc_if.h"

// using namespace rm::modules::algorithm;

extern uint32_t System_time;
extern float yaw, pitch, roll;
// extern Quaternion myquaternion;

Aimbot aimbot;
extern uint8_t UserTxBuf[64];
void Aimbot::Send() {
  memcpy(UserTxBuf, &USB_Tx, 33);
  CDC_Transmit_FS((uint8_t *)&UserTxBuf, 33);
}

void Aimbot::Prepare() {
  this->USB_Tx._SOF = Aimbot::USBSOF;
  this->USB_Tx.ID = Aimbot::Aimbot_USB_Tx_ID;
  this->USB_Tx.TimeStamp = System_time;
  this->USB_Tx.robot_id = globals->ref.data().robot_status.robot_id;
  // refereedata.getrobotid();
  this->USB_Tx.AimbotState = 0;
  // globals->ref.data().;
  // (uint8_t)mygimbal.getAimbotState();
  this->USB_Tx.q0 = globals->ahrs.quaternion().w;
  // myquaternion.w;
  this->USB_Tx.q1 = globals->ahrs.quaternion().x;
  this->USB_Tx.q2 = globals->ahrs.quaternion().y;
  this->USB_Tx.q3 = globals->ahrs.quaternion().z;
  this->USB_Tx._EOF = Aimbot::USBEOF;
}

void Aimbot::Receive(uint8_t *rx_data, uint8_t Len) {
  if (Len == sizeof(Aimbot::USB_Rx)) {
    if (rx_data[0] == USBSOF && rx_data[Len - 1] == USBEOF) {
      switch (rx_data[1]) {
        case Aimbot_USB_Rx_ID:
          memcpy(&USB_Rx, rx_data, Len);
          aimbot_TO = 500;
          break;
        default:
          break;
      }
    }
  }
}