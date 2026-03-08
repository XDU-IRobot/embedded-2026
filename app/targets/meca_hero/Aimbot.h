#ifndef AIMBOT_H
#define AIMBOT_H

#include "librm.hpp"

#include "main.hpp"

#include "usbd_cdc_if.h"

using namespace rm;

// 设置字节对齐为1字节
#pragma pack(push, 1)

// USB通信,自瞄发送包,25字节
struct Aimbot_USB_Transmit {
  uint8_t _SOF;//1
  uint8_t ID;//1

  uint32_t TimeStamp;//4

  f32 q0;//4
  f32 q1;//4
  f32 q2;//4
  f32 q3;//4

  uint8_t robot_id;
  uint8_t AimbotState;

  f32 reserved1;
  f32 reserved2;

  uint8_t _EOF;
};

// USB通信,自瞄接收包,25字节
struct Aimbot_USB_Receive {
  uint8_t _SOF;
  uint8_t ID;

  uint8_t AimbotState;

  uint8_t AimbotTarget;

  f32 PitchRelativeAngle;
  f32 YawRelativeAngle;

  f32 TargetPitchSpeed;
  f32 TargetYawSpeed;

  uint32_t SystemTimer;

  uint8_t _EOF;
};

// 恢复默认字节对齐
#pragma pack(pop)

class Aimbot {
 public:
  static constexpr i16 Aimbot_USB_Tx_ID = 0x01;
  static constexpr i16 Aimbot_USB_Rx_ID = 0x02;

  static constexpr i16 USBSOF = 0x55;
  static constexpr i16 USBEOF = 0xFF;

  Aimbot_USB_Transmit USB_Tx;
  Aimbot_USB_Receive USB_Rx;

  Aimbot() = default;
  ~Aimbot() = default;

  void Prepare();
  void Send();
  void Receive(uint8_t *rx_data, uint8_t Len);

  bool getstate() { return USB_Rx.AimbotState & 0x01; }
  bool getfire() { return USB_Rx.AimbotState & ((u8)1 << 1); }
  // bool isonline() { return aimofflinecount <= 2000; }
};

extern Aimbot aimbot;
#endif /* AIMBOT_H */