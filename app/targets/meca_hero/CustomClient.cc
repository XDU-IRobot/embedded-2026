#include "CustomClient.hpp"

#include <cstdint>

#include "cstring"
#include "usart.h"

namespace rm::device {
void CustomClient::Unpack(uint8_t *rx_data, uint8_t Len) {
  if (Len == sizeof(CClient_Rx)) {
    if (rx_data[0] == CClientSOF && rx_data[Len - 1] == CClientEOF) {
      switch (rx_data[1]) {
        case CClient_USB_Rx_ID:
          memcpy(&CClient_Rx, rx_data, Len);
          _mouse_x = static_cast<uint16_t>(CClient_Rx.payload[0] | CClient_Rx.payload[1] << 8);
          _mouse_y = static_cast<uint16_t>(CClient_Rx.payload[2] | CClient_Rx.payload[3] << 8);
          _mouse_z = static_cast<uint16_t>(CClient_Rx.payload[4] | CClient_Rx.payload[5] << 8);
          _mouse_left = static_cast<uint8_t>(CClient_Rx.payload[6]);
          _mouse_right = static_cast<uint8_t>(CClient_Rx.payload[7]);
          _key = static_cast<uint16_t>(CClient_Rx.payload[8] | CClient_Rx.payload[9] << 8);
          _mouse_mid = static_cast<uint8_t>(CClient_Rx.payload[10]);
          tt = _mouse_x << 16 | _mouse_y;
          VOFA_Prepare_Package(tt, test);
          VOFA_Send_JustFloat_DMA(&huart1, test);
          break;
        default:
          _mouse_x = 0;
          _mouse_y = 0;
          _mouse_z = 0;
          _mouse_left = 0;
          _mouse_right = 0;
          _mouse_mid = 0;
          _key = 0;
          break;
      }
    } else {
      _mouse_x = 0;
      _mouse_y = 0;
      _mouse_z = 0;
      _mouse_left = 0;
      _mouse_right = 0;
      _mouse_mid = 0;
      _key = 0;
    }
  } else {
    _mouse_x = 0;
    _mouse_y = 0;
    _mouse_z = 0;
    _mouse_left = 0;
    _mouse_right = 0;
    _mouse_mid = 0;
    _key = 0;
  }
}
}  // namespace rm::device