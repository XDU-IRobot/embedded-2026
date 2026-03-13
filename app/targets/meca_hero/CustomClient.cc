#include "CustomClient.hpp"

#include <cstdint>

#include "cstring"

namespace rm::device {
void CustomClient::Unpack(uint8_t *rx_data, uint8_t Len) {
  if (Len == sizeof(CClient_Rx)) {
    if (rx_data[0] == CClientSOF && rx_data[Len - 1] == CClientEOF) {
      switch (rx_data[1]) {
        case CClient_USB_Tx_ID:
          memcpy(&CClient_Rx, rx_data, Len);
          break;
        default:
          break;
      }
    }
  }
  _mouse_x = static_cast<uint16_t>(CClient_Rx.payload[0] | CClient_Rx.payload[1] << 8);
  _mouse_y = static_cast<uint16_t>(CClient_Rx.payload[2] | CClient_Rx.payload[3] << 8);
  _mouse_z = static_cast<uint16_t>(CClient_Rx.payload[4] | CClient_Rx.payload[5] << 8);
  _mouse_left = static_cast<uint8_t>(CClient_Rx.payload[6]);
  _mouse_right = static_cast<uint8_t>(CClient_Rx.payload[7]);
  _key = static_cast<uint16_t>(CClient_Rx.payload[8] << 8 | CClient_Rx.payload[9] << 8);
  _mouse_mid = static_cast<uint8_t>(CClient_Rx.payload[10]);
}
}  // namespace rm::device