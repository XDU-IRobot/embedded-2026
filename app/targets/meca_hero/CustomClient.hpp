#pragma once
#include "librm.hpp"
namespace rm::device {
#pragma pack(push, 1)
struct CClient_Tx_Pack {};

struct CClient_Rx_Pack {
  u8 _SOF;
  u8 ID;

  u8 payload[11];

  u8 _EOF;
};
#pragma pack(pop)

class CustomClient {
 public:
  CustomClient() = default;
  ~CustomClient() = default;
  static constexpr i16 CClient_USB_Tx_ID = 0x01;
  static constexpr i16 CClient_USB_Rx_ID = 0x02;

  static constexpr i16 CClientSOF = 0x55;
  static constexpr i16 CClientEOF = 0xFF;

  CClient_Tx_Pack CClient_Tx;
  CClient_Rx_Pack CClient_Rx;

  [[nodiscard]] u16 mouse_x() const { return _mouse_x; }
  [[nodiscard]] u16 mouse_y() const { return _mouse_y; }
  [[nodiscard]] u16 mouse_z() const { return _mouse_z; }
  [[nodiscard]] u8 mouse_left() const { return _mouse_left; }
  [[nodiscard]] u8 mouse_right() const { return _mouse_right; }
  [[nodiscard]] u16 key(DR16::Key key) const { return _key & static_cast<u16>(key); }
  [[nodiscard]] u8 mouse_mid() const { return _mouse_mid; }

  void Pack();
  void Send();
  void Unpack(uint8_t *rx_data, uint8_t Len);

 private:
  u16 _mouse_x;
  u16 _mouse_y;
  u16 _mouse_z;
  u16 _key;
  u8 _mouse_left;
  u8 _mouse_right;
  u8 _mouse_mid;
  u8 _tx_buff[14];
};
}  // namespace rm::device