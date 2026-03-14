#pragma once
#include "VOFA.hpp"
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
  static constexpr uint8_t CClient_USB_Tx_ID = 0x01;
  static constexpr uint8_t CClient_USB_Rx_ID = 0x02;

  static constexpr uint8_t CClientSOF = 0x55;
  static constexpr uint8_t CClientEOF = 0xFF;

  CClient_Tx_Pack CClient_Tx;
  CClient_Rx_Pack CClient_Rx;
  Vofa_TxFrame test;

  [[nodiscard]] uint16_t mouse_x() const { return _mouse_x; }
  [[nodiscard]] uint16_t mouse_y() const { return _mouse_y; }
  [[nodiscard]] uint16_t mouse_z() const { return _mouse_z; }
  [[nodiscard]] bool mouse_left() const { return _mouse_left; }
  [[nodiscard]] bool mouse_right() const { return _mouse_right; }
  [[nodiscard]] bool key(DR16::Key key) const { return _key & static_cast<uint16_t>(key); }
  [[nodiscard]] uint8_t mouse_mid() const { return _mouse_mid; }

  void Pack();
  void Send();
  void Unpack(uint8_t *rx_data, uint8_t Len);

 private:
  u16 _mouse_x;
  u16 _mouse_y;
  u16 _mouse_z;
  u16 _key;
  bool _mouse_left;
  bool _mouse_right;
  u8 _mouse_mid;
  u8 _tx_buff[14];
  float tt;
};
}  // namespace rm::device