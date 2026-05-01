#include "vt03.hpp"
#include "main.hpp"

namespace rm::device {
Rxvt03::Rxvt03(rm::hal::SerialInterface &serial) : serial_(&serial) {
  static rm::hal::SerialRxCallbackFunction rx_callback =
      std::bind(&Rxvt03::RxCallback, this, std::placeholders::_1, std::placeholders::_2);
  this->serial_->AttachRxCallback(rx_callback);
}

void Rxvt03::Begin() { this->serial_->Begin(); }

void Rxvt03::RxCallback(const std::vector<u8> &data, u16 rx_len) {
  rx_callback_cnt++;
  rx_byte_cnt += rx_len;

  for (u16 i = 0; i < rx_len; i++) {
    last_byte = data.at(i);
    *gimbal->vt03 << data.at(i);
  }
}
}  // namespace rm::device