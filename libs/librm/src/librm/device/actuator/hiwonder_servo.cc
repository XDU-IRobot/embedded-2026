//
// Created by 34236 on 2026/3/7.
//

#include "hiwonder_servo.hpp"

namespace rm::device {

hiwonder_servo::hiwonder_servo(rm::hal::SerialInterface &serial) : serial_(&serial) {
    static rm::hal::SerialRxCallbackFunction rx_callback =
            std::bind(&hiwonder_servo::RxCallback, this, std::placeholders::_1, std::placeholders::_2);
    this->serial_->AttachRxCallback(rx_callback);
}

void hiwonder_servo::Begin() const {
    this->serial_->Begin();
}

void hiwonder_servo::SetServoAngle(rm::u16 pos, rm::u8 id, rm::u16 time) const {
    rm::u8 buf[10];
    buf[0] = 0x55;
    buf[1] = 0x55;
    buf[2] = id;
    buf[3] = 7;
    buf[4] = 1;
    buf[5] = static_cast<rm::u8>(pos & 0xff);
    buf[6] = static_cast<rm::u8>(pos >> 8);
    buf[7] = static_cast<rm::u8>(time & 0xff);
    buf[8] = static_cast<rm::u8>(time >> 8);
    buf[9] = static_cast<rm::u8>(~(buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7] + buf[8]));

    this->serial_->Write(buf, 10);
}

void hiwonder_servo::RxCallback(const std::vector<rm::u8> &data, rm::u16 rx_len) {
    // Reserved for servo feedback parsing.
    (void)data;
    (void)rx_len;
}

}  // namespace rm::device
