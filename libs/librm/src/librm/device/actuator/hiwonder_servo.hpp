//
// Created by 34236 on 2026/3/7.
//

#ifndef BOARDA_HIWONDER_SERVO_HPP
#define BOARDA_HIWONDER_SERVO_HPP

#include <vector>

#include "librm/core/typedefs.hpp"
#include "librm/hal/serial.hpp"
#include "librm/device/device.hpp"

namespace rm::device {

class HiwonderServo : public Device {
public:
    HiwonderServo() = delete;
    explicit HiwonderServo(rm::hal::SerialInterface &serial);

    void Begin();
    void SetServoAngle(rm::u16 pos, rm::u8 id, rm::u16 time) const;

private:
    void RxCallback(const std::vector<rm::u8> &data, rm::u16 rx_len);
    rm::hal::SerialInterface *serial_{nullptr};
};

}  // namespace rm::device

#endif //BOARDA_HIWONDER_SERVO_HPP
