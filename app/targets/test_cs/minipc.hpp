
#ifndef GIMBAL_TEST_MINIPC_HPP
#define GIMBAL_TEST_MINIPC_HPP

#include <librm.hpp>

#include "usbd_cdc_if.h"

class MiniPC {
public:
  struct __attribute__((packed)) VisionToGimbal {
    uint8_t SOF;
    uint8_t ID;
    float x_speed;
    float y_speed;
    float yaw_speed;
    float w_speed;
    bool scan_mod_type;
    uint8_t Eof;
  } rx_buffer_{};

  static_assert(sizeof(VisionToGimbal) <= 64);  ///< 确保能在USBCDC的一帧里发出去

public:
  void RxCallback(uint8_t *buf, uint32_t len) {
    if (len == sizeof(VisionToGimbal)) {
      const auto *rx_data = reinterpret_cast<const VisionToGimbal *>(buf);
      rx_buffer_ = *rx_data;
    }
  }
};


#endif  // GIMBAL_TEST_MINIPC_HPP
