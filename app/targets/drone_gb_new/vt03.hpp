#ifndef BOARDC_VT03_H
#define BOARDC_VT03_H

#include <librm.hpp>

using namespace rm;

namespace rm::device {
class Rxvt03 : public Device {
 public:
  Rxvt03() = delete;
  explicit Rxvt03(hal::SerialInterface &serial);
  void Begin();
  void RxCallback(const std::vector<u8> &data, u16 rx_len);

  uint32_t rx_callback_cnt = 0;
  uint32_t rx_byte_cnt = 0;
  uint8_t last_byte = 0;

 private:
  hal::SerialInterface *serial_;
};
}  // namespace rm::device
#endif  // BOARDC_VT03_H
