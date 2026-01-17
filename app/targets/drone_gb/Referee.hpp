#ifndef BOARDC_REFEREE_HPP
#define BOARDC_REFEREE_HPP

#include <librm.hpp>
#include "Gimbal.hpp"

using namespace rm;

namespace rm::device {
class RxReferee : public Device {
 public:
  RxReferee() = delete;
  explicit RxReferee(hal::SerialInterface &serial);
  void Begin();
  void RxCallback(const std::vector<u8> &data, u16 rx_len);

 private:
  hal::SerialInterface *serial_;
};
}  // namespace rm::device

#endif  // BOARDC_REFEREE_HPP