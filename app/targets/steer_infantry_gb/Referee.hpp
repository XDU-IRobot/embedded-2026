#ifndef REFEREE_HPP
#define REFEREE_HPP

#include <librm.hpp>

using namespace rm;

namespace rm::device {
class RxReferee : public Device {
 public:
  RxReferee() = delete;

  explicit RxReferee(rm::hal::SerialInterface &serial);

  void Begin();

  void RxCallback(etl::span<const u8> data);

 private:
  rm::hal::SerialInterface *serial_;
};
}  // namespace rm::device

#endif  // REFEREE_HPP
