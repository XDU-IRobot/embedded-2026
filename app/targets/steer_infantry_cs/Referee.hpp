#ifndef REFEREE_HPP
#define REFEREE_HPP

#include <librm.hpp>

using namespace rm;

namespace rm::device {
class RxReferee : public Device {
 public:
  RxReferee() = delete;

  explicit RxReferee(rm::hal::SerialInterface &serial, Referee<RefereeRevision::kNewV120> &referee);

  void Begin();

  // 改为与 Uart::AttachRxCallback 兼容的签名
  void RxCallback(etl::span<const u8> data);

 private:
  rm::hal::SerialInterface *serial_;
  rm::device::Referee<RefereeRevision::kNewV120> &referee_;
};
}  // namespace rm::device

#endif  // REFEREE_HPP