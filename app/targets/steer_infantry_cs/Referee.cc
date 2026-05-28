#include "Referee.hpp"
#include "main.hpp"

namespace rm::device {
RxReferee::RxReferee(rm::hal::SerialInterface &serial, Referee<RefereeRevision::kNewV120> &referee)
    : serial_(&serial), referee_(referee) {
  this->serial_->AttachRxCallback([this](etl::span<const u8> data) { this->RxCallback(data); });
}

void RxReferee::Begin() { this->serial_->Start(); }

void RxReferee::RxCallback(etl::span<const u8> data) {
  for (const u8 byte : data) {
    *globals->referee_data << byte;
  }
}
}  // namespace rm::device