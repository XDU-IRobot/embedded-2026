#include "Referee.hpp"
#include "main.hpp"

namespace rm::device {
RxReferee::RxReferee(rm::hal::SerialInterface &serial, VT03 &image, Referee<RefereeRevision::kNewV120> &referee)
    : serial_(&serial), image_(image), referee_(referee) {
  this->serial_->AttachRxCallback([this](etl::span<const u8> data) { this->RxCallback(data); });
}

void RxReferee::Begin() { this->serial_->Start(); }

void RxReferee::RxCallback(etl::span<const u8> data) {
  for (const u8 byte : data) {
    *globals->image_data << byte;
    *globals->ref << byte;
  }
}
}  // namespace rm::device