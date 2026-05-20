#include "Referee.hpp"
#include "main.hpp"

namespace rm::device {
RxReferee::RxReferee(rm::hal::SerialInterface &serial) : serial_(&serial) {
  this->serial_->AttachRxCallback([this](etl::span<const u8> data) { this->RxCallback(data); });
}

void RxReferee::Begin() { this->serial_->Start(); }

void RxReferee::RxCallback(etl::span<const u8> data) {
  for (const auto byte : data) {
    *globals->image_data << byte;
  }
}
}  // namespace rm::device