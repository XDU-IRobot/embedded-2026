#pragma once

#include <algorithm>
#include <array>

#include <librm.hpp>

class GimbalCanFeedbackTxBridge final : public rm::device::CanDevice {
 public:
  static constexpr rm::u16 kTxStdId0 = 0x119;
  static constexpr rm::usize kPayloadSize = 4U;

  GimbalCanFeedbackTxBridge(rm::hal::CanInterface& can, const rm::device::HipnucImu* imu)
      : CanDevice(can, kTxStdId0), imu_(imu) {}

  void BindImuSource(const rm::device::HipnucImu* imu) { imu_ = imu; }

  bool QueueSend() {
    if (imu_ == nullptr) {
      return false;
    }

    EncodeFromImu(*imu_);
    can_->Write(kTxStdId0, tx_payload_.data(), tx_payload_.size());

    ReportStatus(kOk);
    return true;
  }

  void RxCallback(const rm::hal::CanFrame* msg) override { (void)msg; }

 private:
  static void PackI16BigEndian(rm::i16 value, rm::u8* out) {
    const auto raw = static_cast<rm::u16>(value);
    out[0] = static_cast<rm::u8>(raw >> 8);
    out[1] = static_cast<rm::u8>(raw);
  }

  static rm::i16 UnitToMilliI16(rm::f32 value) {
    const rm::f32 scaled = value * 1000.0f;
    const rm::f32 clamped = std::clamp(scaled, -32768.0f, 32767.0f);
    return static_cast<rm::i16>(clamped >= 0.0f ? (clamped + 0.5f) : (clamped - 0.5f));
  }

  // Layout (big-endian int16, milli-unit), compatible with GimbalCanFeedbackRxBridge:
  // [0..1] pitch, [2..3] yaw
  void EncodeFromImu(const rm::device::HipnucImu& imu) {
    PackI16BigEndian(UnitToMilliI16(imu.pitch()), &tx_payload_[0]);
    PackI16BigEndian(UnitToMilliI16(imu.yaw()), &tx_payload_[2]);
  }

  const rm::device::HipnucImu* imu_{nullptr};
  std::array<rm::u8, kPayloadSize> tx_payload_{};
};
