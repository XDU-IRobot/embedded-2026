#pragma once

#include <algorithm>
#include <array>

#include <librm.hpp>

class GimbalCanFeedbackTxBridge final : public rm::device::CanDevice {
 public:
  static constexpr rm::u16 kTxStdId0 = 0x119;
  static constexpr rm::u16 kTxStdId1 = 0x11A;
  static constexpr rm::u16 kTxStdId2 = 0x11B;
  static constexpr rm::usize kFullPayloadSize = 18U;
  static constexpr rm::usize kCanFrameSize = 8U;

  GimbalCanFeedbackTxBridge(rm::hal::CanInterface& can, const rm::device::HipnucImu* imu)
      : CanDevice(can, kTxStdId0, kTxStdId1, kTxStdId2), imu_(imu) {}

  void BindImuSource(const rm::device::HipnucImu* imu) { imu_ = imu; }

  bool QueueSend() {
    if (imu_ == nullptr) {
      return false;
    }

    EncodeFromImu(*imu_);

    can_->Write(kTxStdId0, tx_payload_.data(), kCanFrameSize);
    can_->Write(kTxStdId1, &tx_payload_[kCanFrameSize], kCanFrameSize);

    tx_tail_.fill(0);
    tx_tail_[0] = tx_payload_[16];
    tx_tail_[1] = tx_payload_[17];
    can_->Write(kTxStdId2, tx_tail_.data(), tx_tail_.size());

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

  // Original 18-byte layout (big-endian int16, milli-unit):
  // [0..1] roll, [2..3] pitch, [4..5] yaw,
  // [6..7] gyro_x, [8..9] gyro_y, [10..11] gyro_z,
  // [12..13] acc_x, [14..15] acc_y, [16..17] acc_z
  // It is fragmented to 3 classic CAN frames:
  // ID 0x119 -> bytes [0..7], ID 0x11A -> bytes [8..15], ID 0x11B -> bytes [16..17] + padding.
  void EncodeFromImu(const rm::device::HipnucImu& imu) {
    PackI16BigEndian(UnitToMilliI16(imu.roll()), &tx_payload_[0]);
    PackI16BigEndian(UnitToMilliI16(imu.pitch()), &tx_payload_[2]);
    PackI16BigEndian(UnitToMilliI16(imu.yaw()), &tx_payload_[4]);
    PackI16BigEndian(UnitToMilliI16(imu.gyro_x()), &tx_payload_[6]);
    PackI16BigEndian(UnitToMilliI16(imu.gyro_y()), &tx_payload_[8]);
    PackI16BigEndian(UnitToMilliI16(imu.gyro_z()), &tx_payload_[10]);
    PackI16BigEndian(UnitToMilliI16(imu.acc_x()), &tx_payload_[12]);
    PackI16BigEndian(UnitToMilliI16(imu.acc_y()), &tx_payload_[14]);
    PackI16BigEndian(UnitToMilliI16(imu.acc_z()), &tx_payload_[16]);
  }

  const rm::device::HipnucImu* imu_{nullptr};
  std::array<rm::u8, kFullPayloadSize> tx_payload_{};
  std::array<rm::u8, kCanFrameSize> tx_tail_{};
};
