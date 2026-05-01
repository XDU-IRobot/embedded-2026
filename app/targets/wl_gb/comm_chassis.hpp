#pragma once

#include <algorithm>
#include <array>

#include <librm.hpp>

class GimbalToChassisTxBridge final : public rm::device::CanDevice {
 public:
  static constexpr rm::u16 kTxStdIdA = 0x110;
  static constexpr rm::u16 kTxStdIdB = 0x111;
  static constexpr rm::usize kPayloadSize = 8U;

  GimbalToChassisTxBridge(rm::hal::CanInterface& can, const rm::device::HipnucImu* imu, const rm::device::VT03* vt03)
      : CanDevice(can, kTxStdIdA, kTxStdIdB), imu_(imu), vt03_(vt03) {}

  bool QueueSend() {
    EncodeFrameA();
    EncodeFrameB();
    can_->Write(kTxStdIdA, tx_a_.data(), tx_a_.size());
    can_->Write(kTxStdIdB, tx_b_.data(), tx_b_.size());

    ReportStatus(kOk);
    return true;
  }

  void RxCallback(const rm::hal::CanFrame* msg) override { (void)msg; }

 private:
  static void PackI16(rm::i16 value, rm::u8* out) {
    const auto raw = static_cast<rm::u16>(value);
    out[0] = static_cast<rm::u8>(raw >> 8);
    out[1] = static_cast<rm::u8>(raw);
  }

  static void PackU16(rm::u16 value, rm::u8* out) {
    out[0] = static_cast<rm::u8>(value >> 8);
    out[1] = static_cast<rm::u8>(value);
  }

  static rm::i16 RadToMilliI16(rm::f32 rad) {
    const rm::f32 scaled = rad * 1000.0f;
    const rm::f32 clamped = std::clamp(scaled, -32768.0f, 32767.0f);
    return static_cast<rm::i16>(clamped >= 0.0f ? (clamped + 0.5f) : (clamped - 0.5f));
  }

  // Frame A (8 bytes): [0..1] pitch, [2..3] yaw, [4..5] mouse_x, [6..7] mouse_y
  void EncodeFrameA() {
    PackI16(RadToMilliI16(imu_ ? imu_->pitch() : 0.f), &tx_a_[0]);
    PackI16(RadToMilliI16(imu_ ? imu_->yaw() : 0.f), &tx_a_[2]);
    if (vt03_) {
      PackI16(vt03_->data().mouse_x, &tx_a_[4]);
      PackI16(vt03_->data().mouse_y, &tx_a_[6]);
    }
  }

  // Frame B (8 bytes): [0..1] mouse_z, [2] left, [3] right, [4..5] keyboard_key, [6..7] reserved
  void EncodeFrameB() {
    if (vt03_) {
      PackI16(vt03_->data().mouse_z, &tx_b_[0]);
      tx_b_[2] = static_cast<rm::u8>(vt03_->data().mouse_button_left ? 1 : 0);
      tx_b_[3] = static_cast<rm::u8>(vt03_->data().mouse_button_right ? 1 : 0);
      PackU16(vt03_->data().keyboard_key, &tx_b_[4]);
    }
    tx_b_[6] = 0;
    tx_b_[7] = 0;
  }

  const rm::device::HipnucImu* imu_{nullptr};
  const rm::device::VT03* vt03_{nullptr};
  std::array<rm::u8, kPayloadSize> tx_a_{};
  std::array<rm::u8, kPayloadSize> tx_b_{};
};
