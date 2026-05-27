#pragma once

#include <algorithm>
#include <array>

#include <librm.hpp>
bool button_left;
class GimbalToChassisTxBridge final : public rm::device::CanDevice {
 public:
  static constexpr rm::u16 kTxStdIdA = 0x110;
  static constexpr rm::u16 kTxStdIdB = 0x111;
  static constexpr rm::u16 kTxStdIdC = 0x112;
  static constexpr rm::usize kPayloadSize = 8U;

  GimbalToChassisTxBridge(rm::hal::CanInterface& can, const rm::device::HipnucImu* imu, rm::device::VT03* vt03)
      : CanDevice(can, kTxStdIdA, kTxStdIdB, kTxStdIdC), imu_(imu), vt03_(vt03) {}

  void RxCallback(const rm::hal::CanFrame* msg) override {}

  bool QueueSend() {
    EncodeFrameA();
    EncodeFrameC();
    can_->Write(kTxStdIdA, tx_a_.data(), tx_a_.size());
    can_->Write(kTxStdIdC, tx_c_.data(), tx_c_.size());

    if (send_count_ % 50 == 0) {
      EncodeFrameB();
      can_->Write(kTxStdIdB, tx_b_.data(), tx_b_.size());
    }
    send_count_++;

    ReportStatus(kOk);
    return true;
  }

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

  static rm::i16 QuatToI16(rm::f32 q) {
    const rm::f32 scaled = q * 32767.0f;
    const rm::f32 clamped = std::clamp(scaled, -32768.0f, 32767.0f);
    return static_cast<rm::i16>(clamped >= 0.0f ? (clamped + 0.5f) : (clamped - 0.5f));
  }

  static rm::i16 RadToMilliI16(rm::f32 rad) {
    const rm::f32 scaled = rad * 1000.0f;
    const rm::f32 clamped = std::clamp(scaled, -32768.0f, 32767.0f);
    return static_cast<rm::i16>(clamped >= 0.0f ? (clamped + 0.5f) : (clamped - 0.5f));
  }

  // Frame A: [0..1] vt03_online, [2..3] gyro_z, [4..5] gyro_x, [6] mouse_left, [7] mouse_right
  void EncodeFrameA() {
    tx_a_[0] = (vt03_ && vt03_->online_status() == rm::device::Device::kOk) ? 1 : 0;
    tx_a_[1] = 0;
    PackI16(RadToMilliI16(imu_ ? imu_->gyro_z() : 0.f), &tx_a_[2]);
    PackI16(RadToMilliI16(imu_ ? imu_->gyro_x() : 0.f), &tx_a_[4]);
    tx_a_[6] = static_cast<rm::u8>(vt03_ ? (vt03_->data().mouse_button_left ? 1 : 0) : 0);
    tx_a_[7] = static_cast<rm::u8>(vt03_ ? (vt03_->data().mouse_button_right ? 1 : 0) : 0);
  }

  // Frame B: [0..1] mouse_x, [2..3] mouse_y, [4..5] mouse_z, [6..7] keyboard_key
  void EncodeFrameB() {
    if (vt03_) {
      PackI16(vt03_->data().mouse_x, &tx_b_[0]);
      PackI16(vt03_->data().mouse_y, &tx_b_[2]);
      PackI16(vt03_->data().mouse_z, &tx_b_[4]);
      PackU16(vt03_->data().keyboard_key, &tx_b_[6]);
    }
  }

  // Frame C (8 bytes): [0..1] quat_w, [2..3] quat_x, [4..5] quat_y, [6..7] quat_z
  void EncodeFrameC() {
    if (imu_) {
      PackI16(QuatToI16(imu_->quat_w()), &tx_c_[0]);
      PackI16(QuatToI16(imu_->quat_x()), &tx_c_[2]);
      PackI16(QuatToI16(imu_->quat_y()), &tx_c_[4]);
      PackI16(QuatToI16(imu_->quat_z()), &tx_c_[6]);
    }
  }

  const rm::device::HipnucImu* imu_{nullptr};
  rm::device::VT03* vt03_{nullptr};
  std::array<rm::u8, kPayloadSize> tx_a_{};
  std::array<rm::u8, kPayloadSize> tx_b_{};
  std::array<rm::u8, kPayloadSize> tx_c_{};
  uint32_t send_count_{0};
};
