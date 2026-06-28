#pragma once

#include <algorithm>
#include <array>

#include <librm.hpp>
bool button_left;

struct EmyRobotHP {
  rm::u16 hero_1_HP;
  rm::u16 engineer_2_HP;
  rm::u16 standard_3_HP;
  rm::u16 standard_4_HP;
  rm::u16 sentry_7_HP;
};

class GimbalToChassisTxBridge final : public rm::device::CanDevice {
 public:
  static constexpr rm::u16 kTxStdIdA = 0x110;
  static constexpr rm::u16 kTxStdIdB = 0x111;
  static constexpr rm::u16 kTxStdIdC = 0x112;
  static constexpr rm::u16 kTxStdIdD = 0x113;
  static constexpr rm::u16 kTxStdIdE = 0x114;
  static constexpr rm::usize kPayloadSize = 8U;

  GimbalToChassisTxBridge(rm::hal::CanInterface& can, const rm::device::HipnucImu* imu, rm::device::VT03* vt03)
      : CanDevice(can, kTxStdIdA, kTxStdIdB, kTxStdIdC, kTxStdIdD, kTxStdIdE), imu_(imu), vt03_(vt03) {}

  void UpdateRobotHP(const EmyRobotHP& hp) { robot_hp_ = hp; }
  void SetChassisFricRpm(rm::i16 left, rm::i16 right) {
    fric_left_rpm_ = left;
    fric_right_rpm_ = right;
  }

  void RxCallback(const rm::hal::CanFrame* msg) override {}

  bool QueueSend() {
    // Frame A/C: 500Hz (每2周期发一次，主循环1kHz)
    if (send_count_ % 2 == 0) {
      EncodeFrameA();
      EncodeFrameC();
      can_->Write(kTxStdIdA, tx_a_.data(), tx_a_.size());
      can_->Write(kTxStdIdC, tx_c_.data(), tx_c_.size());
    }

    // Frame E: 500Hz (每2周期发一次，主循环1kHz)
    if (send_count_ % 2 == 0) {
      EncodeFrameE();
      can_->Write(kTxStdIdE, tx_e_.data(), tx_e_.size());
    }

    if (send_count_ % 50 == 0) {
      EncodeFrameB();
      EncodeFrameD();
      can_->Write(kTxStdIdB, tx_b_.data(), tx_b_.size());
      can_->Write(kTxStdIdD, tx_d_.data(), tx_d_.size());
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

  // Frame D (8 bytes): [0..1] hero_1_HP, [2..3] engineer_2_HP, [4..5] standard_3_HP, [6..7] standard_4_HP
  void EncodeFrameD() {
    PackU16(robot_hp_.hero_1_HP, &tx_d_[0]);
    PackU16(robot_hp_.engineer_2_HP, &tx_d_[2]);
    PackU16(robot_hp_.standard_3_HP, &tx_d_[4]);
    PackU16(robot_hp_.standard_4_HP, &tx_d_[6]);
  }

  // Frame E (8 bytes): [0..1] sentry_7_HP, [2..3] fric_left_rpm, [4..5] fric_right_rpm
  void EncodeFrameE() {
    PackU16(robot_hp_.sentry_7_HP, &tx_e_[0]);
    PackI16(fric_left_rpm_, &tx_e_[2]);
    PackI16(fric_right_rpm_, &tx_e_[4]);
  }

  const rm::device::HipnucImu* imu_{nullptr};
  rm::device::VT03* vt03_{nullptr};
  EmyRobotHP robot_hp_{};
  rm::i16 fric_left_rpm_{0};
  rm::i16 fric_right_rpm_{0};
  std::array<rm::u8, kPayloadSize> tx_a_{};
  std::array<rm::u8, kPayloadSize> tx_b_{};
  std::array<rm::u8, kPayloadSize> tx_c_{};
  std::array<rm::u8, kPayloadSize> tx_d_{};
  std::array<rm::u8, kPayloadSize> tx_e_{};
  uint32_t send_count_{0};
};

class ChassisToGimbalRxBridge final : public rm::device::CanDevice {
 public:
  static constexpr rm::u16 kRxStdId = 0x120;
  static constexpr rm::usize kPayloadSize = 8U;

  explicit ChassisToGimbalRxBridge(rm::hal::CanInterface& can) : CanDevice(can, kRxStdId) {}

  void RxCallback(const rm::hal::CanFrame* msg) override {
    if (msg == nullptr) return;
    if (msg->rx_std_id == kRxStdId && msg->dlc >= kPayloadSize) {
      combat_mode_ = (msg->data[0] != 0);
      frame_count_++;
      ReportStatus(kOk);
    }
  }

  [[nodiscard]] bool combat_mode() const { return combat_mode_; }
  [[nodiscard]] rm::u32 frame_count() const { return frame_count_; }

 private:
  bool combat_mode_{false};
  rm::u32 frame_count_{0};
};
