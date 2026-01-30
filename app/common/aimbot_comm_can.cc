#include "aimbot_comm_can.hpp"
#include "../targets/test_gb/main.hpp"

namespace rm::device {

AimbotCanCommunicator::AimbotCanCommunicator(rm::hal::CanInterface &can) : CanDevice(can, 0x170) {}

u8 AimbotCanCommunicator::aimbot_state() const { return aimbot_state_; }

u8 AimbotCanCommunicator::aimbot_target() const { return aimbot_target_; }

f32 AimbotCanCommunicator::yaw() const { return yaw_; }

f32 AimbotCanCommunicator::pitch() const { return pitch_; }

u8 AimbotCanCommunicator::nuc_start_flag() const { return nuc_start_flag_; }

void AimbotCanCommunicator::RxCallback(const hal::CanFrame *msg) {
  if (msg->rx_std_id == 0x170) {
    ReportStatus(kOk);
    aimbot_state_ = static_cast<u8>(msg->data[0]);
    aimbot_target_ = static_cast<u8>(msg->data[1]);
    yaw_ = modules::F16ToF32(static_cast<modules::f16>((static_cast<uint16_t>(msg->data[2]) << 8) | msg->data[3]));
    pitch_ = modules::F16ToF32(static_cast<modules::f16>((static_cast<uint16_t>(msg->data[4]) << 8) | msg->data[5]));
    nuc_start_flag_ = static_cast<u8>(msg->data[6]);
  }
}

void AimbotCanCommunicator::UpdateControl(f32 w, f32 x, f32 y, f32 z, u8 robot_id, u8 mode, u16 imu_count,
                                          f32 bullet_speed) {
  tx_buf_[0] = modules::F32ToF16(w) >> 8;
  tx_buf_[1] = modules::F32ToF16(w);
  tx_buf_[2] = modules::F32ToF16(x) >> 8;
  tx_buf_[3] = modules::F32ToF16(x);
  tx_buf_[4] = modules::F32ToF16(y) >> 8;
  tx_buf_[5] = modules::F32ToF16(y);
  const u8 z_sign = (z >= 0.f) ? 1 : 0;
  const u8 id_bit = (robot_id > 100) ? 1 : 0;
  const u8 mode_bits = mode & 0x3;                       // 最低 2 位
  const u8 imu_bits = static_cast<u8>(imu_count) & 0xF;  // 最低 4 位

  tx_buf_[6] = static_cast<u8>((z_sign << 7) | (id_bit << 6) | (mode_bits << 4) | imu_bits);
  tx_buf_[7] = modules::FloatToInt(bullet_speed, 0.f, 32.f, 8);

  this->can_->Write(0x150, tx_buf_, 8);
}

}  // namespace rm::device