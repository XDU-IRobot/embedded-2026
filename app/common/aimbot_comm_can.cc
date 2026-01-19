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

void AimbotCanCommunicator::UpdateQuaternion(f32 w, f32 x, f32 y, f32 z) {
  tx_buf_[0] = modules::F32ToF16(w) >> 8;
  tx_buf_[1] = modules::F32ToF16(w);
  tx_buf_[2] = modules::F32ToF16(x) >> 8;
  tx_buf_[3] = modules::F32ToF16(x);
  tx_buf_[4] = modules::F32ToF16(y) >> 8;
  tx_buf_[5] = modules::F32ToF16(y);
  tx_buf_[6] = modules::F32ToF16(z) >> 8;
  tx_buf_[7] = modules::F32ToF16(z);
  this->can_->Write(0x150, tx_buf_, 8);
}

void AimbotCanCommunicator::UpdateControlFlag(u8 robot_id, u8 mode, u16 imu_count, u32 imu_time) {
  tx_buf_[0] = robot_id;
  tx_buf_[1] = mode;
  tx_buf_[2] = imu_count >> 8;
  tx_buf_[3] = imu_count;
  tx_buf_[4] = imu_time >> 24;
  tx_buf_[5] = imu_time >> 16;
  tx_buf_[6] = imu_time >> 8;
  tx_buf_[7] = imu_time;
  this->can_->Write(0x160, tx_buf_, 8);
}

}  // namespace rm::device