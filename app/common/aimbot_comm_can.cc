#include "aimbot_comm_can.hpp"
#include "../targets/test_gb/main.hpp"

namespace rm::device {

CanCommunicator::CanCommunicator(rm::hal::CanInterface &can) : CanDevice(can, 0x170) {}

u8 CanCommunicator::aimbot_state() const { return aimbot_state_; }

u8 CanCommunicator::aimbot_target() const { return aimbot_target_; }

f32 CanCommunicator::yaw() const { return yaw_; }

f32 CanCommunicator::pitch() const { return pitch_; }

u8 CanCommunicator::nuc_start_flag() const { return nuc_start_flag_; }

void CanCommunicator::RxCallback(const hal::CanFrame *msg) {
  if (msg->rx_std_id == 0x170) {
    aimbot_state_ = static_cast<u8>(msg->data[0]);
    aimbot_target_ = static_cast<u8>(msg->data[1]);
    yaw_ = static_cast<f32>(static_cast<u16>(msg->data[2]) << 8 | static_cast<u16>(msg->data[3])) / 10000.0f;
    pitch_ = static_cast<f32>(static_cast<u16>(msg->data[4]) << 8 | static_cast<u16>(msg->data[5])) / 10000.0f;
    nuc_start_flag_ = static_cast<u8>(msg->data[6]);
  }
}

void CanCommunicator::UpdateQuaternion(f32 w, f32 x, f32 y, f32 z) {
  tx_buf_[0] = static_cast<i16>(w * 10000.0f) >> 8;
  tx_buf_[1] = static_cast<i16>(w * 10000.0f);
  tx_buf_[2] = static_cast<i16>(x * 10000.0f) >> 8;
  tx_buf_[3] = static_cast<i16>(x * 10000.0f);
  tx_buf_[4] = static_cast<i16>(y * 10000.0f) >> 8;
  tx_buf_[5] = static_cast<i16>(y * 10000.0f);
  tx_buf_[6] = static_cast<i16>(z * 10000.0f) >> 8;
  tx_buf_[7] = static_cast<i16>(z * 10000.0f);
  this->can_->Write(0x150, tx_buf_, 8);
}

void CanCommunicator::UpdateControlFlag(u8 robot_id, u8 mode, u16 imu_count) {
  tx_buf_[0] = robot_id;
  tx_buf_[1] = mode;
  tx_buf_[2] = imu_count >> 8;
  tx_buf_[3] = imu_count;
  this->can_->Write(0x160, tx_buf_, 4);
}

}  // namespace rm::device