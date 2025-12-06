#include "Can.hpp"
#include "main.hpp"

namespace rm::device {

CanCommunicator::CanCommunicator(rm::hal::CanInterface &can) : CanDevice(can, 0x170) {}

void CanCommunicator::RxCallback(const hal::CanFrame *msg) {
  if (msg->rx_std_id == 0x170) {
    AimbotState = static_cast<u8>(msg->data[0]);
    AimbotTarget = static_cast<u8>(msg->data[1]);
    Yaw = static_cast<f32>(static_cast<u16>(msg->data[2]) << 8 | static_cast<u16>(msg->data[3])) / 10000.0f;
    Pitch = static_cast<f32>(static_cast<u16>(msg->data[4]) << 8 | static_cast<u16>(msg->data[5])) / 10000.0f;
    NucStartFlag = static_cast<u8>(msg->data[6]);
  }
}

void CanCommunicator::SendMessage() {
  tx_buf1_[0] = static_cast<i16>(globals->ahrs.quaternion().w * 10000.0f) >> 8;
  tx_buf1_[1] = static_cast<i16>(globals->ahrs.quaternion().w * 10000.0f);
  tx_buf1_[2] = static_cast<i16>(globals->ahrs.quaternion().x * 10000.0f) >> 8;
  tx_buf1_[3] = static_cast<i16>(globals->ahrs.quaternion().x * 10000.0f);
  tx_buf1_[4] = static_cast<i16>(globals->ahrs.quaternion().y * 10000.0f) >> 8;
  tx_buf1_[5] = static_cast<i16>(globals->ahrs.quaternion().y * 10000.0f);
  tx_buf1_[6] = static_cast<i16>(globals->ahrs.quaternion().z * 10000.0f) >> 8;
  tx_buf1_[7] = static_cast<i16>(globals->ahrs.quaternion().z * 10000.0f);
  this->can_->Write(0x150, tx_buf1_, 8);
  tx_buf2_[0] = 0;
  tx_buf2_[1] = mode;
  tx_buf2_[2] = imu_count >> 8;
  tx_buf2_[3] = imu_count;
  this->can_->Write(0x160, tx_buf2_, 4);
}

}  // namespace rm::device