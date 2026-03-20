#include "GimbalCommunicator.hpp"

namespace rm::device {
GimbalCommunicator::GimbalCommunicator(hal::CanInterface &can) : CanDevice(can, 0x100) {}

void GimbalCommunicator::RxCallback(const hal::CanFrame *msg) {
  if (msg->rx_std_id == 0x120) {
    remote_speed_x_ = static_cast<f32>(msg->data[0]) / 100.0f;
    remote_speed_y_ = static_cast<f32>(msg->data[1]) / 100.0f;
    chassis_mode_ = static_cast<u8>(msg->data[2]);
    UI_show_flag_ = static_cast<u8>(msg->data[3]);
    get_target_flag_ = static_cast<u8>(msg->data[4]);
    suggest_fire_flag_ = static_cast<u8>(msg->data[5]);
    aim_speed_change_ = static_cast<i8>(msg->data[6]);
  }
}

void GimbalCommunicator::SendGimbalCommand(u16 current_heat, u16 heat_limit, u8 power_state, u8 robot_id) {
  tx_buf_[0] = current_heat >> 8;
  tx_buf_[1] = current_heat;
  tx_buf_[2] = heat_limit >> 8;
  tx_buf_[3] = heat_limit;
  tx_buf_[4] = power_state << 4 | (robot_id < 100 ? 0 : 1);
  this->can_->Write(0x120, tx_buf_, 8);
}
}  // namespace rm::device
