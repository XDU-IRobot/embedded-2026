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

void GimbalCommunicator::SendGimbalCommand(i8 chassis_move_x, i8 chassis_move_y, u8 chassis_state, u8 ui_refresh_flag,
                                             u8 get_target_flag, u8 suggest_fire_flag, i8 aim_speed_change) {
  tx_buf_[0] = current_heat >> 8;
  tx_buf_[1] = current_heat;
  tx_buf_[2] = heat_limit >> 8;
  tx_buf_[3] = heat_limit;
  tx_buf_[4] = power_state << 4 | ((tx_gimbal_data.robot_id < 100) ? 0 : 1);
  this->can_->Write(0x120, tx_buf_, 8);
}
}  // namespace rm::device
