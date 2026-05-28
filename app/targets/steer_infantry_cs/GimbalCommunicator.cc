#include "GimbalCommunicator.hpp"

namespace rm::device {
GimbalCommunicator::GimbalCommunicator(hal::CanInterface &can) : CanDevice(can, 0x110, 0x120) {}

void GimbalCommunicator::RxCallback(const hal::CanFrame *msg) {
  if (msg->rx_std_id == 0x120) {
    remote_speed_x_ = static_cast<f32>(static_cast<i8>(msg->data[0])) / 100.0f;
    remote_speed_y_ = static_cast<f32>(static_cast<i8>(msg->data[1])) / 100.0f;
    chassis_mode_ = static_cast<u8>(msg->data[2]);
    UI_show_flag_ = static_cast<u8>(msg->data[3]);
    get_target_flag_ = static_cast<u8>(msg->data[4]) >> 0 & 0x01;
    suggest_fire_flag_ = static_cast<u8>(msg->data[4]) >> 1 & 0x01;
    aim_speed_change_ = static_cast<i8>(msg->data[5]);
    robot_hp_[0] = static_cast<u16>(msg->data[6]) << 8 | msg->data[7];
  }
  if (msg->rx_std_id == 0x110) {
    robot_hp_[1] = static_cast<u16>(msg->data[0]) << 8 | msg->data[1];
    robot_hp_[2] = static_cast<u16>(msg->data[2]) << 8 | msg->data[3];
    robot_hp_[3] = static_cast<u16>(msg->data[4]) << 8 | msg->data[5];
    robot_hp_[4] = static_cast<u16>(msg->data[6]) << 8 | msg->data[7];
  }
}

void GimbalCommunicator::SendGimbalCommand(u16 current_heat, u16 heat_limit, float ammo_speed, u8 power_state,
                                           u8 robot_id) {
  tx_buf_[0] = current_heat >> 8;
  tx_buf_[1] = current_heat;
  tx_buf_[2] = heat_limit >> 8;
  tx_buf_[3] = heat_limit;
  tx_buf_[4] = rm::modules::FloatToInt(ammo_speed, 0.0f, 32.0f, 8);
  tx_buf_[5] = power_state << 4 | (robot_id < 100 ? 0 : 1);
  this->can_->Write(0x100, tx_buf_, 8);
}
}  // namespace rm::device
