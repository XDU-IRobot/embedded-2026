#include "ChassisCommunicator.hpp"

namespace rm::device {
ChassisCommunicator::ChassisCommunicator(hal::CanInterface &can) : CanDevice(can, 0x100) {}

void ChassisCommunicator::RxCallback(const hal::CanFrame *msg) {
  if (msg->rx_std_id == 0x100) {
    ReportStatus(kOk);
    heat_real_ = static_cast<u16>(msg->data[0]) << 8 | static_cast<u16>(msg->data[1]);
    heat_limit_ = static_cast<u16>(msg->data[2]) << 8 | static_cast<u16>(msg->data[3]);
    ammo_speed_ = modules::IntToFloat(msg->data[4], 0.f, 32.f, 8);
    robot_id_ = msg->data[5] & 0x01;
    gimbal_power_state_ = msg->data[5] >> 4 & 0x01;
    chassis_power_state_ = msg->data[5] >> 5 & 0x01;
    ammo_power_state_ = msg->data[5] >> 6 & 0x01;
  }
}

void ChassisCommunicator::SendChassisCommand(f32 chassis_move_x, f32 chassis_move_y, u8 chassis_state,
                                             u8 ui_refresh_flag, u8 get_target_flag, u8 suggest_fire_flag,
                                             i8 aim_speed_change) {
  tx_buf_[0] = static_cast<i8>(chassis_move_x);
  tx_buf_[1] = static_cast<i8>(chassis_move_y);
  tx_buf_[2] = chassis_state;
  tx_buf_[3] = ui_refresh_flag;
  tx_buf_[4] = get_target_flag;
  tx_buf_[5] = suggest_fire_flag;
  tx_buf_[6] = aim_speed_change;
  this->can_->Write(0x120, tx_buf_, 8);
}
}  // namespace rm::device
