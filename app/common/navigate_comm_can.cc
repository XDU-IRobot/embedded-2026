#include "navigate_comm_can.hpp"

namespace rm::device {

NavigateCanCommunicator::NavigateCanCommunicator(rm::hal::CanInterface &can) : CanDevice(can, 0x180, 0x190) {}

f32 NavigateCanCommunicator::chassis_target_x() const { return chassis_target_x_; }

f32 NavigateCanCommunicator::chassis_target_y() const { return chassis_target_y_; }

f32 NavigateCanCommunicator::chassis_target_w() const { return chassis_target_w_; }

f32 NavigateCanCommunicator::target_yaw_speed() const { return target_yaw_speed_; }

u8 NavigateCanCommunicator::scan_mode() const { return scan_mode_; }

u8 NavigateCanCommunicator::perception_flag() const { return perception_flag_; }

void NavigateCanCommunicator::RxCallback(const hal::CanFrame *msg) {
  if (msg->rx_std_id == 0x180) {
    ReportStatus(kOk);
    chassis_target_x_ = static_cast<f32>(static_cast<i16>(msg->data[0] << 8 | msg->data[1])) / 1000.0f;
    chassis_target_y_ = static_cast<f32>(static_cast<i16>(msg->data[2] << 8 | msg->data[3])) / 1000.0f;
    chassis_target_w_ = static_cast<f32>(static_cast<i16>(msg->data[4] << 8 | msg->data[5])) / 1000.0f;
    target_yaw_speed_ = static_cast<f32>(static_cast<i16>(msg->data[6] << 8 | msg->data[7])) / 1000.0f;
  }
  if (msg->rx_std_id == 0x190) {
    ReportStatus(kOk);
    scan_mode_ = msg->data[0];
    perception_flag_ = msg->data[1];
  }
}

void NavigateCanCommunicator::Update() { this->can_->Write(0x111, tx_buf_, 8); }

}  // namespace rm::device