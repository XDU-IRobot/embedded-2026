#include "radar_comm_can.hpp"

namespace rm::device {

RadarCanCommunicator::RadarCanCommunicator(rm::hal::CanInterface &can) : CanDevice(can, 0x301) {}

void RadarCanCommunicator::RxCallback(const hal::CanFrame *msg) {
  if (msg->rx_std_id == 0x301) {
    ReportStatus(kOk);
    distance_mm_ = static_cast<u16>((static_cast<uint16_t>(msg->data[1]) << 8) | msg->data[0]);
    planner_distance_mm_ = static_cast<u16>((static_cast<uint16_t>(msg->data[3]) << 8) | msg->data[2]);
    yaw_mard_ = static_cast<i16>((static_cast<uint16_t>(msg->data[5]) << 8) | msg->data[4]);
    status_ = static_cast<u8>(msg->data[6]);
    vaild_ = (status_ & 0x01) != 0;  // 假设最低位表示数据有效性
    fresh_ = (status_ & 0x02) != 0;  // 假设第二位表示数据新鲜度
    counter_ = static_cast<u8>(msg->data[7]);
  } else {
    vaild_ = false;
    fresh_ = false;
  }
}

}  // namespace rm::device