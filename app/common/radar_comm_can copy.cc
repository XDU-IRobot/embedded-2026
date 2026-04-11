#include "radar_comm_can copy.hpp"

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
    // yaw_=0;
    // pitch_ = 0;
    // aimbot_state_ = 0;
  }
}

// void AimbotCanCommunicator::UpdateControl(f32 yaw, f32 pitch, f32 roll, u8 robot_id, u8 mode, u16 imu_count,
//                                           f32 bullet_speed) {
//   tx_buf_[0] = modules::F32ToF16(yaw) >> 8;
//   tx_buf_[1] = modules::F32ToF16(yaw);
//   tx_buf_[2] = modules::F32ToF16(pitch) >> 8;
//   tx_buf_[3] = modules::F32ToF16(pitch);
//   tx_buf_[4] = modules::F32ToF16(roll) >> 8;
//   tx_buf_[5] = modules::F32ToF16(roll);
//   const u8 id_bit = robot_id > 100 ? 1 : 0;
//   const u8 mode_bits = mode & 0x3;                       // 最低 2 位
//   const u8 imu_bits = static_cast<u8>(imu_count) & 0xF;  // 最低 4 位

//   tx_buf_[6] = static_cast<u8>(id_bit << 6 | mode_bits << 4 | imu_bits);
//   tx_buf_[7] = modules::FloatToInt(bullet_speed, 0.f, 32.f, 8);

//   this->can_->Write(0x150, tx_buf_, 8);
// }

// void AimbotCanCommunicator::UpdateQuaternion(f32 w, f32 x, f32 y, f32 z) {
//   tx_buf_[0] = static_cast<i16>(w * 10000.0f) >> 8;
//   tx_buf_[1] = static_cast<i16>(w * 10000.0f);
//   tx_buf_[2] = static_cast<i16>(x * 10000.0f) >> 8;
//   tx_buf_[3] = static_cast<i16>(x * 10000.0f);
//   tx_buf_[4] = static_cast<i16>(y * 10000.0f) >> 8;
//   tx_buf_[5] = static_cast<i16>(y * 10000.0f);
//   tx_buf_[6] = static_cast<i16>(z * 10000.0f) >> 8;
//   tx_buf_[7] = static_cast<i16>(z * 10000.0f);
//   this->can_->Write(0x150, tx_buf_, 8);
// }

// void AimbotCanCommunicator::UpdateControlFlag(u8 robot_id, u8 mode, u16 imu_count, u32 imu_time) {
//   tx_buf_[0] = robot_id;
//   tx_buf_[1] = mode;
//   tx_buf_[2] = imu_count >> 8;
//   tx_buf_[3] = imu_count;
//   tx_buf_[4] = imu_time >> 24;
//   tx_buf_[5] = imu_time >> 16;
//   tx_buf_[6] = imu_time >> 8;
//   tx_buf_[7] = imu_time;
//   this->can_->Write(0x160, tx_buf_, 8);
// }

}  // namespace rm::device