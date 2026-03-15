#include "librm.hpp"

#include "communiate.hpp"
#include "global.hpp"
using namespace rm;
using namespace rm::device;
uint8_t id;
i8 speed;

/*
@brief:底盘通信类的实现
*/
void ChassisCommunicator::RxCallback(const hal::CanFrame *msg) {
  speed = chassis_data_rx.BulletSpeed;
  if (msg->rx_std_id == 0x119) {
    chassis_data_rx.GimbalInitFlag = static_cast<u8>(msg->data[0]);
    chassis_data_rx.BulletSpeed = static_cast<i8>(msg->data[1]);
  }
}
void ChassisCommunicator::SendChassisCommand() {
  tx_buf_[0] = global.chassis_communicator->gimbal_data_tx.ChassisMoveYRequest >> 8;  // 遥控器y轴数值
  tx_buf_[1] = global.chassis_communicator->gimbal_data_tx.ChassisMoveYRequest;
  tx_buf_[2] = global.chassis_communicator->gimbal_data_tx.ChassisStateRequest;  // 底盘状态
  tx_buf_[3] = global.chassis_communicator->gimbal_data_tx.L0Change;             // 腿长要求

  this->can_->Write(0x111, tx_buf_, 4);
}

/*
@brief:图传原始数据接收类的实现
*/

TcReceiver::TcReceiver(hal::SerialInterface &serial) : serial_{&serial} {}

void TcReceiver::Begin() { this->serial_->Begin(); }

void TcReceiver::RxCallback(const std::vector<u8> &data, u16 rx_len) {
  for (u16 i = 0; i < rx_len; i++) {
    global.bc->tcremote << data.at(i);
  }
}

extern "C" {}