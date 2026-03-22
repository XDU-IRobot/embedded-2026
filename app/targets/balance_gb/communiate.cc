#include "librm.hpp"

#include "communiate.hpp"
#include "global.hpp"
using namespace rm;
using namespace rm::device;
uint8_t id;
VT03 tcremote;

/*
@brief:底盘通信类的实现
*/
void ChassisCommunicator::RxCallback(const hal::CanFrame *msg) {
  if (msg->rx_std_id == 0x119) {
    chassis_data_rx.HeatLimit = static_cast<u16>(msg->data[0] << 8) | static_cast<u16>(msg->data[1]);
    chassis_data_rx.HeatCurrent = static_cast<u16>(msg->data[2] << 8) | static_cast<u16>(msg->data[3]);
    chassis_data_rx.CoolingSpeed = static_cast<u16>(msg->data[4] << 8) | static_cast<u16>(msg->data[5]);
    chassis_data_rx.id = (0x01 & static_cast<u16>(msg->data[6]));
    chassis_data_rx.GimbalOutState = (0x10 & static_cast<u16>(msg->data[6])) >> 4;
    chassis_data_rx.ChassisOutState = (0x20 & static_cast<u16>(msg->data[6])) >> 5;
    chassis_data_rx.AmmoOutState = (0x40 & static_cast<u16>(msg->data[6])) >> 6;
  }
  id = chassis_data_rx.id;
}
void ChassisCommunicator::SendChassisCommand() {
  tx_buf_[0] = global.chassis_tx->gimbal_data_tx.ChassisMoveYRequest >> 8;  // 遥控器y轴数值
  tx_buf_[1] = global.chassis_tx->gimbal_data_tx.ChassisMoveYRequest;
  tx_buf_[2] = global.chassis_tx->gimbal_data_tx.ChassisStateRequest;  // 底盘状态
  tx_buf_[3] = global.chassis_tx->gimbal_data_tx.L0Change;             // 腿长要求
  tx_buf_[4] = global.chassis_tx->gimbal_data_tx.ui_flag;              // ui指令

  this->can_->Write(0x111, tx_buf_, 5);
}

/*
@brief:图传原始数据接收类的实现
*/
namespace rm::device {
TcReceiver::TcReceiver(rm::hal::SerialInterface &serial) : serial_(&serial) {
  static rm::hal::SerialRxCallbackFunction rx_callback =
      std::bind(&TcReceiver::RxCallback, this, std::placeholders::_1, std::placeholders::_2);
  this->serial_->AttachRxCallback(rx_callback);
}

void TcReceiver::Begin() {
  this->serial_->Begin();
}

void TcReceiver::RxCallback(const std::vector<u8> &data, u16 rx_len) {
  for (u16 i = 0; i < rx_len; i++) {
    tcremote << data.at(i);
  }
}
}     // namespace rm::device
extern "C" {}