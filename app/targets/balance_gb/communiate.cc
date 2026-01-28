#include "librm.hpp"

#include "communiate.hpp"
#include "global.hpp"
using namespace rm;
using namespace rm::device;
uint8_t id;

ChassisCommunicator::ChassisCommunicator(hal::CanInterface &can) : CanDevice(can, 0x119) {}

void ChassisCommunicator::RxCallback(const hal::CanFrame *msg) {
  if (msg->rx_std_id == 0x119) {
    chassis_data_rx.GimbalInitFlag = static_cast<u8>(msg->data[0]);
  }
}
void ChassisCommunicator::SendChassisCommand() {
  tx_buf_[0] = global.chassis_communicator->request_state.ChassisMoveYRequest >> 8;  // 遥控器y轴数值
  tx_buf_[1] = global.chassis_communicator->request_state.ChassisMoveYRequest;
  tx_buf_[2] = global.chassis_communicator->request_state.ChassisStateRequest;  // 底盘状态
  tx_buf_[3] = global.chassis_communicator->request_state.L0Change;             // 腿长要求

  this->can_->Write(0x111, tx_buf_, 4);
}

extern "C" {

}