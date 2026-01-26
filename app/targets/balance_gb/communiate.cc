#include "librm.hpp"

#include "communiate.hpp"
#include "global.hpp"
using namespace rm;
using namespace rm::device;
uint8_t id;

//ChassisController *chassis_controller;

ChassisController::ChassisController(hal::CanInterface &can)
    : CanDevice(can, 0x100) {}

void ChassisController::RxCallback(const hal::CanFrame *msg) {
}
void ChassisController::SendChassisCommand() {
  tx_buf_[0] = global.chassis_controller->request_state.ChassisMoveYRequest >> 8;   //遥控器y轴数值
  tx_buf_[1] = global.chassis_controller->request_state.ChassisMoveYRequest;
  tx_buf_[2] = global.chassis_controller->request_state.ChassisStateRequest;       //底盘状态
  tx_buf_[3] = global.chassis_controller->request_state.L0Change;                  //腿长要求

  this->can_->Write(0x111, tx_buf_, 4);
}


extern "C" {

}