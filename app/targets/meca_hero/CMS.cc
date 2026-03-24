#include "CMS.H"

CMS::CMS(hal::CanInterface &can) : CanDevice(can, CAP_FEDBACK_ID) {}

void CMS::RxCallback(const hal::CanFrame *msg) {
  offline_counter = 0;
  int16_t i = ((uint16_t)msg->data[2] << 8 | msg->data[3]);
  int16_t v = ((uint16_t)msg->data[0] << 8 | msg->data[1]);
  cms_i = int16_to_float(i, 32000, -32000, 500, 0);
  cms_v = int16_to_float(v, 32000, -32000, 30, 0);
  cms_status = ((uint16_t)msg->data[4] << 8 | msg->data[5]);
}

void CMS::SendCapBuffer(u16 power) {
  tx_buf_[0] = power >> 8;
  tx_buf_[1] = power;
  this->can_->Write(CAP_BUFFER_ID, tx_buf_, 2);
}

void CMS::SendCapPower(u16 powerlimit) {
  tx_buf_[0] = powerlimit >> 8;
  tx_buf_[1] = powerlimit;
  tx_buf_[2] = OUTPUTPOWER >> 8;
  tx_buf_[3] = OUTPUTPOWER & 0xff;
  tx_buf_[4] = INPUTPOWER >> 8;
  tx_buf_[5] = INPUTPOWER & 0xff;
  tx_buf_[6] = 0x00;
  tx_buf_[7] = 0x01;

  this->can_->Write(CAP_POWER_ID, tx_buf_, 8);
}