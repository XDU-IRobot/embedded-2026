#include "Communicate.h"
#include "librm.hpp"

using namespace rm::modules;
Chassis_Command mychassis;
Communicate::Communicate(hal::CanInterface &can) : CanDevice(can, 0x60) {}

void Communicate::RxCallback(const hal::CanFrame *msg) {
  // referee_.powerstate = msg->data[0] & 0x0F;
  // referee_.robot_id = ((msg->data[0] >> 4) > 7) ? 101 : 1;
  // referee_.speed = IntToFloat(msg->data[1], 12, 18, 8);
  // referee_.realheat = (msg->data[2] << 8) | msg->data[3];
  // referee_.maxheat = (msg->data[4] << 8) | msg->data[5];
  // referee_.cooling = (msg->data[6] << 8) | msg->data[7];

  // 发送端规则 → 接收端完全对齐（1:1 对应，无误差）
  referee_.powerstate = msg->data[0] & 0x07;                      // 低3位：功率输出使能（和发送端一致）
  referee_.robot_id = (msg->data[0] >> 4) & 0x0F;                // 高4位：机器人ID（发送端标准规则）
  referee_.speed = IntToFloat(msg->data[1], 12, 18, 8);          // 射速（不变，已正确）
  referee_.realheat = (msg->data[2] << 8) | msg->data[3];        // 当前热量（不变，已正确）
  referee_.maxheat = (msg->data[4] << 8) | msg->data[5];         // 最大热量（不变，已正确）
  referee_.cooling = (msg->data[6] << 8) | msg->data[7];         // 冷却速率（不变，已正确）
}

void Communicate::SendChassisCommand() {
  tx_buf_[0] =
      static_cast<u8>(mychassis._command.chassis.state) | (static_cast<u8>(mychassis._command.chassis.leg_length) << 4);

  tx_buf_[1] = mychassis._command.chassis.move_x;
  tx_buf_[2] = mychassis._command.chassis.move_y;

  tx_buf_[3] = mychassis._command.ui.ui1;
  tx_buf_[4] = mychassis._command.ui.ui2;
  tx_buf_[5] = mychassis._command.ui.ui3;
  tx_buf_[6] = mychassis._command.ui.ui4;
  this->can_->Write(0x59, tx_buf_, 7);
}

RefereeData refereedata;

RefereeData::RefereeData() {}

RefereeData::~RefereeData() {}

bool RefereeData::getPowerState(enum POWER id) const { return _powerstate[static_cast<u8>(id)]; }

u16 RefereeData::getRealHeat() const { return _realheat; }

u8 RefereeData::getrobotid() const { return _robotid; }

u16 RefereeData::getmaxheat() const { return _maxheat; }

u16 RefereeData::getcooling() const { return _cooling; }

f32 RefereeData::getspeed() const { return _speed; }

void RefereeData::act(Communicate &communicate) {
  for (int i = 0; i < 3; i++) {
    _prepowerstate[i] = _powerstate[i];
  }
  _powerstate[0] = communicate.referee_.powerstate & 0x01;
  _powerstate[1] = communicate.referee_.powerstate & 0x02;
  _powerstate[2] = communicate.referee_.powerstate & 0x04;
  _robotid = communicate.referee_.robot_id;
  _cooling = communicate.referee_.cooling;
  _maxheat = communicate.referee_.maxheat;
  _realheat = communicate.referee_.realheat;
  _speed = communicate.referee_.speed;
}