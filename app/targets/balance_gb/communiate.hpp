#ifndef COMMUNIATE_H
#define COMMUNIATE_H

#include <librm.hpp>

using namespace rm;
using namespace rm::device;

class ChassisController final : public CanDevice {
 public:
  // ChassisController(rm::hal::CanInterface &can, uint32_t rx_std_id) : CanDevice{can, rx_std_id} {}

  explicit ChassisController(hal::CanInterface &can);
  ChassisController() = delete;
  ~ChassisController() override = default;

  void RxCallback(const hal::CanFrame *msg) override;
  void SendChassisCommand();

  struct GimbalRequestState {
    i16 ChassisMoveYRequest;  // y轴运动控制
    u8 ChassisStateRequest;   // 状态
    u8 L0Change; /* 腿长变换  0x00 低腿长  0x01  正常腿长   0x02  跳跃时先下蹲  0x03  伸腿  0x04  收腿  0x05  跳跃缓冲
                    0x06  测试高腿长*/
  };

  GimbalRequestState request_state;

  bool jump_flag = false;
  i16 jump_count = 0;

 private:
  u8 tx_buf_[8]{0};
};

#endif