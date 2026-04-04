#ifndef COMMUNIATE_H
#define COMMUNIATE_H

#include <librm.hpp>

using namespace rm;
//using namespace rm::device;

/*
@brief:与底盘通信的can设备
*/
class ChassisCommunicator final : public device::CanDevice {
 public:
  ChassisCommunicator(rm::hal::CanInterface &can, uint32_t rx_std_id) : CanDevice{can, rx_std_id} {}

  void RxCallback(const hal::CanFrame *msg) override;
  void SendChassisCommand();

  struct TxGimbalData {
    i16 ChassisMoveYRequest;  // y轴运动控制
    u8 ChassisStateRequest;   // 状态
    u8 L0Change; /* 腿长变换  0x00 低腿长  0x01  正常腿长   0x02  跳跃时先下蹲  0x03  伸腿  0x04  收腿  0x05  跳跃缓冲
                    0x06  测试高腿长*/
    u8 ui_flag;   // ui指令
  };

  TxGimbalData gimbal_data_tx;

  struct RxChassisData {
    u8 GimbalInitFlag;  // 倒地自启云台控制

    u16 HeatLimit;      //热量限制
    u8 GimbalOutState;  //云台输出状态
    u8 ChassisOutState;  //底盘输出状态
    u8 AmmoOutState;    //发射机构输出状态
    u16 HeatCurrent;    //现在瞬时热量
    u16 CoolingSpeed;   //冷却速度
    u8 Bulletspeed;      //弹速
    u8 id;              //机器人id
  };

  RxChassisData chassis_data_rx;

  bool jump_flag = false;
  i16 jump_count = 0;

 private:
  u8 tx_buf_[8]{0};
};

/*
@brief:接受图传数据原始字节流并转发给VT03处理的串口设备
*/
namespace rm::device {
class TcReceiver : public Device{
public:
  TcReceiver() = delete;
  explicit TcReceiver(hal::SerialInterface &serial);

  void Begin();
  void RxCallback(const std::vector<u8> &data, u16 rx_len);

  // bool offline{false};
  // u16 offlinecounter{0};

private:
  hal::SerialInterface *serial_;
};
}
#endif