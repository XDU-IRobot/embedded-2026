#ifndef REFEREE_HPP
#define REFEREE_HPP

#include <librm.hpp>

using namespace rm;

// 裁判系统全局变量声明
extern volatile uint8_t g_robot_id;
extern volatile uint8_t game_status;

// 飞镖信息全局变量声明
extern volatile u8 dart_remaining_time;      // 己方飞镖发射剩余时间（秒）
extern volatile u8 dart_last_hit_target;     // 最近一次己方飞镖击中的目标
extern volatile u8 dart_enemy_hit_count;     // 对方最近被击中目标累计被击中次数
extern volatile u8 dart_selected_target;     // 飞镖此时选定的击打目标

// 飞镖客户端指令全局变量声明 (0x020A)
extern volatile u8 dart_launch_opening_status;   // 飞镖发射站状态：0=已开启, 1=关闭, 2=正在开启或关闭中
extern volatile u16 dart_target_change_time;     // 切换击打目标时的比赛剩余时间（秒）
extern volatile u16 dart_latest_launch_cmd_time; // 最后一次操作手确定发射指令时的比赛剩余时间（秒）

namespace rm::device {
class RxReferee : public Device {
 public:
  RxReferee() = delete;

  explicit RxReferee(rm::hal::SerialInterface &serial);

  void Begin();

  void Process();

  void RxCallback(etl::span<const u8> data);

 private:
  rm::hal::SerialInterface *serial_;
  static constexpr u16 kRingBufSize = 1024;  // 环形缓冲区大小，足够容纳多帧数据
  u8 rx_ring_buf_[kRingBufSize];
  volatile u16 head_{0};  // DMA 中断写入位置
  u16 tail_{0};           // 主循环读取位置
};
}  // namespace rm::device

#endif // REFEREE_HPP
