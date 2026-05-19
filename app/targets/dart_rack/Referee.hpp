#ifndef REFEREE_HPP
#define REFEREE_HPP

#include <librm.hpp>

using namespace rm;

// 裁判系统全局变量声明 (FreeMaster)
extern volatile uint8_t glb_robot_id;
extern volatile uint8_t glb_game_status;
extern volatile u16 glb_stage_remain_time;   // 当前阶段剩余时间（秒）
extern volatile u64 glb_sync_timestamp;      // 裁判系统同步Unix时间戳

// 飞镖信息全局变量声明 (FreeMaster)
extern volatile u8 glb_dart_remaining_time;      // 己方飞镖发射剩余时间（秒）
extern volatile u8 glb_dart_last_hit_target;     // 最近一次己方飞镖击中的目标
extern volatile u8 glb_dart_enemy_hit_count;     // 对方最近被击中目标累计被击中次数
extern volatile u8 glb_dart_selected_target;     // 飞镖此时选定的击打目标

// 飞镖客户端指令全局变量声明 (0x020A) (FreeMaster)
extern volatile u8 glb_dart_launch_opening_status;   // 飞镖发射站状态：0=已开启, 1=关闭, 2=正在开启或关闭中
extern volatile u16 glb_dart_target_change_time;     // 切换击打目标时的比赛剩余时间（秒）
extern volatile u16 glb_dart_latest_launch_cmd_time; // 最后一次操作手确定发射指令时的比赛剩余时间（秒）
extern volatile u16 glb_referee_head;                // 环形缓冲区写指针
extern volatile u32 glb_referee_cb_count;            // 回调触发总次数
extern volatile u16 glb_referee_last_cmd;            // 最近一次回调的cmd_id
extern volatile u8 glb_referee_buf0;                 // 环形缓冲区第0字节
extern volatile u8 glb_referee_buf1;                 // 环形缓冲区第1字节
extern volatile u8 glb_referee_buf2;                 // 环形缓冲区第2字节
extern volatile u8 glb_referee_buf3;                 // 环形缓冲区第3字节
extern volatile u8 glb_referee_buf4;                 // 环形缓冲区第4字节
extern volatile u8 glb_referee_buf5;                 // 环形缓冲区第5字节
extern volatile u8 glb_referee_buf6;                 // 环形缓冲区第6字节

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
