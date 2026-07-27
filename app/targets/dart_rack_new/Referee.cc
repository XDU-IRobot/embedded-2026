#include "Referee.hpp"

#include <librm.hpp>
#include "dart_sys.hpp"

volatile uint8_t glb_robot_id = 0;       // 机器人ID (FreeMaster)
volatile uint8_t glb_game_status = 0;    // 比赛进行阶段 (FreeMaster)
volatile u16 glb_stage_remain_time = 0;  // 当前阶段剩余时间（秒）(FreeMaster)
volatile u64 glb_sync_timestamp = 0;     // 裁判系统同步Unix时间戳 (FreeMaster)

volatile u8 glb_dart_remaining_time = 0;   // 己方飞镖发射剩余时间（秒）(FreeMaster)
volatile u8 glb_dart_last_hit_target = 0;  // 最近一次己方飞镖击中的目标 (FreeMaster)
volatile u8 glb_dart_enemy_hit_count = 0;  // 对方最近被击中目标累计被击中次数 (FreeMaster)
volatile u8 glb_dart_selected_target = 0;  // 飞镖此时选定的击打目标 (FreeMaster)

volatile u8 glb_dart_launch_opening_status = 1;  // 当前飞镖发射站的状态 (FreeMaster)
volatile u16 glb_dart_target_change_time = 0;    // 切换击打目标时的比赛剩余时间（秒）(FreeMaster)
volatile u16 glb_dart_latest_launch_cmd_time = 0;  // 最后一次操作手确定发射指令时的比赛剩余时间（秒）(FreeMaster)
volatile u16 glb_referee_head = 0;      // 环形缓冲区写指针 (FreeMaster)
volatile u32 glb_referee_cb_count = 0;  // 回调触发总次数 (FreeMaster)
volatile u16 glb_referee_last_cmd = 0;  // 最近一次回调的cmd_id (FreeMaster)
volatile u8 glb_referee_buf0 = 0;       // 环形缓冲区第0字节 (FreeMaster)
volatile u8 glb_referee_buf1 = 0;       // 环形缓冲区第1字节 (FreeMaster)
volatile u8 glb_referee_buf2 = 0;       // 环形缓冲区第2字节 (FreeMaster)
volatile u8 glb_referee_buf3 = 0;       // 环形缓冲区第3字节 (FreeMaster)
volatile u8 glb_referee_buf4 = 0;       // 环形缓冲区第4字节 (FreeMaster)
volatile u8 glb_referee_buf5 = 0;       // 环形缓冲区第5字节 (FreeMaster)
volatile u8 glb_referee_buf6 = 0;       // 环形缓冲区第6字节 (FreeMaster)

namespace rm::device {
RxReferee::RxReferee(rm::hal::SerialInterface &serial) : serial_(&serial) {
  this->serial_->AttachRxCallback([this](etl::span<const u8> data) { RxCallback(data); });
}

void RxReferee::Begin() {
  this->serial_->Start();

  // 注册裁判系统解析回调（主循环 Process() 解析完一帧后触发）
  dart_sys.referee_data_buffer->AttachCallback([](u16 cmd_id, u8 seq) {
    glb_referee_cb_count++;
    glb_referee_last_cmd = cmd_id;
    if (cmd_id == 0x0201) {
      // 飞镖id (0x0201)
      glb_robot_id = dart_sys.referee_data_buffer->data().robot_status.robot_id;  // 解析机器人的ID
    } else if (cmd_id == 0x0001) {
      // 比赛进行阶段 (0x0001)
      auto &gs = dart_sys.referee_data_buffer->data().game_status;
      auto progress = gs.game_progress;
      if (progress == 0)
        glb_game_status = 0;  // 未开始比赛
      else if (progress == 1)
        glb_game_status = 1;  // 准备阶段
      else if (progress == 2)
        glb_game_status = 2;  // 十五秒裁判系统自检
      else if (progress == 3)
        glb_game_status = 3;  // 五秒倒计时
      else if (progress == 4)
        glb_game_status = 4;  // 比赛进行中
      else if (progress == 5)
        glb_game_status = 5;  // 比赛结算
      glb_stage_remain_time = gs.stage_remain_time;
      glb_sync_timestamp = gs.SyncTimeStamp;
    } else if (cmd_id == 0x0105) {
      // 飞镖信息 (0x0105)
      auto &dart = dart_sys.referee_data_buffer->data().dart_info;  // 己方飞镖发射剩余时间，单位：秒
      glb_dart_remaining_time = dart.dart_remaining_time;           // 解析 dart_info 的位域
      u16 info = dart.dart_info;
      glb_dart_last_hit_target = info & 0x07;
      // bit 0-2: 最近一次己方飞镖击中的目标
      // 0=开局默认, 1=击中前哨站, 2=击中基地固定目标, 3=击中基地随机固定目标
      // 4=击中基地随机移动目标, 5=击中基地末端移动目标
      glb_dart_enemy_hit_count = (info >> 3) & 0x07;
      // bit 3-5: 对方最近被击中的目标累计被击中计数次数
      // 开局默认为0，至多为4
      glb_dart_selected_target = (info >> 6) & 0x07;
      // bit 6-8: 飞镖此时选定的击打目标
      // 0=开局默认或未选定/选定前哨站, 1=选中基地固定目标
      // 2=选中基地随机固定目标, 3=选中基地随机移动目标, 4=选中基地末端移动目标
    } else if (cmd_id == 0x020A) {
      // 飞镖客户端指令 (0x020A)
      auto &dart_cmd = dart_sys.referee_data_buffer->data().dart_client_cmd;
      glb_dart_launch_opening_status = dart_cmd.dart_launch_opening_status;
      // 当前飞镖发射站的状态
      // 0=已经开启, 1=关闭, 2=正在开启或关闭中
      glb_dart_target_change_time = dart_cmd.target_change_time;
      // 切换击打目标时的比赛剩余时间，单位：秒
      // 无/未切换动作时默认为0
      glb_dart_latest_launch_cmd_time = dart_cmd.latest_launch_cmd_time;
      // 最后一次操作手确定发射指令时的比赛剩余时间，单位：秒
      // 初始值为0
    }
  });
}

void RxReferee::RxCallback(etl::span<const u8> data) {
  // DMA 中断回调：只做快速拷贝到环形缓冲区，不进行耗时操作
  u16 head = head_;
  for (u16 i = 0; i < data.size(); i++) {
    rx_ring_buf_[head] = data[i];
    head = (head + 1) & (kRingBufSize - 1);  // 2的幂次取模，比 % 快
  }
  head_ = head;  // 原子更新写指针
  glb_referee_head = head;
}

void RxReferee::Process() {
  // 主循环：从环形缓冲区读取并喂入解析器
  u16 head = head_;
  if (tail_ != head) {
    glb_referee_buf0 = rx_ring_buf_[tail_];
    glb_referee_buf1 = rx_ring_buf_[(tail_ + 1) & (kRingBufSize - 1)];
    glb_referee_buf2 = rx_ring_buf_[(tail_ + 2) & (kRingBufSize - 1)];
    glb_referee_buf3 = rx_ring_buf_[(tail_ + 3) & (kRingBufSize - 1)];
    glb_referee_buf4 = rx_ring_buf_[(tail_ + 4) & (kRingBufSize - 1)];
    glb_referee_buf5 = rx_ring_buf_[(tail_ + 5) & (kRingBufSize - 1)];
    glb_referee_buf6 = rx_ring_buf_[(tail_ + 6) & (kRingBufSize - 1)];
  }
  while (tail_ != head) {
    *dart_sys.referee_data_buffer << rx_ring_buf_[tail_];
    tail_ = (tail_ + 1) & (kRingBufSize - 1);
  }
}
}  // namespace rm::device
