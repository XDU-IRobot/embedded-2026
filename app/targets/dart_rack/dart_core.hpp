#pragma once

#include <librm.hpp>
#include "usb.hpp"
#include "encoder_counter.hpp"

// 状态机变量定义
enum class AbleState : uint8_t { kOff = 0, kOn = 1 };

enum class PhaseState : uint8_t { kUncomplete = 0, kDone = 1 };

enum class ModeState : uint8_t { kUnable = 0, kInit = 1, kload = 2, kAdd = 3, kAim = 4, kFire = 5 };

struct AutoMode {
  AbleState enabled = AbleState::kOff;
};

struct AdjustMode {
  AbleState enabled = AbleState::kOff;
};

struct ManualMode {
  AbleState enabled = AbleState::kOff;
  ModeState mode = ModeState::kUnable;
  PhaseState init = PhaseState::kUncomplete;
  PhaseState add = PhaseState::kUncomplete;
  PhaseState load = PhaseState::kUncomplete;
  PhaseState aim = PhaseState::kUncomplete;
  PhaseState fire = PhaseState::kUncomplete;

  // 初始化部分标志位
  bool is_yaw_init_done = false;  // yaw轴初始化完成标志位
  bool is_load_reset_done = false;
  bool is_trigger_reset_done = false;
  bool is_trigger_force_init_done = false;
  bool is_trigger_init_done = false;
  bool is_load_down_done = false;
  bool is_load_up_done = false;
  bool is_trigger_lock_done = false;
  void ManualModeClear()  // 清空所有标志位
  {
    mode = ModeState::kUnable;
    init = PhaseState::kUncomplete;
    add = PhaseState::kUncomplete;
    load = PhaseState::kUncomplete;
    aim = PhaseState::kUncomplete;
    fire = PhaseState::kUncomplete;
    is_yaw_init_done = false;  // yaw轴初始化完成标志位
    is_load_reset_done = false;
    is_trigger_reset_done = false;
    is_trigger_force_init_done = false;
    is_trigger_init_done = false;
    is_load_down_done = false;
    is_load_up_done = false;
    is_trigger_lock_done = false;
  }
};

struct DartState {
  AbleState unable = AbleState::kOn;
  AutoMode auto_mode;
  ManualMode manual_mode;
  AdjustMode adjust_mode;
};

inline void DartStateClear(DartState &state)  // 清空所有状态
{
  state.unable = AbleState::kOn;
  state.auto_mode.enabled = AbleState::kOff;
  state.manual_mode.enabled = AbleState::kOff;
  state.manual_mode.ManualModeClear();
}

inline void DartManualModeClear(ManualMode &mode)  // 清空手动模式状态
{
  mode.enabled = AbleState::kOff;
  mode.ManualModeClear();
}

enum class DartCount : uint8_t { kFirst = 0, kSecond = 1, kThird = 2, kFourth = 3 };

// 发射架核心结构体
struct DartRack {
  DartState state_{};

  // 硬件接口
  rm::hal::Can *can1_{nullptr};     ///< CAN 总线接口
  rm::hal::Serial *dbus_{nullptr};  ///< 遥控器串口接口

  // 设备
  rm::device::DR16 *rc_{nullptr};                    ///< 遥控器
  rm::device::M3508 *load_motor_r_{nullptr};         ///< 右上膛电机
  rm::device::M3508 *load_motor_l_{nullptr};         ///< 左上膛电机
  rm::device::M2006 *trigger_motor_{nullptr};        ///< 扳机活动电机
  rm::device::M2006 *trigger_motor_force_{nullptr};  ///< 扳机释放电机
  rm::device::M2006 *yaw_motor_{nullptr};            ///< yaw轴调节电机
  rm::device::JyMe02Can *yaw_encoder_{nullptr};      ///< 编码器

  // usb设备
  USBVisionReceive_SCM_t *vision_data_{nullptr};  ///< 视觉数据
  // PID 控制器
  rm::modules::PID load_motor_r_speed_pid_{};
  rm::modules::PID load_motor_l_speed_pid_{};
  rm::modules::PID trigger_motor_speed_pid_{};
  rm::modules::PID trigger_motor_force_pid_{};
  rm::modules::PID yaw_motor_speed_pid_{};

  // 编码器计圈器
  EncoderCounter load_motor_r_odometer_;
  EncoderCounter load_motor_l_odometer_;
  EncoderCounter trigger_motor_odometer_;
  EncoderCounter trigger_motor_force_odometer_;
  DartCount dart_count_{DartCount::kFirst};

  // 视觉结构体

  // yaw轴相关常量
  static constexpr float kYawEcdMax = 52.6000f;                      //< ME02 编码器最大值
  static constexpr float kYawEcdMin = 35.5000f;                      //< ME02 编码器最小值
  static constexpr float kYawEcd[4] = {47.0f, 44.0f, 48.0f, 50.0f};  //< ME02 编码器四发镖位置
  // 扳机相关常量
  static constexpr int32_t kTriggerEcdMax = 800000;
  static constexpr int32_t kTriggerEcdMin = 0;
  static constexpr int32_t kTriggerEcd[4] = {30000, 20000, 40000, 60000};  //< 扳机四发镖位置

  static constexpr int32_t kLoadEcdPerDart = 650000;  //< 上膛电机每发镖编码器最小增量
  /*
  上膛距离与扳机位置存在一定关系，理论上
  kLoadEcd[i] = kLoadEcdPerDart + kTriggerEcd[i]/扳机丝杆步长/扳机电机减速比*上膛丝杆步长*上膛电机减速比
  */
  static constexpr int32_t kLoadEcd[4] = {665000, 670000, 675000, 680000};  //< 上膛四发镖位置
  void Init();
  void Update();
};

extern DartRack *dart_rack;
