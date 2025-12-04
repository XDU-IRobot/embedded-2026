#ifndef STATE_DEFINITIONS
#define STATE_DEFINITIONS

#include <librm.hpp>
#include "device_encoder.hpp"
#include "dji_odometer.hpp"
// 状态机变量定义

enum class AbleState : uint8_t { kOff = 0, kOn = 1 };

enum class PhaseState : uint8_t { kUncomplete = 0, kDone = 1 };

enum class ModeState : uint8_t { kUnable = 0, kAdd = 1, kInit = 2, kReload = 3, kChamber = 4, kAim = 5, kFire = 6 };

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
  PhaseState reload = PhaseState::kUncomplete;
  PhaseState chamber = PhaseState::kUncomplete;
  PhaseState aim = PhaseState::kUncomplete;
  PhaseState fire = PhaseState::kUncomplete;
  void ManualModeClear()  // 清空所有标志位
  {
    mode = ModeState::kUnable;
    init = PhaseState::kUncomplete;
    reload = PhaseState::kUncomplete;
    chamber = PhaseState::kUncomplete;
    aim = PhaseState::kUncomplete;
    fire = PhaseState::kUncomplete;
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

enum class DartCount : uint8_t { kFirst = 0, kSecond = 1, kThird = 2, kFourth = 3 };

// 发射架核心结构体
struct DartRack {
 public:
  DartState *state{};
  // 硬件接口指针
  rm::hal::Can *can1{nullptr};     ///< CAN 总线接口
  rm::hal::Serial *dbus{nullptr};  ///< 遥控器串口接口
  rm::device::DR16 *rc{nullptr};   //< 遥控器
  // can设备指针
  rm::device::M3508 *load_motor_r{nullptr};         //< 右上膛电机
  rm::device::M3508 *load_motor_l{nullptr};         //< 左上膛电机
  rm::device::M2006 *trigger_motor{nullptr};        //< 扳机活动电机
  rm::device::M2006 *trigger_motor_force{nullptr};  //< 扳机释放电机
  rm::device::M2006 *yaw_motor{nullptr};            //< yaw轴调节电机
                                                    // 编码器设备指针
  rm::device::ME02 *yaw_encoder{nullptr};
  // PID 控制器
  rm::modules::PID *load_motor_r_speed_pid{};
  rm::modules::PID *load_motor_l_speed_pid{};
  rm::modules::PID *trigger_motor_speed_pid{};
  rm::modules::PID *trigger_motor_force_pid{};
  rm::modules::PID *yaw_motor_speed_pid{};

  // DJI电机计圈器
  rm::device::DjiOdometer *load_motor_r_odometer{nullptr};
  rm::device::DjiOdometer *load_motor_l_odometer{nullptr};
  rm::device::DjiOdometer *trigger_motor_odometer{nullptr};
  rm::device::DjiOdometer *trigger_motor_force_odometer{nullptr};
  DartCount dart_count = DartCount::kFirst;

  // yaw轴相关常量
  static constexpr float_t kYawEcdMax = 52.6000f;                      //< ME02 编码器最大值
  static constexpr float_t kYawEcdMin = 35.5000f;                      //< ME02 编码器最小值
  static constexpr float_t kYawEcd[4] = {47.0f, 44.0f, 48.0f, 50.0f};  //< ME02 编码器四发镖位置
  static constexpr float_t kYawEcdSecond = 55.0f;

 private:
 public:
  static void Init();
  static void DataUpdate();
  static void DataSend();
};

extern DartRack *dart_rack;
#endif
