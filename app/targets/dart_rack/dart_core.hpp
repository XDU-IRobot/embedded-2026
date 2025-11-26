#ifndef STATE_DEFINITIONS
#define STATE_DEFINITIONS

#include <librm.hpp>
// 状态机变量定义

enum class AbleState : uint8_t { kOff = 0, kOn = 1 };

enum class PhaseState : uint8_t { kUncomplete = 0, kDone = 1 };

enum class ModeState : uint8_t { kUnable = 0, kAdd = 1, kInit = 2, kReload = 3, kChamber = 4, kAim = 5, kFire = 6 };

struct AutoMode {
  AbleState enabled = AbleState::kOff;
};

struct ManualMode {
  AbleState enabled = AbleState::kOff;
  ModeState mode = ModeState::kUnable;
  PhaseState init = PhaseState::kUncomplete;
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
};

inline void DartStateClear(DartState &state)  // 清空所有状态
{
  state.unable = AbleState::kOn;
  state.auto_mode.enabled = AbleState::kOff;
  state.manual_mode.enabled = AbleState::kOff;
  state.manual_mode.ManualModeClear();
}

struct DartRack {
 public:
  DartState *state;
  // 硬件接口指针
  rm::hal::Can *can1{nullptr};     ///< CAN 总线接口
  rm::hal::Serial *dbus{nullptr};  ///< 遥控器串口接口
  rm::device::DR16 *rc{nullptr};   //< 遥控器
  // can设备指针
  rm::device::M3508 *load_motor_r{nullptr};
  rm::device::M3508 *load_motor_l{nullptr};
  rm::device::M2006 *trigger_motor{nullptr};
  rm::device::M2006 *trigger_motor_force{nullptr};
  rm::device::M2006 *yaw_motor{nullptr};
  // PID 控制器
  rm::modules::PID *load_motor_r_speed_pid;
  rm::modules::PID *load_motor_l_speed_pid;
  rm::modules::PID *trigger_motor_speed_pid;
  rm::modules::PID *trigger_motor_force_pid;
  rm::modules::PID *yaw_motor_speed_pid;

 private:
 public:
  static void Init();
};

extern DartRack *dart_rack;
#endif
