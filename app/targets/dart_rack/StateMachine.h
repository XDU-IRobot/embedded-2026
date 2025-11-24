#ifndef BOARDC_STATEMACHINE_H
#define BOARDC_STATEMACHINE_H

#include <cstdint>

// 状态机变量定义
namespace dart_rack_state {

enum class AbleState : uint8_t {
  kOff = 0,
  kOn  = 1
};

enum class PhaseState : uint8_t {
  kUncomplete = 0,
  kDone       = 1
};

struct AutoMode {
  AbleState enabled = AbleState::kOff;
};

struct ManualMode {
  AbleState enabled = AbleState::kOff;
  PhaseState init = PhaseState::kUncomplete;
  PhaseState reload = PhaseState::kUncomplete;
  PhaseState chamber = PhaseState::kUncomplete;
  PhaseState aim = PhaseState::kUncomplete;
  PhaseState fire = PhaseState::kUncomplete;
};

struct DartState {
  AbleState unable = AbleState::kOn;
  AutoMode auto_mode;
  ManualMode manual_mode;
};

}

#endif // BOARDC_STATEMACHINE_H
