#pragma once

#include <etl/unordered_map.h>
#include <librm.hpp>

namespace rc_ch {

constexpr int LEFT_X = 3;
constexpr int LEFT_Y = 2;
constexpr int RIGHT_X = 0;
constexpr int RIGHT_Y = 1;

constexpr int SA = 4;
constexpr int SB = 5;
constexpr int SC = 6;
constexpr int SD = 7;
constexpr int SE = 8;
constexpr int SF = 9;
constexpr int SG = 10;
constexpr int SH = 11;

constexpr int LS = 12;
constexpr int RS = 13;
constexpr int LD = 14;
constexpr int RD = 15;
}  // namespace rc_ch

enum class SwitchPosition {
  kUnknown = 0u,
  kDown,
  kMid,
  kUp,
};

/**
 * @brief 基于rm::device::Sbus类对天地飞ET16s进行二次封装
 */
class WflyET16s : public rm::device::Sbus {
 public:
  WflyET16s(rm::hal::SerialInterface &serial) : Sbus(serial) {}

  [[nodiscard]] float left_x() const { return (static_cast<float>(channel(rc_ch::LEFT_X)) - 992.f) / 671.f; }
  [[nodiscard]] float left_y() const { return (static_cast<float>(channel(rc_ch::LEFT_Y)) - 992.f) / 671.f; }
  [[nodiscard]] float right_x() const { return (static_cast<float>(channel(rc_ch::RIGHT_X)) - 992.f) / 671.f; }
  [[nodiscard]] float right_y() const { return (static_cast<float>(channel(rc_ch::RIGHT_Y)) - 992.f) / 671.f; }

  [[nodiscard]] SwitchPosition switch_position(const i8 channel_num) const {
    if (channel_num >= 0 && channel_num <= 11) {
      if (channel(channel_num) == 1663) return SwitchPosition::kDown;
      if (channel(channel_num) == 992) return SwitchPosition::kMid;
      if (channel(channel_num) == 321) return SwitchPosition::kUp;
      return SwitchPosition::kUnknown;
    }
    return SwitchPosition::kUnknown;
  }

  [[nodiscard]] i16 wheel_position(const i8 channel_num) const {
    if (channel_num >= 12 && channel_num <= 15) {
      return channel(channel_num) - 992;
    }
    return 0;
  }
};
