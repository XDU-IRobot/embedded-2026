#ifndef CONTROLSOURCE_H
#define CONTROLSOURCE_H

#include "librm.hpp"

using namespace rm::device;
using namespace rm;

namespace rm::device {
class ControlSource {
public:
  ControlSource() = default;

  void act(DR16 &dr16_, VT03 &vt03_);
  void fact(const DR16 &dr16_);
  void fact(const VT03 &vt03_);
  void fact();

  [[nodiscard]] i16 left_x() const;
  [[nodiscard]] i16 left_y() const;
  [[nodiscard]] i16 right_x() const;
  [[nodiscard]] i16 right_y() const;
  [[nodiscard]] i16 dial() const;
  [[nodiscard]] DR16::SwitchPosition switch_l() const;
  [[nodiscard]] DR16::SwitchPosition switch_r() const;
  [[nodiscard]] i16 mouse_x() const;
  [[nodiscard]] i16 mouse_y() const;
  [[nodiscard]] i16 mouse_z() const;
  [[nodiscard]] bool mouse_button_left() const;
  [[nodiscard]] bool mouse_button_right() const;
  [[nodiscard]] bool key(DR16::Key key) const;

private:
  i16 axes_[5]{0};   // [0]: right_x, [1]: right_y, [2]: left_x, [3]: left_y, [4]: dial; 取值范围:-660~660;
  i16 mouse_[3]{0};  // [0]: x, [1]: y, [2]: z; 取值范围:-32768~32767;
  bool mouse_button_[2]{false};                           // [0]: left, [1]: right
  DR16::SwitchPosition switches_[2]{DR16::SwitchPosition::kUnknown};  // [0]: right, [1]: left
  u16 keyboard_key_;                                      // 每一位代表一个键，0为未按下，1为按下
};
}  // namespace rm::device

#endif /* CONTROLSOURCE_H */