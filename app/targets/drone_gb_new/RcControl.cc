#include "RcControl.hpp"

namespace rm::device {
void ControlSource::act(DR16 &dr16_, VT03 &vt03_) {
  if (dr16_.online_status() == Device::kOk) {
    fact(dr16_);
  } else if (vt03_.online_status() == Device::kOk) {
    fact(vt03_);
  } else {
    fact();
  }
}
void ControlSource::fact(const DR16 &dr16_) {
  memcpy(this->axes_, dr16_.axes_, sizeof(this->axes_));
  memcpy(this->mouse_, dr16_.mouse_, sizeof(this->mouse_));
  memcpy(this->mouse_button_, dr16_.mouse_button_, sizeof(this->mouse_button_));
  memcpy(this->switches_, dr16_.switches_, sizeof(this->switches_));
  this->keyboard_key_ = dr16_.keyboard_key_;
}
void ControlSource::fact(const VT03 &vt03_) {
  memset(this->axes_, 0, sizeof(this->axes_));
  this->axes_[0] = vt03_.data().right_x * 660;
  this->axes_[1] = vt03_.data().right_y * 660;
  this->axes_[2] = vt03_.data().left_x * 660;
  this->axes_[3] = vt03_.data().left_y * 660;
  this->axes_[4] = vt03_.data().dial * 660;
  this->mouse_[0] = vt03_.data().mouse_x;
  this->mouse_[1] = vt03_.data().mouse_y;
  this->mouse_[2] = vt03_.data().mouse_z;
  this->mouse_button_[0] = vt03_.data().mouse_button_left;
  this->mouse_button_[1] = vt03_.data().mouse_button_right;
  this->switches_[0] = DR16::SwitchPosition::kUp;
  this->switches_[1] = DR16::SwitchPosition::kMid;
  this->keyboard_key_ = vt03_.data().keyboard_key;
};
void ControlSource::fact() {
  memset(this->axes_, 0, sizeof(this->axes_));
  memset(this->mouse_, 0, sizeof(this->mouse_));
  memset(this->mouse_button_, 0, sizeof(this->mouse_button_));
  this->switches_[0] = DR16::SwitchPosition::kMid;
  this->switches_[1] = DR16::SwitchPosition::kDown;
  this->keyboard_key_ = 0;
};
i16 ControlSource::left_x() const { return this->axes_[2]; }
i16 ControlSource::left_y() const { return this->axes_[3]; }
i16 ControlSource::right_x() const { return this->axes_[0]; }
i16 ControlSource::right_y() const { return this->axes_[1]; }
i16 ControlSource::dial() const { return this->axes_[4]; }
DR16::SwitchPosition ControlSource::switch_l() const { return this->switches_[1]; }
DR16::SwitchPosition ControlSource::switch_r() const { return this->switches_[0]; }
i16 ControlSource::mouse_x() const { return this->mouse_[0]; }
i16 ControlSource::mouse_y() const { return this->mouse_[1]; }
i16 ControlSource::mouse_z() const { return this->mouse_[2]; }
bool ControlSource::mouse_button_left() const { return this->mouse_button_[0]; }
bool ControlSource::mouse_button_right() const { return this->mouse_button_[1]; }
bool ControlSource::key(DR16::Key key) const { return (this->keyboard_key_ & static_cast<u16>(key)); }
}  // namespace rm::device