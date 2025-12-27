
#include "fsm.hpp"

#include "global.hpp"
#include "remote_control.hpp"

#include <librm.hpp>

f32 rc1 = 0.f;
f32 x_speed_ = 0.f;
f32 y_speed_ = 0.f;
f32 yaw_speed_ = 0.f;
f32 w_speed_ = 0.f;

f32 Limit(f32 speed) {
  if (speed > 2.f) {
    speed = 2.f;
  } else if (speed < -2.f) {
    speed = -2.f;
  } else {
    speed = speed;
  }

  return speed;
}

void Fsm::Transit(State new_mode) {
  // put transit logic below
  if (new_mode != mode_) {
    if (new_mode == State::kNoForce) {
      global.chassis->Enable(false);
    } else {
      global.chassis->Enable(true);
    }
  }
  // put transit logic above
  mode_ = new_mode;
}

void Fsm::Update() {
  if (!global.rc->device_rc.all_device_ok()) {
    Transit(State::kAuto);
  } else {
    switch (global.rc->rc->switch_r()) {  ///< right switch
      case DR16::SwitchPosition::kDown: {
        Transit(State::kNoForce);
        break;
      }
      case DR16::SwitchPosition::kMid: {
        Transit(State::kManual);
        break;
      }
      case DR16::SwitchPosition::kUp: {
        Transit(State::kAuto);
        break;
      }
      default: {
        break;
      }
    }
    //}

    switch (mode_) {
      case State::kNoForce: {
        rc1 = global.rc->rc->left_y();
        break;
      }
      case State::kManual: {
        rc1 = global.rc->rc->left_x();
        global.chassis->UpdateTargetSpeed(global.rc->rc->left_x() / 330.f, -global.rc->rc->left_y() / 330.f,
                                          global.rc->rc->dial() / 200.f);
        break;
      }
      case State::kAuto: {
        x_speed_ = Limit(global.minipc->rx_buffer_.x_speed);
        y_speed_ = Limit(global.minipc->rx_buffer_.y_speed);
        yaw_speed_ = Limit(global.minipc->rx_buffer_.yaw_speed);
        w_speed_ = Limit(global.minipc->rx_buffer_.w_speed);

        global.chassis->UpdateTargetSpeed(x_speed_, -y_speed_, yaw_speed_);
        break;
      }
      default: {
        break;
      }
    }
    global.chassis->Update();
    // device::DjiMotor<>::SendCommand();
  }
}