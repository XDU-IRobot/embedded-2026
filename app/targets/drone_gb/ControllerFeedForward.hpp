#ifndef BOARDC_CONTROLLERFEEDFORWARD_HPP
#define BOARDC_CONTROLLERFEEDFORWARD_HPP

#include <cmath>

#include "librm.hpp"

class Feedforward {
 public:
  Feedforward() = default;

  void Init(float Ts, float k_ff) {
    Ts_ = Ts;
    k_ff_ = k_ff;
    initialized_ = false;
    yaw_speed_feedforward_ = 0.f;
  }

  void SetMaxOutput(float max_output) { max_output_ = std::fabs(max_output); }

  void Reset(float current_target_yaw = 0.f) {
    target_yaw_ = current_target_yaw;
    last_target_yaw_ = current_target_yaw;
    yaw_speed_feedforward_ = 0.f;
    initialized_ = true;
  }

  float Update(float target_yaw) {
    target_yaw_ = target_yaw;

    // Prevent invalid dt from generating infinities.
    if (Ts_ <= 1e-6f) {
      last_target_yaw_ = target_yaw_;
      return 0.f;
    }

    // First update only aligns history to current target.
    if (!initialized_) {
      Reset(target_yaw_);
      return 0.f;
    }

    // Use shortest-path angle difference in [-pi, pi) to avoid wrap-direction flips.
    const float delta_yaw = rm::modules::Wrap(target_yaw_ - last_target_yaw_ + M_PI, 0.f, 2.f * M_PI) - M_PI;

    yaw_speed_feedforward_ = (delta_yaw / Ts_) * k_ff_;
    yaw_speed_feedforward_ = rm::modules::Clamp(yaw_speed_feedforward_, -max_output_, max_output_);

    last_target_yaw_ = target_yaw_;
    return yaw_speed_feedforward_;
  }

 private:
  float yaw_speed_feedforward_ = 0.f;
  float target_yaw_ = 0.f;
  float last_target_yaw_ = 0.f;
  float k_ff_ = 0.f;
  float Ts_ = 0.f;
  float max_output_ = 120.f;
  bool initialized_ = false;
};

#endif  // BOARDC_CONTROLLERFEEDFORWARD_HPP