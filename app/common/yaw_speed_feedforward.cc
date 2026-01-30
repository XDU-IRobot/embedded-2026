
#include "yaw_speed_feedforward.hpp"

YawSpeedFeedforward::YawSpeedFeedforward(float Ts, float k_ff) {
  Ts_ = Ts;
  k_ff_ = k_ff;
}

void YawSpeedFeedforward::Update(float target_yaw) {
  target_yaw_ = target_yaw;
  if (target_yaw_ - last_target_yaw_ > 5) last_target_yaw_ += 2 * M_PI;
  if (target_yaw_ - last_target_yaw_ < -5) last_target_yaw_ -= 2 * M_PI;
  yaw_speed_feedforward_ = (target_yaw_ - last_target_yaw_) / Ts_ * k_ff_;
  last_target_yaw_ = target_yaw_;
}

float YawSpeedFeedforward::GetYawSpeedFeedforward() { return yaw_speed_feedforward_; }
