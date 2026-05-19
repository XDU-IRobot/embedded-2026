#ifndef BOARDC_ANGLEDIFF2_H
#define BOARDC_ANGLEDIFF2_H
#include <librm.hpp>
class AngleDiff2 {
 public:
  AngleDiff2() = default;

  void Reset(float value) {
    initialized_ = true;
    last_value_ = value;
    last_vel_ = 0.0f;

    vel_ = 0.0f;
    acc_ = 0.0f;
  }

  void Update(float value, float dt, bool is_angle_wrap = false) {
    if (dt <= 1e-6f) {
      vel_ = 0.0f;
      acc_ = 0.0f;
      return;
    }

    if (!initialized_) {
      Reset(value);
      return;
    }

    float delta = value - last_value_;

    if (is_angle_wrap) {
      delta = rm::modules::Wrap(delta, -M_PI, M_PI);
    }

    const float raw_vel = delta / dt;
    const float raw_acc = (raw_vel - last_vel_) / dt;

    vel_ = vel_alpha_ * raw_vel + (1.0f - vel_alpha_) * vel_;
    acc_ = acc_alpha_ * raw_acc + (1.0f - acc_alpha_) * acc_;

    last_value_ = value;
    last_vel_ = vel_;
  }

  void SetFilter(float vel_alpha, float acc_alpha) {
    vel_alpha_ = rm::modules::Clamp(vel_alpha, 0.0f, 1.0f);
    acc_alpha_ = rm::modules::Clamp(acc_alpha, 0.0f, 1.0f);
  }

  float vel() const { return vel_; }
  float acc() const { return acc_; }

 private:
  bool initialized_ = false;

  float last_value_ = 0.0f;
  float last_vel_ = 0.0f;

  float vel_ = 0.0f;
  float acc_ = 0.0f;

  float vel_alpha_ = 0.35f;
  float acc_alpha_ = 0.20f;
};

#endif  // BOARDC_ANGLEDIFF2_H
