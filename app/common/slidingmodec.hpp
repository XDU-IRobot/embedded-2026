#pragma once

#include "math.h"
#include <librm.hpp>

// 优质参数
// SMC YawSMC(20, 120, 0, 0.001, 21, 27, 25000, 0.8, 0.5, 1.0);
class SMC {
public:
  float C;
  float K;
  float target;
  float error_eps;
  float q;
  float p;
  float u_max;
  float J;
  float epsilon;
  float angle{0.f};
  float ang_vel{0.f};
  float dt{1.f};

  float u = 0.f;

  SMC(float C, float K, float ref, float error_eps, float q, float p, float u_max, float J, float epsilon, float dt)
      : C(C), K(K), target(ref), error_eps(error_eps), q(q), p(p), u_max(u_max), J(J), epsilon(epsilon), dt(dt) {}

  void SMCInit(float C, float K, float ref, float error_eps, float q, float p, float u_max, float J, float epsilon, float dt);
  void Update(float target_pos, float angle_now, float angle_vel);
  void Reset(float angle_now);

  SMC &SetCircularCycle(const float cycle, const bool is_enabled = true) {
    is_circular_ = is_enabled;
    circular_cycle_ = cycle;
    return *this;
  };

  // Getter
  float GetOutput() const { return u; }
  float GetError() const { return error_; }
  float GetSlidingSurface() const { return s_; }

 private:
  bool is_circular_{false};
  bool reset_pending_{true};

  float circular_cycle_{std::numeric_limits<float>::infinity()};

  float error_ = 0.f;
  float error_last_ = 0.f;
  float target_dot_ = 0.f;
  float target_ddot_ = 0.f;
  float target_last_ = 0.f;

  float s_ = 0.f;
  float ds_ = 0.f;

  static float Sat(float y) {
    if (fabs(y) <= 1)
      return y;
    else
      return Signal(y);
  }

  static int8_t Signal(float y) {
    if (y > 0)
      return 1;
    else if (y == 0)
      return 0;
    else
      return -1;
  }
};

// extern SMC YawSMC;
