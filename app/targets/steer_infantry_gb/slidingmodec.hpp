#pragma once

#include "math.h"
#include <librm.hpp>

// 优质参数
// SMC YawSMC(20, 120, 0, 0.001, 21, 27, 25000, 0.8, 0.5);
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
  float angle = 0.f;
  float ang_vel = 0.f;

  float u = 0.f;

  SMC(float C, float K, float ref, float error_eps, float q, float p, float u_max, float J, float epsilon)
      : C(C), K(K), target(ref), error_eps(error_eps), q(q), p(p), u_max(u_max), J(J), epsilon(epsilon) {};

  void SMCInit(float C, float K, float ref, float error_eps, float q, float p, float u_max, float J, float epsilon);
  void Update(float angle_now, float angle_vel);
  void SetTarget(const float target_pos) { target = target_pos; }
  void Reset(float angle_now);
  void Enable(bool enable);

  float GetError() const { return error; }
  float GetSlidingSurface() const { return s; }
  bool IsEnabled() const { return enabled_; }

 private:
  bool enabled_ = false;
  bool reset_pending_ = true;

  float error = 0.f;
  float error_last = 0.f;
  float target_dot = 0.f;
  float target_ddot = 0.f;
  float target_last = 0.f;

  float s = 0.f;
  float ds = 0.f;

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
