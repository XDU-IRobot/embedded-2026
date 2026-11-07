#include "slidingmodec.hpp"
#include <cmath>

/**
 * @brief SMC 更新
 */
void SMC::Update(float target_pos, float angle_now, float angle_vel, float dt) {
  target = target_pos;

  if (!enabled_) {
    u = 0.0f;
    s = 0.0f;
    return;
  }

  if (reset_pending_) {
    Reset(angle_now);
    reset_pending_ = false;
  }

  angle = angle_now;
  ang_vel = angle_vel;
  float e_qp;
  float qp = q / p;

  error = rm::modules::Wrap(angle - target, -M_PI, M_PI);
  float error_dot = ang_vel - target_dot;
  const float target_delta = rm::modules::Wrap(target - target_last, -M_PI, M_PI) / dt;
  target_ddot = (target_delta - target_dot) / dt;
  target_dot = target_delta;
  target_last = target;

  // error = angle - target;
  // float error_dot = ang_vel - target_dot;
  // target_ddot = (target - target_last) - target_dot;
  // target_dot = (target - target_last);

  if (fabs(error) < error_eps) {
    u = 0;
    s = 0.0f;
    return;
  }

  if (error < 0)
    e_qp = -pow(abs(error), qp);
  else
    e_qp = pow(abs(error), qp);

  s = error_dot + C * e_qp;
  ds = -epsilon * Sat(s) - K * s;
  u = J * (target_ddot + ds - C * qp * error_dot * e_qp / (error));

  if (abs(error) < 1 * M_PI / 180.0f) {
    // error = angle - target;

    s = C * error + error_dot;  // smc surface
    u = J * (target_ddot - C * error_dot - epsilon * Sat(s) - K * s);
  }

  if (u > u_max) u = u_max;
  if (u < -u_max) u = -u_max;

  // target_last = target;
}

/**
 * @brief SMC 重置
 */
void SMC::Reset(float angle_now) {
  angle = angle_now;
  ang_vel = 0.0f;

  target_last = target;
  target_dot = 0.0f;
  target_ddot = 0.0f;

  error = 0.0f;
  error_last = 0.0f;
  s = 0.0f;
  ds = 0.0f;
  u = 0.0f;
}

/**
 * @brief SMC 初始化
 */
void SMC::SMCInit(float C, float K, float ref, float error_eps, float q, float p, float u_max, float J, float epsilon) {
  this->C = C;
  this->K = K;
  this->target = ref;
  this->error_eps = error_eps;
  this->q = q;
  this->p = p;
  this->u_max = u_max;
  this->J = J;
  this->epsilon = epsilon;
}

/**
 * @brief SMC 使能
 */
void SMC::Enable(bool enable) {
  enabled_ = enable;

  if (!enable) {
    u = 0.0f;
    s = 0.0f;
    reset_pending_ = true;
  }
}