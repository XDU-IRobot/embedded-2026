#include "slidingmodec.hpp"
#include <cmath>

/**
 * @brief SMC 更新
 */
void SMC::Update(float target_pos, float angle_now, float angle_vel) {
  target = target_pos;

  if (reset_pending_) {
    Reset(angle_now);
    reset_pending_ = false;
  }

  angle = angle_now;
  ang_vel = angle_vel;
  float e_qp;
  float qp = q / p;
  float target_delta = target - target_last_;

  if (is_circular_) {
    error_ = rm::modules::Wrap(angle - target, -circular_cycle_ / 2, circular_cycle_ / 2);
    target_delta = rm::modules::Wrap(target - target_last_, -circular_cycle_ / 2, circular_cycle_ / 2) / dt;
  } else {
    error_ = angle - target;
    target_delta /= dt;
  }

  target_ddot_ = (target_delta - target_dot_) / dt;
  target_dot_ = target_delta;
  target_last_ = target;
  float error_dot = ang_vel - target_dot_;

  // error = angle - target;
  // float error_dot = ang_vel - target_dot;
  // target_ddot = (target - target_last) - target_dot;
  // target_dot = (target - target_last);

  if (fabs(error_) < error_eps) {
    u = 0;
    s_ = 0.0f;
    return;
  }

  if (error_ < 0)
    e_qp = -pow(abs(error_), qp);
  else
    e_qp = pow(abs(error_), qp);

  s_ = error_dot + C * e_qp;
  ds_ = -epsilon * Sat(s_) - K * s_;
  u = J * (target_ddot_ + ds_ - C * qp * error_dot * e_qp / (error_));

  // // 此处不平滑  TODO
  // if (abs(error_) < 1) {
  //   // error = angle - target;
  //
  //   s_ = C * error_ + error_dot;  // smc surface
  //   u = J * (target_ddot_ - C * error_dot - epsilon * Sat(s_) - K * s_);
  // }

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

  target_last_ = target;
  target_dot_ = 0.0f;
  target_ddot_ = 0.0f;

  error_ = 0.0f;
  error_last_ = 0.0f;
  s_ = 0.0f;
  ds_ = 0.0f;
  u = 0.0f;
}

/**
 * @brief SMC 初始化
 */
void SMC::SMCInit(float C, float K, float ref, float error_eps, float q, float p, float u_max, float J, float epsilon,
                  float dt = 1.0f) {
  this->C = C;
  this->K = K;
  this->target = ref;
  this->error_eps = error_eps;
  this->q = q;
  this->p = p;
  this->u_max = u_max;
  this->J = J;
  this->epsilon = epsilon;
  this->dt = dt;
}