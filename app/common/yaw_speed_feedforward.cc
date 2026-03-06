/**
 * @file   yaw_speed_feedforward.cc
 * @brief  实现速度前馈，不仅仅只是yaw，也可以加入pitch
 **/
#include "yaw_speed_feedforward.hpp"

/**
 * @brief          构造函数
 * @param[in]      Ts    采样时间
 * @param[in]      k_ff  前馈系数，越大越快
 * @returns        无
 */
YawSpeedFeedforward::YawSpeedFeedforward(float Ts, float k_ff) {
  Ts_ = Ts;
  k_ff_ = k_ff;
}

/**
 * @brief          Update
 * @param[in]      target_yaw   目标角度
 * @returns        无
 */
void YawSpeedFeedforward::Update(float target_yaw) {
  target_yaw_ = target_yaw;
  if (target_yaw_ - last_target_yaw_ > 5) last_target_yaw_ += 2 * M_PI;
  if (target_yaw_ - last_target_yaw_ < -5) last_target_yaw_ -= 2 * M_PI;
  yaw_speed_feedforward_ = (target_yaw_ - last_target_yaw_) / Ts_ * k_ff_;
  last_target_yaw_ = target_yaw_;
}

/**
 * @brief          GetYawSpeedFeedforward
 * @param[in]      无
 * @returns        速度前馈
 */
float YawSpeedFeedforward::GetYawSpeedFeedforward() { return yaw_speed_feedforward_; }
