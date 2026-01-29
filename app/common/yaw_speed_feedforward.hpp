#include <librm.hpp>

#ifndef BOARDC_YAW_SPEED_FEEDFORWARD_HPP
#define BOARDC_YAW_SPEED_FEEDFORWARD_HPP

class YawSpeedFeedforward {
  public:
    YawSpeedFeedforward() = default;

    YawSpeedFeedforward(float Ts,float k_ff);

    void Update(float target_yaw);

    float GetYawSpeedFeedforward();

  private:
    float yaw_speed_feedforward_ = 0.f;
    float target_yaw_ = 0.f;
    float last_target_yaw_ = 0.f;
    float k_ff_=0.f;
    float Ts_=0.f;
};


#endif  // BOARDC_YAW_SPEED_FEEDFORWARD_HPP
