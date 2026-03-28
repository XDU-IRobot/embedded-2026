#ifndef BOARDC_CONTROLLERFEEDFORWARD_HPP
#define BOARDC_CONTROLLERFEEDFORWARD_HPP

#include "librm.hpp"

class Feedforward {
public:
    Feedforward() = default;
    void Init(float Ts, float k_ff){Ts_=Ts;k_ff_=k_ff;}
    float Update(float target_yaw)
    {
        target_yaw_ = target_yaw;
        if (target_yaw_ - last_target_yaw_ > 5) last_target_yaw_ += 2 * M_PI;
        if (target_yaw_ - last_target_yaw_ < -5) last_target_yaw_ -= 2 * M_PI;
        yaw_speed_feedforward_ = (target_yaw_ - last_target_yaw_) / Ts_ * k_ff_;
        last_target_yaw_ = target_yaw_;
        return yaw_speed_feedforward_;
    }

private:
    float yaw_speed_feedforward_ = 0.f;
    float target_yaw_ = 0.f;
    float last_target_yaw_ = 0.f;
    float k_ff_ = 0.f;
    float Ts_ = 0.f;
};

#endif //BOARDC_CONTROLLERFEEDFORWARD_HPP