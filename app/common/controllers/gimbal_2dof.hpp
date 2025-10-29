
#pragma once

#include <librm.hpp>

/**
 * @brief 二轴云台控制器
 */
class Gimbal2Dof {
 public:
  Gimbal2Dof() { pid_.yaw_position.SetCircular(true).SetCircularCycle(M_PI * 2); }

  void UpdateState(float yaw_position, float yaw_speed, float pitch_position, float pitch_speed) {}

  void Update(float dt = 1.f) {}

  // getters
  auto &pid() { return pid_; }

 private:
  struct {
    rm::modules::PID yaw_speed, yaw_position, pitch_speed, pitch_position;
  } pid_;
};