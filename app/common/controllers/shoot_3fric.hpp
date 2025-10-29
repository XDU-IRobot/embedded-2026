
#pragma once

#include <librm.hpp>

/**
 * @brief 三摩擦轮发射机构控制器
 */
class Shoot3Fric {
 public:
  Shoot3Fric() {}

  void UpdateState(float yaw_position, float yaw_speed, float pitch_position, float pitch_speed) {}

  void Update(float dt = 1.f) {}

  // getters
  auto &pid() { return pid_; }

 private:
  struct {
    rm::modules::PID yaw_speed, yaw_position, pitch_speed, pitch_position;
  } pid_;
};