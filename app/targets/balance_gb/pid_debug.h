#pragma once
#include <librm.hpp>

class Debug {
 public:
  f32 pitch_position_kp = 2.f, pitch_position_ki = 0.f, pitch_position_kd = 0.f;
  f32 pitch_speed_kp = 0.f, pitch_speed_ki = 0.f, pitch_speed_kd = 0.f;
  f32 pitch_position_max = 100.f, pitch_position_imax = 0.f;
  f32 pitch_speed_max = 0.f, pitch_speed_imax = 0.f;

  f32 yaw_position_kp = 0.f, yaw_position_ki = 0.f, yaw_position_kd = 0.f;
} pid_debug;
