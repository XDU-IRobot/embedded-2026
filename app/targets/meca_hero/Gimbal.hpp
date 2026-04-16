#pragma once

#include "main.hpp"
#include "State.hpp"
#include "librm.hpp"

class GimbalHero {
public:
  GimbalHero() = default;

  void GimbalUpdate();

  //监测变量
  float euler_yaw_{0};
  float euler_pitch_{0};
  float euler_roll_{0};

private:
  //调参变量
  float gyro_rectification_{0}; // 陀螺仪Z轴修正值
  int pitch_ff_{}; //pitch轴PID前馈

  bool Enable();
  void UnableUpdate();
  void EnableUpdate();
  void AhrsUpdate();
  void AimbotControl();
};