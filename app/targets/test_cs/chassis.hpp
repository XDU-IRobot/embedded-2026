
#pragma once

#include <librm.hpp>
#include "main.h"

using namespace rm::device;
using namespace rm;

inline f32 speed_;
class Chassis {
  float RadsToRpm(float rads) {
    f32 speed =rads * 60.f / (2.f * (float)M_PI);
    if(speed > 0.f) {
      return 10*speed;
    }else if(speed < 0.f){
      return -10*speed;
    }else{
      return 0;
    }
  }
  int calc_dir_lf(float v){
    if(v>0.f){
      return 0;
    }else if(v<0.f){
      return 1;
    }else{
      return 0;
    }
  }
  int calc_dir_lb(float v){
    if(v>0.f){
      return 1;
    }else if(v<0.f){
      return 0;
    }else{
      return 0;
    }
  }
  int calc_dir_rf(float v){
    if(v>0.f){
      return 0;
    }else if(v<0.f){
      return 1;
    }else{
      return 0;
    }
  }
  int calc_dir_rb(float v){
    if(v>0.f){
      return 1;
    }else if(v<0.f){
      return 0;
    }else{
      return 0;
    }
  }

 public:
  Chassis(rm::device::ZdtStepper *lf, device::ZdtStepper *lb, device::ZdtStepper *rf, device::ZdtStepper *rb)
      : motors_{lf, lb, rf, rb} {}
  ~Chassis() = default;

  void UpdateTargetSpeed(float x, float y, float w) {
    // 计算目标轮速
    const auto wheel_speeds = kinematics_.Forward(x, y, w);

    if (!enabled_)
        return;

    // 静态变量保存上一次执行的阶段
    static uint8_t phase = 0;

    switch (phase) {
      case 0:
        motors_.lf->MotorVelCtrl(
            calc_dir_lf(wheel_speeds.lf_speed),
            RadsToRpm(wheel_speeds.lf_speed),
            0, true);
        break;

      case 1:
        motors_.lb->MotorVelCtrl(
            calc_dir_lb(wheel_speeds.lr_speed),
            RadsToRpm(wheel_speeds.lr_speed),
            0, true);
        break;

      case 2:
        motors_.rf->MotorVelCtrl(
            calc_dir_rf(wheel_speeds.rf_speed),
            RadsToRpm(wheel_speeds.rf_speed),
            0, true);
        break;

      case 3:
        motors_.rb->MotorVelCtrl(
            calc_dir_rb(wheel_speeds.rr_speed),
            RadsToRpm(wheel_speeds.rr_speed),
            0, true);

        break;
      case 4:
        motors_.rb->MotorSyncCtrl();   // 同步发送在最后一相完成
        break;
    }

    // 循环执行下一个阶段
    phase = (phase + 1) % 5;
}

  void Update() {}
  void Enable(bool enable) {
    enabled_ = enable;
    if (!enabled_) {
      motors_.lf->MotorVelCtrl(1,0.f, 0, true);
      motors_.lb->MotorVelCtrl(1,0.f, 0, true);
      motors_.rf->MotorVelCtrl(1,0.f, 0, true);
      motors_.rb->MotorVelCtrl(1,0.f, 0, true);
      motors_.lf->MotorSyncCtrl();
      //motors_.lb->MotorSyncCtrl();
      //motors_.rf->MotorSyncCtrl();
      //motors_.rb->MotorSyncCtrl();
    }
  }

  [[nodiscard]] auto &motors() const { return motors_; }
  [[nodiscard]] bool enabled() const { return enabled_; }

 private:
  bool enabled_{false};
  rm::modules::MecanumChassis kinematics_{0.3f, 0.3f};
  struct {
    device::ZdtStepper *lf{nullptr}, *lb{nullptr}, *rf{nullptr}, *rb{nullptr};
  } motors_{};
};