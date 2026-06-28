#pragma once

#include <librm.hpp>

#if WHEEL_LEGGED_ROBOT_VARIANT == 1
constexpr int kFrictionWheelCount = 6;
constexpr uint16_t kFwMotorIds[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
constexpr float kFwTargetSpeedRpm = 3400.0f;
constexpr float kFwSpeedKp = 30.0f;
constexpr float kFwSpeedKi = 0.01f;
constexpr float kFwSpeedKd = 0.0f;
constexpr float kFwSpeedMaxOut = 10000.0f;
constexpr float kFwSpeedMaxIout = 0.0f;
constexpr float kFwBrakeTargetRpm = -100.0f;
constexpr float kFwBrakeThresholdRpm = 500.0f;
#else
constexpr int kFrictionWheelCount = 2;
constexpr float kFricSpeedTargetRpm = 6200.0f;
constexpr float kFricSpeedKp = 10.0f;
constexpr float kFricSpeedKi = 0.0f;
constexpr float kFricSpeedKd = 0.0f;
constexpr float kFricSpeedMaxOut = 16000.0f;
constexpr float kFricSpeedMaxIout = 2000.0f;
constexpr float kFricBrakeTargetRpm = -100.0f;
constexpr float kFricBrakeThresholdRpm = 500.0f;
#endif

constexpr float kFricSpeedStepRpm = 20.0f;

class ShootCtrl {
 public:
  void Init(rm::hal::CanInterface& can) {
    can_ = &can;

#if WHEEL_LEGGED_ROBOT_VARIANT == 1
    for (int i = 0; i < kFrictionWheelCount; ++i) {
      fw_motors_[i].emplace(can, kFwMotorIds[i], i > 0);
      fw_speed_pid_[i].emplace(kFwSpeedKp, kFwSpeedKi, kFwSpeedKd, kFwSpeedMaxOut, kFwSpeedMaxIout);
    }
#else
    fric_left_.emplace(can, 0x01);
    fric_right_.emplace(can, 0x04);
    fric_left_pid_.emplace(kFricSpeedKp, kFricSpeedKi, kFricSpeedKd, kFricSpeedMaxOut, kFricSpeedMaxIout);
    fric_right_pid_.emplace(kFricSpeedKp, kFricSpeedKi, kFricSpeedKd, kFricSpeedMaxOut, kFricSpeedMaxIout);
#endif

    fric_speed_target_rpm_ =
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
        kFwTargetSpeedRpm;
#else
        kFricSpeedTargetRpm;
#endif
  }

  void Update(bool enter_shoot) {
    if (!can_) return;

#if WHEEL_LEGGED_ROBOT_VARIANT == 1
    for (int i = 0; i < kFrictionWheelCount; ++i) {
      if (enter_shoot) {
        const float target = (i % 2 == 0) ? fric_speed_target_rpm_ : -fric_speed_target_rpm_;
        fw_speed_pid_[i]->Update(target, fw_motors_[i]->rpm());
        fw_motors_[i]->SetCurrent(fw_speed_pid_[i]->out());
      } else {
        if (fw_motors_[i]->rpm() >= kFwBrakeThresholdRpm) {
          fw_speed_pid_[i]->Update(kFwBrakeTargetRpm, fw_motors_[i]->rpm());
          fw_motors_[i]->SetCurrent(fw_speed_pid_[i]->out());
        } else {
          fw_speed_pid_[i]->Clear();
          fw_motors_[i]->SetCurrent(0);
        }
      }
    }
    rm::device::DjiMotorBase::SendCommand(*can_);
#else
    if (enter_shoot) {
      fric_left_pid_->Update(fric_speed_target_rpm_, fric_left_->rpm());
      fric_left_->SetCurrent(fric_left_pid_->out());
      fric_right_pid_->Update(-fric_speed_target_rpm_, fric_right_->rpm());
      fric_right_->SetCurrent(fric_right_pid_->out());
    } else {
      if (fric_left_->rpm() >= kFricBrakeThresholdRpm) {
        fric_left_pid_->Update(kFricBrakeTargetRpm, fric_left_->rpm());
        fric_left_->SetCurrent(fric_left_pid_->out());
      } else {
        fric_left_pid_->Clear();
        fric_left_->SetCurrent(0);
      }
      if (fric_right_->rpm() <= -kFricBrakeThresholdRpm) {
        fric_right_pid_->Update(-kFricBrakeTargetRpm, fric_right_->rpm());
        fric_right_->SetCurrent(fric_right_pid_->out());
      } else {
        fric_right_pid_->Clear();
        fric_right_->SetCurrent(0);
      }
    }
    rm::device::DjiMotorBase::SendCommand(*can_);
#endif
  }

  void AdjustSpeed(float delta) { fric_speed_target_rpm_ += delta; }

  float fric_speed_target_rpm() const { return fric_speed_target_rpm_; }

#if WHEEL_LEGGED_ROBOT_VARIANT == 1
  int16_t fw_rpm(int index) const {
    if (index >= 0 && index < kFrictionWheelCount && fw_motors_[index]) return fw_motors_[index]->rpm();
    return 0;
  }
#else
  int16_t fric_left_rpm() const { return fric_left_ ? fric_left_->rpm() : 0; }
  int16_t fric_right_rpm() const { return fric_right_ ? fric_right_->rpm() : 0; }
#endif

 private:
  rm::hal::CanInterface* can_{nullptr};
  float fric_speed_target_rpm_{0.0f};

#if WHEEL_LEGGED_ROBOT_VARIANT == 1
  std::optional<rm::device::M3508> fw_motors_[kFrictionWheelCount];
  std::optional<rm::modules::PID> fw_speed_pid_[kFrictionWheelCount];
#else
  std::optional<rm::device::M3508> fric_left_;
  std::optional<rm::device::M3508> fric_right_;
  std::optional<rm::modules::PID> fric_left_pid_;
  std::optional<rm::modules::PID> fric_right_pid_;
#endif
};
