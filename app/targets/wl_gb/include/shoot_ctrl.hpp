#pragma once

#include <cmath>

#include <librm.hpp>

#if WHEEL_LEGGED_ROBOT_VARIANT == 1
constexpr int kFrictionWheelCount = 6;
constexpr uint16_t kFwMotorIds[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
// 吊射参数
constexpr float kFwLobTargetSpeedRpm_123 = 5050.0f;
constexpr float kFwLobTargetSpeedRpm_456 = 5100.0f;
// 平时参数
constexpr float kFwNormalTargetSpeedRpm_123 = 3650.0f;
constexpr float kFwNormalTargetSpeedRpm_456 = 3650.0f;

constexpr float kFwSpeedKp = 10.0f;
constexpr float kFwSpeedKi = 0.0f;
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

constexpr float kShotReadyThresholdRpm = 50.0f;
constexpr float kShotDropThresholdRpm = 140.0f;

class ShootCtrl {
 public:
  void Init(rm::hal::CanInterface& can) {
    can_ = &can;

#if WHEEL_LEGGED_ROBOT_VARIANT == 1
    for (int i = 0; i < kFrictionWheelCount; ++i) {
      fw_motors_[i].emplace(can, kFwMotorIds[i]);
      fw_speed_pid_[i].emplace(kFwSpeedKp, kFwSpeedKi, kFwSpeedKd, kFwSpeedMaxOut, kFwSpeedMaxIout);
    }
#else
    fric_left_.emplace(can, 0x01);
    fric_right_.emplace(can, 0x02);
    fric_left_pid_.emplace(kFricSpeedKp, kFricSpeedKi, kFricSpeedKd, kFricSpeedMaxOut, kFricSpeedMaxIout);
    fric_right_pid_.emplace(kFricSpeedKp, kFricSpeedKi, kFricSpeedKd, kFricSpeedMaxOut, kFricSpeedMaxIout);
#endif

    fric_speed_target_rpm_ =
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
        kFwNormalTargetSpeedRpm_123;
#else
        kFricSpeedTargetRpm;
#endif
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
    fric_speed_target_rpm_456_ = kFwNormalTargetSpeedRpm_456;
#endif
  }

  void Update(bool enter_shoot) {
    shot_this_cycle_ = false;
    if (!can_) return;

#if WHEEL_LEGGED_ROBOT_VARIANT == 1
    for (int i = 0; i < kFrictionWheelCount; ++i) {
      // ID 2,5 正转, 其余反转
      const bool positive = (i == 1 || i == 4);
      const float speed_target = (i < 3) ? fric_speed_target_rpm_ : fric_speed_target_rpm_456_;
      if (enter_shoot) {
        const float target = positive ? speed_target : -speed_target;
        fw_speed_pid_[i]->Update(target, fw_motors_[i]->rpm());
        fw_motors_[i]->SetCurrent(fw_speed_pid_[i]->out());
        // fw_motors_[i]->SetCurrent(0);
      } else {
        if (std::abs(fw_motors_[i]->rpm()) >= kFwBrakeThresholdRpm) {
          const float brake_target = positive ? kFwBrakeTargetRpm : -kFwBrakeTargetRpm;
          fw_speed_pid_[i]->Update(brake_target, fw_motors_[i]->rpm());
          fw_motors_[i]->SetCurrent(fw_speed_pid_[i]->out());
          // fw_motors_[i]->SetCurrent(0/*);
        } else {
          fw_speed_pid_[i]->Clear();
          fw_motors_[i]->SetCurrent(0);
        }
      }
    }
    if (enter_shoot) {
      DetectShot();
    }
    rm::device::DjiMotorBase::SendCommand(*can_);
#else
    if (enter_shoot) {
      fric_left_pid_->Update(fric_speed_target_rpm_, fric_left_->rpm());
      fric_left_->SetCurrent(fric_left_pid_->out());
      fric_right_pid_->Update(-fric_speed_target_rpm_, fric_right_->rpm());
      fric_right_->SetCurrent(fric_right_pid_->out());
      DetectShot();
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

  void AdjustSpeed(float delta) {
    fric_speed_target_rpm_ += delta;
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
    fric_speed_target_rpm_456_ += delta;
#endif
  }

#if WHEEL_LEGGED_ROBOT_VARIANT == 1
  void SetLongDistanceMode(bool lob) {
    if (lob) {
      fric_speed_target_rpm_ = kFwLobTargetSpeedRpm_123;
      fric_speed_target_rpm_456_ = kFwLobTargetSpeedRpm_456;
    } else {
      fric_speed_target_rpm_ = kFwNormalTargetSpeedRpm_123;
      fric_speed_target_rpm_456_ = kFwNormalTargetSpeedRpm_456;
    }
  }
#endif

  bool PopShotDetected() {
    bool v = shot_this_cycle_;
    shot_this_cycle_ = false;
    return v;
  }
  uint32_t shot_count() const { return shot_count_; }

  float fric_speed_target_rpm() const { return fric_speed_target_rpm_; }

#if WHEEL_LEGGED_ROBOT_VARIANT == 1
  rm::i16 fw_rpm(int index) const {
    if (index >= 0 && index < kFrictionWheelCount && fw_motors_[index]) return fw_motors_[index]->rpm();
    return 0;
  }
#else
  int16_t fric_left_rpm() const { return fric_left_ ? fric_left_->rpm() : 0; }
  int16_t fric_right_rpm() const { return fric_right_ ? fric_right_->rpm() : 0; }
#endif

#if WHEEL_LEGGED_ROBOT_VARIANT == 1
  void DetectShot() {
    const float target_abs = std::fabs(fric_speed_target_rpm_456_);
    const float rpm_abs_4 = std::fabs(fw_motors_[3]->rpm());
    const float rpm_abs_5 = std::fabs(fw_motors_[4]->rpm());
    const float rpm_abs_6 = std::fabs(fw_motors_[5]->rpm());

    if (target_abs > 0.0f && rpm_abs_4 >= target_abs - kShotReadyThresholdRpm &&
        rpm_abs_5 >= target_abs - kShotReadyThresholdRpm && rpm_abs_6 >= target_abs - kShotReadyThresholdRpm) {
      fric_ready_ = true;
    }

    if (fric_ready_ &&
        (target_abs - rpm_abs_4 > kShotDropThresholdRpm || target_abs - rpm_abs_5 > kShotDropThresholdRpm ||
         target_abs - rpm_abs_6 > kShotDropThresholdRpm)) {
      ++shot_count_;
      fric_ready_ = false;
      shot_this_cycle_ = true;
    }
  }
#else
  void DetectShot() {
    const float target_abs = std::fabs(fric_speed_target_rpm_);
    const float left_abs = std::fabs(fric_left_->rpm());
    const float right_abs = std::fabs(fric_right_->rpm());

    if (target_abs > 0.0f && left_abs >= target_abs - kShotReadyThresholdRpm &&
        right_abs >= target_abs - kShotReadyThresholdRpm) {
      fric_ready_ = true;
    }

    if (fric_ready_ &&
        (target_abs - left_abs > kShotDropThresholdRpm || target_abs - right_abs > kShotDropThresholdRpm)) {
      ++shot_count_;
      fric_ready_ = false;
      shot_this_cycle_ = true;
    }
  }
#endif

 private:
  rm::hal::CanInterface* can_{nullptr};
  float fric_speed_target_rpm_{0.0f};
  bool fric_ready_{false};
  uint32_t shot_count_{0};
  bool shot_this_cycle_{false};
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
  float fric_speed_target_rpm_456_{0.0f};
  std::optional<rm::device::M3508> fw_motors_[kFrictionWheelCount];
  std::optional<rm::modules::PID> fw_speed_pid_[kFrictionWheelCount];
#else
  std::optional<rm::device::M3508> fric_left_;
  std::optional<rm::device::M3508> fric_right_;
  std::optional<rm::modules::PID> fric_left_pid_;
  std::optional<rm::modules::PID> fric_right_pid_;
#endif
};
