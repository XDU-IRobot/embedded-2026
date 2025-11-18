#pragma once

#include <librm.hpp>

/**
 * @brief 双摩擦轮发射机构控制器
 */
class Shoot2Fric {
 public:
  explicit Shoot2Fric(int bullets_per_drum, float reduction_ratio)
      : bullets_per_drum_(bullets_per_drum), loader_reduction_ratio_(reduction_ratio) {
    pid_.loader_position.SetCircular(true).SetCircularCycle(2.0f * M_PI);
  }

  /**
   * @brief 更新状态
   */
  void Update(float fric_1_speed, float fric_2_speed, float loader_position, float loader_speed, float dt = 1.0f) {
    state_.fric_1_speed = fric_1_speed;
    state_.fric_2_speed = fric_2_speed;
    state_.loader_speed = loader_speed;
    state_.loader_position = loader_position;

    if (loader_position >= target_.loader_position - 2000.0f) {
      single_shoot_complete_ = true;
    }

    if (!enabled_) {
      output_.fric_1 = 0.0f;
      output_.fric_2 = 0.0f;
      output_.loader = 0.0f;
      return;
    }

    // 摩擦轮控制
    if (!armed_) {
      target_.fric_speed = 0.0f;
    }

    pid_.fric_1_speed.Update(target_.fric_speed, state_.fric_1_speed, dt);
    output_.fric_1 = pid_.fric_1_speed.out();

    pid_.fric_2_speed.Update(-target_.fric_speed, state_.fric_2_speed, dt);
    output_.fric_2 = pid_.fric_2_speed.out();

    // 拨盘控制
    if (!single_shoot_complete_) {
      // 单发模式：位置 - 速度串级
      pid_.loader_position.Update(target_.loader_position, state_.loader_position, dt);
      pid_.loader_speed.Update(-7000.0f, state_.loader_speed, dt);
      output_.loader = pid_.loader_speed.out();
    } else {
      // 连发模式：速度环
      target_.loader_position = state_.loader_position;
      pid_.loader_speed.Update(target_.loader_speed, state_.loader_speed, dt);
      output_.loader = pid_.loader_speed.out();
    }
  }

  /**
   * @brief 开火逻辑
   */
  void Fire() {
    if (!armed_) return;

    if (mode_ == kSingleShot && single_shoot_complete_) {
      // 拨盘加一个子弹间隔
      target_.loader_position =
          state_.loader_position + loader_reduction_ratio_ / static_cast<float>(bullets_per_drum_) * 8191.f;
      state_.loader_circle_num_ = 0;
      single_shoot_complete_ = false;

    } else if (mode_ == kFullAuto) {
      target_.loader_speed = calculated_target_loader_speed_ * loader_reduction_ratio_;
    } else {
      target_.loader_speed = 0.0f;
    }
  }

  enum Mode {
    kStop,
    kFullAuto,
    kSingleShot,
  };

  void SetMode(Mode mode) { mode_ = mode; }

  /// 设置射频（发/秒）
  void SetShootFrequency(float frequency) {
    calculated_target_loader_speed_ = frequency / static_cast<float>(bullets_per_drum_) * 60.0f;  // rpm
  }

  /// 设置摩擦轮线速度（rad/s）
  void SetArmSpeed(float fric_speed) { target_.fric_speed = fric_speed; }

  void Arm(bool enable) { armed_ = enable; }

  void Enable(bool enable) { enabled_ = enable; }

  auto &pid() { return pid_; }
  auto &state() { return state_; }
  auto &target() { return target_; }
  auto &output() { return output_; }
  auto &shoot_flag() { return single_shoot_complete_; }

 private:
  bool enabled_{false};
  bool armed_{true};
  bool single_shoot_complete_{true};
  const int bullets_per_drum_;
  const float loader_reduction_ratio_;
  float calculated_target_loader_speed_{0.0f};
  Mode mode_{kFullAuto};

  struct {
    rm::modules::PID fric_1_speed;
    rm::modules::PID fric_2_speed;
    rm::modules::PID loader_position;
    rm::modules::PID loader_speed;
  } pid_;

  struct {
    float fric_1_speed;
    float fric_2_speed;
    float loader_speed;
    float loader_position;
    int loader_circle_num_{0};
  } state_{};

  struct {
    float fric_speed;
    float loader_speed;
    float loader_position;
  } target_{};

  struct {
    float fric_1;
    float fric_2;
    float loader;
  } output_{};
};
