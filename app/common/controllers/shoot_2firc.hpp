#pragma once

#include <librm.hpp>

/**
 * @brief 简化版双摩擦轮发射机构控制器（仅速度控制）
 *
 * 接口说明：
 * - Update(fric_1_speed, fric_2_speed, loader_speed, dt)
 *   其中速度单位均为 rpm（与 dial_motor->rpm() / friction->rpm() 保持一致）
 * - SetLoaderSpeed(loader_rpm) 直接设置目标转速（rpm）
 */
class Shoot2Fric {
 public:
  Shoot2Fric() = default;

  void Update(float fric_1_speed, float fric_2_speed, float loader_speed, float dt = 1.0f) {
    state_.fric_1_speed = fric_1_speed;
    state_.fric_2_speed = fric_2_speed;
    state_.loader_speed = loader_speed;

    if (!enabled_) {
      output_.fric_1 = 0.0f;
      output_.fric_2 = 0.0f;
      output_.loader = 0.0f;
      return;
    }

    // 摩擦轮
    if (!armed_) {
      target_.fric_speed = 0.0f;
    }
    pid_.fric_1_speed.Update(target_.fric_speed, state_.fric_1_speed, dt);
    output_.fric_1 = pid_.fric_1_speed.out();

    pid_.fric_2_speed.Update(-target_.fric_speed, state_.fric_2_speed, dt);
    output_.fric_2 = pid_.fric_2_speed.out();

    // 拨盘：速度环（单位 rpm）
    pid_.loader_speed.Update(target_.loader_speed, state_.loader_speed, dt);
    output_.loader = pid_.loader_speed.out();
  }

  enum Mode { kStop, kFullAuto };

  void SetMode(Mode mode) { mode_ = mode; }

  void SetLoaderSpeed(float loader_rpm) { target_.loader_speed = loader_rpm; }

  void SetArmSpeed(float fric_speed) { target_.fric_speed = fric_speed; }

  float GetLoaderSpeed() const { return target_.loader_speed; }

  float GetArmSpeed() const { return target_.fric_speed; }

  void Arm(bool enable) { armed_ = enable; }

  void Enable(bool enable) { enabled_ = enable; }

  auto &pid() { return pid_; }
  auto &state() { return state_; }
  auto &target() { return target_; }
  auto &output() { return output_; }

 private:
  bool enabled_{false};
  bool armed_{true};
  Mode mode_{kFullAuto};

  struct {
    rm::modules::PID fric_1_speed;
    rm::modules::PID fric_2_speed;
    rm::modules::PID loader_speed;
  } pid_;

  struct {
    float fric_1_speed{0.f};
    float fric_2_speed{0.f};
    float loader_speed{0.f};  // rpm
  } state_{};

  struct {
    float fric_speed{0.f};
    float loader_speed{0.f};  // rpm
  } target_{};

  struct {
    float fric_1{0.f};
    float fric_2{0.f};
    float loader{0.f};
  } output_{};
};