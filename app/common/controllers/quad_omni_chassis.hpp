#pragma once

#include <librm.hpp>

/**
 * @brief 四全向轮底盘控制器
 */
class QuadOmniChassis {
 public:
  QuadOmniChassis() {
    pid_.lf_wheel.SetDiffFirst(true);
    pid_.rf_wheel.SetDiffFirst(true);
    pid_.lb_wheel.SetDiffFirst(true);
    pid_.rb_wheel.SetDiffFirst(true);
  }

  /**
   * @brief 更新一步，角度单位均为弧度，速度单位均为弧度每秒
   */
  void Update(float lf_wheel_speed, float rf_wheel_speed, float lb_wheel_speed, float rb_wheel_speed, float dt = 1.f) {
    // 更新当前状态
    state_.lf_wheel_speed = lf_wheel_speed;
    state_.rf_wheel_speed = rf_wheel_speed;
    state_.lb_wheel_speed = lb_wheel_speed;
    state_.rb_wheel_speed = rb_wheel_speed;

    if (!enabled_) {
      output_.lf_wheel = 0.f;
      output_.rf_wheel = 0.f;
      output_.lb_wheel = 0.f;
      output_.rb_wheel = 0.f;
      return;
    }

    // 运动学解算
    const auto fk_result = kinematics_.Forward(target_.vx, target_.vy, target_.w);

    // 轮速控制
    pid_.lf_wheel.Update(fk_result.lf_speed, state_.lf_wheel_speed, dt);
    output_.lf_wheel = pid_.lf_wheel.out();
    pid_.rf_wheel.Update(fk_result.rf_speed, state_.rf_wheel_speed, dt);
    output_.rf_wheel = pid_.rf_wheel.out();
    pid_.lb_wheel.Update(fk_result.lr_speed, state_.lb_wheel_speed, dt);
    output_.lb_wheel = pid_.lb_wheel.out();
    pid_.rb_wheel.Update(fk_result.rr_speed, state_.rb_wheel_speed, dt);
    output_.rb_wheel = pid_.rb_wheel.out();
  }

  /**
   * @brief 设置目标线速度和角速度，单位分别为米每秒和弧度每秒
   */
  void SetTarget(float vx, float vy, float w) {
    target_.vx = vx;
    target_.vy = vy;
    target_.w = w;
  }

  /**
   * @brief 启用或禁用控制器（切换有力无力）
   */
  void Enable(bool enable) { enabled_ = enable; }

  // getters
  auto &pid() { return pid_; }
  auto &state() { return state_; }
  auto &target() { return target_; }
  auto &output() { return output_; }

 private:
  bool enabled_{false};                      ///< 有力/无力？
  rm::modules::QuadOmniChassis kinematics_;  ///< 底盘运动学模型
  struct {
    rm::modules::PID lf_wheel;  ///< 左前轮速度环
    rm::modules::PID rf_wheel;  ///< 右前轮速度环
    rm::modules::PID lb_wheel;  ///< 左后轮速度环
    rm::modules::PID rb_wheel;  ///< 右后轮速度环
  } pid_;
  struct {
    float lf_wheel_speed;  ///< 左前轮速度
    float rf_wheel_speed;  ///< 右前轮速度
    float lb_wheel_speed;  ///< 左后轮速度
    float rb_wheel_speed;  ///< 右后轮速度
  } state_{};              ///< 当前状态
  struct {
    float vx;   ///< 目标线速度 x 方向
    float vy;   ///< 目标线速度 y 方向
    float w;    ///< 目标角速度
  } target_{};  ///< 目标状态
  struct {
    float lf_wheel;  ///< 左前轮控制输出
    float rf_wheel;  ///< 右前轮控制输出
    float lb_wheel;  ///< 左后轮控制输出
    float rb_wheel;  ///< 右后轮控制输出
  } output_{};       ///< 控制输出
};