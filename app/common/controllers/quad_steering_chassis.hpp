#pragma once

#include <librm.hpp>

/**
 * @brief 四舵轮底盘控制器
 */
class QuadSteeringChassis {
 public:
  QuadSteeringChassis(float wheel_radius, float chassis_radius) : kinematics_(chassis_radius) {
    pid_.lf_steer_position.SetCircular(true).SetCircularCycle(M_PI * 2.0f);
    pid_.rf_steer_position.SetCircular(true).SetCircularCycle(M_PI * 2.0f);
    pid_.lb_steer_position.SetCircular(true).SetCircularCycle(M_PI * 2.0f);
    pid_.rb_steer_position.SetCircular(true).SetCircularCycle(M_PI * 2.0f);
  }

  /**
   * @brief 更新一步，角度单位均为弧度，速度单位均为弧度每秒
   */
  void Update(float lf_steer_position, float lf_steer_speed, float rf_steer_position, float rf_steer_speed,
              float lb_steer_position, float lb_steer_speed, float rb_steer_position, float rb_steer_speed,
              float lf_wheel_speed, float rf_wheel_speed, float lb_wheel_speed, float rb_wheel_speed, float dt = 1.f) {
    // 更新当前状态
    state_.lf_steer_position = lf_steer_position;
    state_.rf_steer_position = rf_steer_position;
    state_.lb_steer_position = lb_steer_position;
    state_.rb_steer_position = rb_steer_position;
    state_.lf_steer_speed = lf_steer_speed;
    state_.rf_steer_speed = rf_steer_speed;
    state_.lb_steer_speed = lb_steer_speed;
    state_.rb_steer_speed = rb_steer_speed;
    state_.lf_wheel_speed = lf_wheel_speed;
    state_.rf_wheel_speed = rf_wheel_speed;
    state_.lb_wheel_speed = lb_wheel_speed;
    state_.rb_wheel_speed = rb_wheel_speed;

    if (!enabled_) {
      output_.lf_wheel = 0.f;
      output_.rf_wheel = 0.f;
      output_.lb_wheel = 0.f;
      output_.rb_wheel = 0.f;
      output_.lf_steer = 0.f;
      output_.rf_steer = 0.f;
      output_.lb_steer = 0.f;
      output_.rb_steer = 0.f;
      return;
    }

    // 运动学解算
    const auto fk_result =
        kinematics_.Forward(target_.vx, target_.vy, target_.w, state_.lf_steer_position, state_.rf_steer_position,
                            state_.lb_steer_position, state_.rb_steer_position);

    // 舵控制
    pid_.lf_steer_position.Update(static_cast<float>(fk_result.lf_steer_position), state_.lf_steer_position, dt);
    pid_.rf_steer_position.Update(static_cast<float>(fk_result.rf_steer_position), state_.rf_steer_position, dt);
    pid_.lb_steer_position.Update(static_cast<float>(fk_result.lr_steer_position), state_.lb_steer_position, dt);
    pid_.rb_steer_position.Update(static_cast<float>(fk_result.rr_steer_position), state_.rb_steer_position, dt);

    if (speed_pid_enabled_) {
      const float lf_steer_target_speed = pid_.lf_steer_position.out();
      pid_.lf_steer_speed.Update(lf_steer_target_speed, state_.lf_steer_speed, dt);
      output_.lf_steer = pid_.lf_steer_speed.out();
    } else {
      output_.lf_steer = pid_.lf_steer_position.out();
    }
    if (speed_pid_enabled_) {
      const float rf_steer_target_speed = pid_.rf_steer_position.out();
      pid_.rf_steer_speed.Update(rf_steer_target_speed, state_.rf_steer_speed, dt);
      output_.rf_steer = pid_.rf_steer_speed.out();
    } else {
      output_.rf_steer = pid_.rf_steer_position.out();
    }
    if (speed_pid_enabled_) {
      const float lb_steer_target_speed = pid_.lb_steer_position.out();
      pid_.lb_steer_speed.Update(lb_steer_target_speed, state_.lb_steer_speed, dt);
      output_.lb_steer = pid_.lb_steer_speed.out();
    } else {
      output_.lb_steer = pid_.lb_steer_position.out();
    }
    // 右后
    if (speed_pid_enabled_) {
      const float rb_steer_target_speed = pid_.rb_steer_position.out();
      pid_.rb_steer_speed.Update(rb_steer_target_speed, state_.rb_steer_speed, dt);
      output_.rb_steer = pid_.rb_steer_speed.out();
    } else {
      output_.rb_steer = pid_.rb_steer_position.out();
    }

    // 轮速控制
    pid_.lf_wheel.Update(static_cast<float>(-fk_result.lf_wheel_speed), state_.lf_wheel_speed, dt);
    output_.lf_wheel = pid_.lf_wheel.out();
    pid_.rf_wheel.Update(static_cast<float>(-fk_result.rf_wheel_speed), state_.rf_wheel_speed, dt);
    output_.rf_wheel = pid_.rf_wheel.out();
    pid_.lb_wheel.Update(static_cast<float>(-fk_result.lr_wheel_speed), state_.lb_wheel_speed, dt);
    output_.lb_wheel = pid_.lb_wheel.out();
    pid_.rb_wheel.Update(static_cast<float>(-fk_result.rr_wheel_speed), state_.rb_wheel_speed, dt);
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
   * @brief 启用或禁用速度环 PID 控制（切换单位置环或速度位置双环控制）
   */
  void EnableSpeedPid(bool enable) { speed_pid_enabled_ = enable; }

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
  bool speed_pid_enabled_{true};             ///< 单环/双环？
  rm::modules::SteeringChassis kinematics_;  ///< 底盘运动学模型
  struct {
    rm::modules::PID lf_wheel;                           ///< 左前轮速度环
    rm::modules::PID rf_wheel;                           ///< 右前轮速度环
    rm::modules::PID lb_wheel;                           ///< 左后轮速度环
    rm::modules::PID rb_wheel;                           ///< 右后轮速度环
    rm::modules::PID lf_steer_speed, lf_steer_position;  ///< 左前舵速度环和位置环
    rm::modules::PID rf_steer_speed, rf_steer_position;  ///< 右前舵速度环和位置环
    rm::modules::PID lb_steer_speed, lb_steer_position;  ///< 左后舵速度环和位置环
    rm::modules::PID rb_steer_speed, rb_steer_position;  ///< 右后舵速度环和位置环
  } pid_;

  struct {
    float lf_wheel_speed;     ///< 左前轮速度
    float rf_wheel_speed;     ///< 右前轮速度
    float lb_wheel_speed;     ///< 左后轮速度
    float rb_wheel_speed;     ///< 右后轮速度
    float lf_steer_position;  ///< 左前舵位置
    float rf_steer_position;  ///< 右前舵位置
    float lb_steer_position;  ///< 左后舵位置
    float rb_steer_position;  ///< 右后舵位置
    float lf_steer_speed;     ///< 左前舵速度
    float rf_steer_speed;     ///< 右前舵速度
    float lb_steer_speed;     ///< 左后舵速度
    float rb_steer_speed;     ///< 右后舵速度
  } state_{};                 ///< 当前状态
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
    float lf_steer;  ///< 左前舵控制输出
    float rf_steer;  ///< 右前舵控制输出
    float lb_steer;  ///< 左后舵控制输出
    float rb_steer;  ///< 右后舵控制输出
  } output_{};       ///< 控制输出
};
