#pragma once

#include <librm.hpp>

/**
 * @brief 二轴云台控制器
 */
class Gimbal2Dof {
 public:
  Gimbal2Dof() {
    pid_.yaw_position
        .SetCircular(true)           //
        .SetCircularCycle(M_PI * 2)  //
        .SetFuzzy(true)              //
        .SetFuzzyErrorScale(M_PI);
    pid_.pitch_position
        .SetFuzzy(true)  //
        .SetFuzzyErrorScale(M_PI);
  }

  /**
   * @brief 更新一步，角度单位均为弧度，速度单位均为弧度每秒
   */
  void Update(float yaw_position, float yaw_speed, float pitch_position, float pitch_speed, float dt = 1.f) {
    state_.yaw_position = yaw_position;
    state_.yaw_speed = yaw_speed;
    state_.pitch_position = pitch_position;
    state_.pitch_speed = pitch_speed;

    if (!enabled_) {  // 无力，控制量设0直接返回
      output_.yaw = 0.f;
      output_.pitch = 0.f;
      return;
    }

    pid_.yaw_position.Update(target_.yaw_position, state_.yaw_position, dt);
    pid_.pitch_position.Update(target_.pitch_position, state_.pitch_position, dt);

    if (speed_pid_enabled_) {  // 速度位置双环
      const float yaw_speed_target = pid_.yaw_position.out() + target_.yaw_speed_ff;
      pid_.yaw_speed.Update(yaw_speed_target, state_.yaw_speed, dt);
      output_.yaw = pid_.yaw_speed.out() + target_.yaw_output_ff;
      const float pitch_speed_target = pid_.pitch_position.out();
      pid_.pitch_speed.Update(pitch_speed_target, state_.pitch_speed, dt);
      output_.pitch = pid_.pitch_speed.out();
    } else {  // 单位置环
      output_.yaw = pid_.yaw_position.out() + target_.yaw_output_ff;
      output_.pitch = pid_.pitch_position.out();
    }
  }

  /**
   * @brief 设置目标位置和前馈量
   */
  void SetTarget(float yaw_position, float pitch_position, float yaw_speed_feedforward = 0.f,
                 float yaw_control_feedforward = 0.f) {
    target_.yaw_position = yaw_position;
    target_.pitch_position = pitch_position;
    target_.yaw_speed_ff = yaw_speed_feedforward;
    target_.yaw_output_ff = yaw_control_feedforward;
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
  bool enabled_{false};           ///< 有力/无力？
  bool speed_pid_enabled_{true};  ///< 单环/双环？
  struct {
    rm::modules::PID yaw_speed, yaw_position, pitch_speed, pitch_position;
  } pid_;
  struct {
    float yaw_position;
    float yaw_speed;
    float pitch_position;
    float pitch_speed;
  } state_{};  ///< 当前状态
  struct {
    float yaw_position;
    float pitch_position;
    float yaw_speed_ff;   ///< Yaw 速度前馈
    float yaw_output_ff;  ///< Yaw 控制量前馈，控制量具体是力矩、电流或者什么，取决于电机驱动
  } target_{};            ///< 目标状态
  struct {
    float yaw;
    float pitch;
  } output_{};  ///< 控制输出
};