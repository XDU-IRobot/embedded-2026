#pragma once

#include <librm.hpp>

/**
 * @brief 二轴云台控制器（支持位置-速度-电流三环，并可单独控制Yaw电流环）
 */
class Gimbal2Dof {
 public:
  Gimbal2Dof() {
    // 位置环参数
    pid_.yaw_position
        .SetCircular(true)
        .SetCircularCycle(M_PI * 2)
        .SetFuzzy(true)
        .SetFuzzyErrorScale(M_PI)
        .SetFuzzyDErrorScale(M_PI * 100);
    pid_.pitch_position
        .SetFuzzy(true)
        .SetFuzzyErrorScale(M_PI);
    // 速度环与电流环可在此按需初始化
  }

  /**
   * @brief 更新一步控制
   * @param yaw_position  当前yaw角度 (rad)
   * @param yaw_speed     当前yaw角速度 (rad/s)
   * @param yaw_current   当前yaw电流 (A或实际单位)
   * @param pitch_position当前pitch角度 (rad)
   * @param pitch_speed   当前pitch角速度 (rad/s)
   * @param pitch_current 当前pitch电流 (保留参数，Pitch轴未使用)
   * @param dt            控制周期，调用者应传入实际值
   */
  void Update(float yaw_position, float yaw_speed, float yaw_current,
              float pitch_position, float pitch_speed, float pitch_current,
              float dt = 1.f) {
    state_.yaw_position = yaw_position;
    state_.yaw_speed = yaw_speed;
    state_.yaw_current = yaw_current;
    state_.pitch_position = pitch_position;
    state_.pitch_speed = pitch_speed;
    state_.pitch_current = pitch_current;  // 暂未使用

    if (!enabled_) {
      output_.yaw = 0.f;
      output_.pitch = 0.f;
      return;
    }

    // 位置环（两轴通用）
    pid_.yaw_position.Update(target_.yaw_position, state_.yaw_position, dt);
    pid_.pitch_position.Update(target_.pitch_position, state_.pitch_position, dt);

    if (speed_pid_enabled_) {
      // ================== Yaw 轴 ==================
      const float yaw_speed_target = pid_.yaw_position.out() + target_.yaw_speed_ff;
      pid_.yaw_speed.Update(yaw_speed_target, state_.yaw_speed, dt);

      if (yaw_current_pid_enabled_) {          // 三环模式
        const float yaw_current_target = pid_.yaw_speed.out() + target_.yaw_current_ff;
        pid_.yaw_current.Update(yaw_current_target, state_.yaw_current, dt);
        output_.yaw = pid_.yaw_current.out();
      } else {                                 // 仅位置-速度双环（与旧版兼容）
        output_.yaw = pid_.yaw_speed.out();
      }

      // ================== Pitch 轴（保持位置-速度双环，不动）==================
      const float pitch_speed_target = pid_.pitch_position.out();
      pid_.pitch_speed.Update(pitch_speed_target, state_.pitch_speed, dt);
      output_.pitch = pid_.pitch_speed.out();

    } else {  // 仅位置环（两轴行为与旧版完全一致）
      output_.yaw = pid_.yaw_position.out() + target_.yaw_current_ff;
      output_.pitch = pid_.pitch_position.out() + target_.pitch_current_ff;
    }
  }

  /**
   * @brief 设置目标位置和前馈量
   * @param yaw_position         目标yaw角度
   * @param pitch_position       目标pitch角度
   * @param yaw_speed_feedforward  yaw速度前馈（加在速度目标上）
   * @param yaw_current_feedforward yaw电流前馈（三环时加在电流目标，单环时加在输出）
   * @param pitch_current_feedforward pitch电流前馈（单环时加在输出）
   */
  void SetTarget(float yaw_position, float pitch_position,
                 float yaw_speed_feedforward = 0.f,
                 float yaw_current_feedforward = 0.f,
                 float pitch_current_feedforward = 0.f) {
    target_.yaw_position = yaw_position;
    target_.pitch_position = pitch_position;
    target_.yaw_speed_ff = yaw_speed_feedforward;
    target_.yaw_current_ff = yaw_current_feedforward;
    target_.pitch_current_ff = pitch_current_feedforward;
  }

  /**
   * @brief 启用或禁用速度环（两轴同时，与旧版一致）
   */
  void EnableSpeedPid(bool enable) { speed_pid_enabled_ = enable; }

  /**
   * @brief 单独启用或禁用Yaw轴的电流环（即是否在速度环后再串电流环）
   * @note 仅在 speed_pid_enabled_ == true 时有效，Pitch轴不受影响
   */
  void EnableYawCurrentPid(bool enable) { yaw_current_pid_enabled_ = enable; }

  /**
   * @brief 启用或禁用控制器（有力/无力）
   */
  void Enable(bool enable) { enabled_ = enable; }

  // getters
  auto &pid() { return pid_; }
  auto &state() { return state_; }
  auto &target() { return target_; }
  auto &output() { return output_; }

 private:
  bool enabled_{false};                   ///< 有力/无力
  bool speed_pid_enabled_{true};          ///< 位置环以外的速度环总开关（两轴）
  bool yaw_current_pid_enabled_{false};   ///< 是否开启Yaw电流环（默认关闭，保持双环兼容）

  struct {
    rm::modules::PID yaw_speed, yaw_position, pitch_speed, pitch_position;
    rm::modules::PID yaw_current, pitch_current;   ///< 电流环（pitch保留但未使用）
  } pid_;

  struct {
    float yaw_position;
    float yaw_speed;
    float yaw_current;          ///< 当前yaw电流（实测值）
    float pitch_position;
    float pitch_speed;
    float pitch_current;        ///< 当前pitch电流（保留，未参与计算）
  } state_{};

  struct {
    float yaw_position;
    float pitch_position;
    float yaw_speed_ff;         ///< Yaw速度前馈
    float yaw_current_ff;       ///< Yaw电流前馈（三环时加在电流目标，单环时加在输出）
    float pitch_current_ff;     ///< Pitch电流前馈（仅单环模式下有效）
  } target_{};

  struct {
    float yaw;   ///< Yaw控制输出（三环时为电流给定，双环时为速度环输出，单环时含义取决于实际驱动）
    float pitch; ///< Pitch控制输出（保持速度环输出或单环输出）
  } output_{};
};