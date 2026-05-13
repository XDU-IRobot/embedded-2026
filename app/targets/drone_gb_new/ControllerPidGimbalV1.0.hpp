#pragma once

#include <librm.hpp>

/**
 * @brief 二轴云台控制器
 */
class Gimbal2DofV1 {
 public:
  int time = 0;  // 保留原接口/可见成员，但内部不再依赖它做核心控制

  Gimbal2Dof() {
    pid_.yaw_position
        .SetCircular(true)
        .SetCircularCycle(M_PI * 2)
        .SetFuzzy(true)
        .SetFuzzyErrorScale(M_PI)
        .SetFuzzyDErrorScale(M_PI * 100);

    pid_.pitch_position
        .SetFuzzy(true)
        .SetFuzzyErrorScale(M_PI)
        .SetFuzzyDErrorScale(M_PI * 100);
  }

  /**
   * @brief 更新一步，角度单位均为弧度，速度单位均为弧度每秒
   *
   * @param yaw_position 当前 yaw 角度，rad
   * @param yaw_speed 当前 yaw 角速度，rad/s
   * @param pitch_position 当前 pitch 角度，rad
   * @param pitch_speed 当前 pitch 角速度，rad/s
   * @param dt 当前控制周期，单位 s
   *
   * @note 为了兼容旧接口，dt 默认值仍是 1.f。
   *       但如果调用者不传 dt，本函数会自动按 1000Hz 控制周期处理，即 0.001s。
   */
  void Update(float yaw_position,
              float yaw_speed,
              float pitch_position,
              float pitch_speed,
              float dt = 1.f) {
    state_.yaw_position = yaw_position;
    state_.yaw_speed = yaw_speed;
    state_.pitch_position = pitch_position;
    state_.pitch_speed = pitch_speed;

    const float control_dt = NormalizeDt(dt);

    if (!enabled_) {
      output_.yaw = 0.f;
      output_.pitch = 0.f;

      if (last_enabled_) {
        ClearAllPid();
      }

      last_enabled_ = false;
      return;
    }

    if (!last_enabled_) {
      ClearAllPid();
      outer_dt_accum_ = 0.f;
      time = 0;
      last_enabled_ = true;
    }

    UpdatePositionLoop(control_dt);

    if (speed_pid_enabled_) {
      UpdateCascadeLoop(control_dt);
    } else {
      UpdateSinglePositionLoop();
    }
  }

  /**
   * @brief 设置目标位置和前馈量
   */
  void SetTarget(float yaw_position,
                 float pitch_position,
                 float yaw_speed_feedforward = 0.f,
                 float yaw_control_feedforward = 0.f) {
    target_.yaw_position = yaw_position;
    target_.pitch_position = pitch_position;
    target_.yaw_speed_ff = yaw_speed_feedforward;
    target_.yaw_output_ff = yaw_control_feedforward;
  }

  /**
   * @brief 启用或禁用速度环 PID 控制（切换单位置环或速度位置双环控制）
   */
  void EnableSpeedPid(bool enable) {
    if (speed_pid_enabled_ != enable) {
      pid_.yaw_speed.Clear();
      pid_.pitch_speed.Clear();
    }

    speed_pid_enabled_ = enable;
  }

  /**
   * @brief 启用或禁用控制器（切换有力无力）
   */
  void Enable(bool enable) {
    if (enabled_ != enable) {
      ClearAllPid();
      outer_dt_accum_ = 0.f;
      time = 0;
    }

    enabled_ = enable;
    last_enabled_ = enable;
  }

  // getters
  auto &pid() { return pid_; }
  auto &state() { return state_; }
  auto &target() { return target_; }
  auto &output() { return output_; }

 private:
  static constexpr float kDefaultInnerDt = 0.001f;  // 默认认为 Update 以 1000Hz 调用
  static constexpr float kOuterPeriod = 0.004f;     // 位置环 250Hz
  static constexpr float kMinDt = 0.0001f;          // 10kHz，防止 dt 过小导致 D 项爆炸
  static constexpr float kMaxDt = 0.02f;            // 50Hz，防止异常卡顿导致积分暴涨

  bool enabled_{false};           ///< 有力/无力？
  bool last_enabled_{false};      ///< 上一次是否使能，用于检测使能边沿
  bool speed_pid_enabled_{true};  ///< 单环/双环？

  float outer_dt_accum_{0.f};     ///< 外环累计时间

  struct {
    rm::modules::PID yaw_speed;
    rm::modules::PID yaw_position;
    rm::modules::PID pitch_speed;
    rm::modules::PID pitch_position;
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
    float yaw_speed_ff;    ///< Yaw 速度前馈
    float yaw_output_ff;   ///< Yaw 控制量前馈，控制量具体是力矩、电流或者什么，取决于电机驱动
  } target_{};             ///< 目标状态

  struct {
    float yaw;
    float pitch;
  } output_{};  ///< 控制输出

  static float ClampFloat(float value, float min_value, float max_value) {
    if (value < min_value) {
      return min_value;
    }

    if (value > max_value) {
      return max_value;
    }

    return value;
  }

  /**
   * @brief 归一化 dt
   *
   * 由于旧接口默认 dt = 1.f，这在 1000Hz 控制里非常危险。
   * 因此这里做兼容：
   * - 如果调用者没有传 dt，通常会得到 1.f，此时按默认 0.001s 处理；
   * - 如果调用者显式传入真实 dt，例如 0.001f / 0.002f / 0.004f，则按真实 dt 使用；
   * - 对异常值做限幅，防止积分和微分爆炸。
   */
  static float NormalizeDt(float dt) {
    if (dt <= 0.f) {
      return kDefaultInnerDt;
    }

    // 兼容旧调用：Update(...), 默认 dt = 1.f
    // 在云台 1000Hz 调用场景下，这里应视为 0.001s，而不是 1s。
    if (dt == 1.f) {
      return kDefaultInnerDt;
    }

    return ClampFloat(dt, kMinDt, kMaxDt);
  }

  void UpdatePositionLoop(float dt) {
    outer_dt_accum_ += dt;

    if (outer_dt_accum_ < kOuterPeriod) {
      return;
    }

    // 使用真实累计时间作为外环 dt。
    // 如果调度有轻微抖动，比如 0.0039 / 0.0041，也不会破坏 PID 的物理意义。
    const float position_dt = outer_dt_accum_;
    outer_dt_accum_ = 0.f;

    pid_.yaw_position.Update(
        target_.yaw_position,
        state_.yaw_position,
        position_dt);

    pid_.pitch_position.Update(
        target_.pitch_position,
        state_.pitch_position,
        position_dt);

    // 保留 time 的可见性：这里让它表示外环刚刚更新过。
    time = 0;
  }

  void UpdateCascadeLoop(float dt) {
    const float yaw_speed_target =
        pid_.yaw_position.out() + target_.yaw_speed_ff;

    pid_.yaw_speed.Update(
        yaw_speed_target,
        state_.yaw_speed,
        dt);

    output_.yaw =
        pid_.yaw_speed.out() + target_.yaw_output_ff;

    const float pitch_speed_target =
        pid_.pitch_position.out();

    pid_.pitch_speed.Update(
        pitch_speed_target,
        state_.pitch_speed,
        dt);

    output_.pitch =
        pid_.pitch_speed.out();
  }

  void UpdateSinglePositionLoop() {
    output_.yaw =
        pid_.yaw_position.out() + target_.yaw_output_ff;

    output_.pitch =
        pid_.pitch_position.out();
  }

  void ClearAllPid() {
    pid_.yaw_speed.Clear();
    pid_.pitch_speed.Clear();
    pid_.yaw_position.Clear();
    pid_.pitch_position.Clear();

    output_.yaw = 0.f;
    output_.pitch = 0.f;

    outer_dt_accum_ = 0.f;
  }
};
