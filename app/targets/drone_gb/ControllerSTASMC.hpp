#pragma once

#include <cmath>

/**
 * @brief 二轴云台 级联双环 超螺旋滑模控制器 (位置外环 + 速度内环 STA + 动力学前馈)
 */
class GimbalSTASMC {
 public:
  GimbalSTASMC() = default;

  void Update(float yaw_position, float yaw_speed, float pitch_position, float pitch_speed, float dt = 0.001f) {
    // 1. 安全检查，防止 dt 异常导致除零或积分爆炸
    if (dt <= 1e-6f) return;

    state_.yaw_position = yaw_position;
    state_.yaw_speed = yaw_speed;
    state_.pitch_position = pitch_position;
    state_.pitch_speed = pitch_speed;

    if (!enabled_) {
      output_.yaw = 0.f;
      output_.pitch = 0.f;
      return;
    }

    // ================== 外环：位置环 (P 控制) ==================
    // 计算位置误差
    float yaw_pos_err = target_.yaw_position - state_.yaw_position;
    yaw_pos_err = rm::modules::Wrap(yaw_pos_err + M_PI, 0, 2 * M_PI) - M_PI;

    float pitch_pos_err = target_.pitch_position - state_.pitch_position;

    // 输出目标速度 = 纯 P 控制 + 外部给定的前馈速度
    float yaw_target_speed = params_.yaw_pos.kp * yaw_pos_err + target_.yaw_speed_ff;
    float pitch_target_speed = params_.pitch_pos.kp * pitch_pos_err + target_.pitch_speed_ff;

    // 目标速度限幅
    yaw_target_speed = clamp(yaw_target_speed, -params_.yaw_pos.max_speed, params_.yaw_pos.max_speed);
    pitch_target_speed = clamp(pitch_target_speed, -params_.pitch_pos.max_speed, params_.pitch_pos.max_speed);

    if (just_enabled_) {
      last_.yaw_target_speed = yaw_target_speed;
      last_.pitch_target_speed = pitch_target_speed;
      just_enabled_ = false;
    }

    // ================== 动力学前馈估算 (内部计算加速度并滤波) ==================
    // 1. 差分计算原始目标加速度
    float yaw_raw_accel = (yaw_target_speed - last_.yaw_target_speed) / dt;
    float pitch_raw_accel = (pitch_target_speed - last_.pitch_target_speed) / dt;

    last_.yaw_target_speed = yaw_target_speed;
    last_.pitch_target_speed = pitch_target_speed;

    // 2. 加速度低通滤波 (极其关键，防止��度阶跃产生无限大的加速度毛刺)
    last_.yaw_filtered_accel = params_.yaw_model.accel_alpha * yaw_raw_accel +
                               (1.0f - params_.yaw_model.accel_alpha) * last_.yaw_filtered_accel;
    last_.pitch_filtered_accel = params_.pitch_model.accel_alpha * pitch_raw_accel +
                                 (1.0f - params_.pitch_model.accel_alpha) * last_.pitch_filtered_accel;

    // 3. 加速度安全限幅
    float yaw_accel = clamp(last_.yaw_filtered_accel, -params_.yaw_model.max_accel, params_.yaw_model.max_accel);
    float pitch_accel =
        clamp(last_.pitch_filtered_accel, -params_.pitch_model.max_accel, params_.pitch_model.max_accel);

    // 4. 计算基于物理模型的前馈输出 (J*a + B*v + 外部扭矩前馈)
    float yaw_model_ff =
        params_.yaw_model.J * yaw_accel + params_.yaw_model.B * yaw_target_speed + target_.yaw_torque_ff;
    float pitch_model_ff =
        params_.pitch_model.J * pitch_accel + params_.pitch_model.B * pitch_target_speed + target_.pitch_torque_ff;

    // ================== 内环：速度环 (超螺旋 STA 滑模) ==================
    // 滑模面 s 就是速度误差 (因为外环已经充当了 lambda 的作用)
    float yaw_s = yaw_target_speed - state_.yaw_speed;
    float pitch_s = pitch_target_speed - state_.pitch_speed;

    // 滑模面低通滤波（可选，滤除速度评估带来的高频噪声）
    yaw_s = params_.yaw_spd.s_filter_alpha * yaw_s + (1.0f - params_.yaw_spd.s_filter_alpha) * last_.yaw_s_prev;
    pitch_s =
        params_.pitch_spd.s_filter_alpha * pitch_s + (1.0f - params_.pitch_spd.s_filter_alpha) * last_.pitch_s_prev;
    last_.yaw_s_prev = yaw_s;
    last_.pitch_s_prev = pitch_s;

    // 计算 sat(s) 饱和函数，引入边界层 phi 减小穿轴抖振
    const float yaw_sat = sat(yaw_s, params_.yaw_spd.phi);
    const float pitch_sat = sat(pitch_s, params_.pitch_spd.phi);

    // 计算积分漏斗 (与物理时间 dt 解耦，Leak rate 表示每秒泄放比例)
    const float yaw_leak_factor = std::fmax(0.0f, 1.0f - params_.yaw_spd.leak_rate * dt);
    const float pitch_leak_factor = std::fmax(0.0f, 1.0f - params_.pitch_spd.leak_rate * dt);

    // 超螺旋积分项更新 (直接将 k2 乘在积分前，使得 i_limit 具有明确的物理限制意义)
    yaw_i_ += params_.yaw_spd.k2 * yaw_sat * dt;
    pitch_i_ += params_.pitch_spd.k2 * pitch_sat * dt;

    // 积分泄放
    yaw_i_ *= yaw_leak_factor;
    pitch_i_ *= pitch_leak_factor;

    // 积分限幅
    yaw_i_ = clamp(yaw_i_, -params_.yaw_spd.i_limit, params_.yaw_spd.i_limit);
    pitch_i_ = clamp(pitch_i_, -params_.pitch_spd.i_limit, params_.pitch_spd.i_limit);

    // 超螺旋控制律总输出 = 线性比例 + 超螺旋非线性项 + 积分项 + 动力学前馈
    output_.yaw =
        params_.yaw_spd.kp * yaw_s + params_.yaw_spd.k1 * std::sqrt(std::fabs(yaw_s)) * yaw_sat + yaw_i_ + yaw_model_ff;

    output_.pitch = params_.pitch_spd.kp * pitch_s + params_.pitch_spd.k1 * std::sqrt(std::fabs(pitch_s)) * pitch_sat +
                    pitch_i_ + pitch_model_ff;

    // 最终输出限幅
    output_.yaw = clamp(output_.yaw, -params_.yaw_spd.out_limit, params_.yaw_spd.out_limit);
    output_.pitch = clamp(output_.pitch, -params_.pitch_spd.out_limit, params_.pitch_spd.out_limit);
  }

  void SetTarget(float yaw_position, float pitch_position, float yaw_speed_feedforward = 0.f,
                 float pitch_speed_feedforward = 0.f, float yaw_torque_feedforward = 0.f,
                 float pitch_torque_feedforward = 0.f) {
    target_.yaw_position = yaw_position;
    target_.pitch_position = pitch_position;
    target_.yaw_speed_ff = yaw_speed_feedforward;
    target_.pitch_speed_ff = pitch_speed_feedforward;
    target_.yaw_torque_ff = yaw_torque_feedforward;
    target_.pitch_torque_ff = pitch_torque_feedforward;
  }

  void Enable(bool enable) {
    if (enable && !enabled_) {
      // 刚开启时重置历史状态，防止突变(Derivative Kick)
      last_ = {};
      yaw_i_ = 0.f;
      pitch_i_ = 0.f;
      just_enabled_ = true;  // 【新增】记录刚刚启动
    }
    enabled_ = enable;
  }

  auto &params() { return params_; }
  auto &state() { return state_; }
  auto &target() { return target_; }
  auto &output() { return output_; }

 private:
  struct PosParams {
    float kp{0.0f};          // 位置环P参数
    float max_speed{20.0f};  // 最大输出速度限制

    float kp_real{0.0f};          // 位置环P参数
    float max_speed_real{20.0f};  // 最大输出速度限制
  };

  struct SpeedParams {
    float kp{0.0f};              // 速度误差线性增益
    float k1{0.0f};              // 超螺旋强度1 (sqrt(|s|)项)
    float k2{0.0f};              // 超螺旋强度2 (积分项增益)
    float phi{0.0f};             // 边界层宽度
    float i_limit{5.0f};         // 积分限幅
    float out_limit{10.0f};      // 最终输出限幅
    float s_filter_alpha{1.0f};  // 滑模面低通滤波系数
    float leak_rate{0.5f};       // 积分泄放率

    float kp_real{0.0f};              // 速度误差线性增益
    float k1_real{0.0f};              // 超螺旋强度1 (sqrt(|s|)项)
    float k2_real{0.0f};              // 超螺旋强度2 (积分项增益)
    float phi_real{0.0f};             // 边界层宽度
    float i_limit_real{5.0f};         // 积分限幅
    float out_limit_real{10.0f};      // 最终输出限幅
    float s_filter_alpha_real{1.0f};  // 滑模面低通滤波系数
    float leak_rate_real{0.5f};       // 积分泄放率
  };

  // 【新增】动力学前馈模型参数
  struct ModelParams {
    float J{0.0f};             // 转动惯量系数
    float B{0.0f};             // 粘性摩擦/阻尼系数
    float accel_alpha{0.1f};   // 目标加速度低通滤波系数 (0~1)
    float max_accel{1000.0f};  // 允许的最大内部参考加速度

    float J_real{0.0f};             // 转动惯量系数
    float B_real{0.0f};             // 粘性摩擦/阻尼系数
    float accel_alpha_real{0.1f};   // 目标加速度低通滤波系数 (0~1)
    float max_accel_real{1000.0f};  // 允许的最大内部参考加速度
  };

  static float sat(float s, float phi) {
    if (phi <= 0.f) return (s > 0.f) ? 1.f : ((s < 0.f) ? -1.f : 0.f);
    if (s > phi) return 1.f;
    if (s < -phi) return -1.f;
    return s / phi;
  }

  static float clamp(float x, float min_v, float max_v) {
    if (x > max_v) return max_v;
    if (x < min_v) return min_v;
    return x;
  }

 private:
  bool enabled_{false};
  bool just_enabled_{false};  // 启动瞬间防冲击标志位

  struct {
    PosParams yaw_pos;
    SpeedParams yaw_spd;
    ModelParams yaw_model;  // 【新增】

    PosParams pitch_pos;
    SpeedParams pitch_spd;
    ModelParams pitch_model;  // 【新增】
  } params_;

  struct {
    float yaw_position;
    float yaw_speed;
    float pitch_position;
    float pitch_speed;
  } state_{};

  struct {
    float yaw_position;
    float pitch_position;
    float yaw_speed_ff;
    float pitch_speed_ff;
    float yaw_torque_ff;
    float pitch_torque_ff;
  } target_{};

  struct {
    float yaw_s_prev = 0.f;
    float pitch_s_prev = 0.f;

    // 【新增】记录上一帧的目标速度，用于差分算加速度
    float yaw_target_speed = 0.f;
    float pitch_target_speed = 0.f;

    // 【新增】记录滤波后的加速度，用于迭代平滑
    float yaw_filtered_accel = 0.f;
    float pitch_filtered_accel = 0.f;
  } last_{};

  float yaw_i_{0.f};
  float pitch_i_{0.f};

  struct {
    float yaw;
    float pitch;
  } output_{};
};