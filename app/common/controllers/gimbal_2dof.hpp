#pragma once

#include "motor_driver.hpp"

#include <librm.hpp>
#include "smc.hpp"

/**
 * @brief 二轴云台控制器
 */
class Gimbal2Dof {
 public:
  const float kGm6020VoltageCmdLimit = 29000.0f;
  const float kDM4310TorqueCmdLimit = 9.8f;
  const float kDegToRad = M_PI / 180.0f;

  Gimbal2Dof() {
    motor_.yaw.SetMode(MotorDriver::Mode::kSMC);
    motor_.pitch.SetMode(MotorDriver::Mode::kSMC);
    motor_.yaw.SetOutputLimits(-kGm6020VoltageCmdLimit, kGm6020VoltageCmdLimit);
    motor_.pitch.SetOutputLimits(-kDM4310TorqueCmdLimit, kDM4310TorqueCmdLimit);
    motor_.pitch.SetPositionLimits(-36 * kDegToRad, 10 * kDegToRad, false);
  };

  void UpdateFilter(float yaw_position, float yaw_speed, float pitch_position, float pitch_speed, float dt) {
    if (!filter_.is_initialized) {
      // filter_.yaw_speed.Reset(yaw_speed);
      // filter_.pitch_speed.Reset(pitch_speed);
      filter_.is_initialized = true;
    }

    // filter_.yaw_speed.Update(yaw_speed);
    // filter_.pitch_speed.Update(pitch_speed);
  }

  /**
   * @brief 更新一步，角度单位均为弧度，速度单位均为弧度每秒
   */
  void Update(float yaw_position, float yaw_speed, float pitch_position, float pitch_speed, float dt = 1.f) {
    state_.yaw_position = yaw_position;
    state_.yaw_speed = yaw_speed;
    state_.pitch_position = pitch_position;
    state_.pitch_speed = pitch_speed;

    // dt = 1.f;
    motor_.yaw.Update(yaw_position, yaw_speed, dt);
    motor_.pitch.Update(pitch_position, pitch_speed, dt);

    output_.yaw = motor_.yaw.output();
    output_.pitch = motor_.pitch.output();
  }

  /**
   * @brief 带有 imu 的更新一步，传入编码器位置做机械限位
   * @note  请让需要限位的轴的 enc 传入为过零处理后的数值（Yaw 需要小陀螺时请勿加机械限位）
   */
  void UpdateWithImu(float yaw_imu_position, float yaw_enc_position, float yaw_speed, float pitch_imu_position,
                     float pitch_enc_position, float pitch_speed, float dt = 1.f) {
    state_.yaw_position = yaw_imu_position;
    state_.yaw_speed = yaw_speed;
    state_.pitch_position = pitch_imu_position;
    state_.pitch_speed = pitch_speed;

    dt = 1.f;
    motor_.yaw.UpdateWithImu(yaw_imu_position, yaw_enc_position, yaw_speed, dt);
    motor_.pitch.UpdateWithImu(pitch_imu_position, pitch_enc_position, pitch_speed, dt);

    output_.yaw = motor_.yaw.output();
    output_.pitch = motor_.pitch.output();
  }

  /**
   * @brief 设置目标位置
   */
  void SetTarget(float yaw_position, float pitch_position) {
    target_.yaw_position = yaw_position;
    target_.pitch_position = pitch_position;

    motor_.yaw.SetPosition(yaw_position);
    motor_.pitch.SetPosition(pitch_position);
  }

  /**
   * @brief Yaw 设置前馈量
   */
  void SetYawFeedforward(float speed_feedforward, float output_feedforward) {
    motor_.yaw.SetFeedforward(speed_feedforward, output_feedforward);
  }

  /**
   * @brief Pitch 设置前馈量
   */
  void SetPitchFeedforward(float speed_feedforward, float output_feedforward) {
    motor_.pitch.SetFeedforward(speed_feedforward, output_feedforward);
  }

  /**
   * @brief 启用或禁用控制器（切换有力无力）
   */
  void Enable(bool is_enabled) {
    motor_.yaw.Enable(is_enabled);
    motor_.pitch.Enable(is_enabled);
  }

  // getters
  [[nodiscard]] auto &motor() { return motor_; }
  [[nodiscard]] auto &state() { return state_; }
  [[nodiscard]] auto &target() { return target_; }
  [[nodiscard]] auto &filter() { return filter_; }
  [[nodiscard]] auto &output() { return output_; }

 private:
  struct {
    MotorDriver yaw, pitch;
  } motor_{};
  struct {
    float yaw_position;
    float yaw_speed;
    float pitch_position;
    float pitch_speed;
  } state_{};  ///< 当前状态
  struct {
    float yaw_position;
    float pitch_position;
  } target_{};  ///< 目标状态
  struct {
    float yaw;
    float pitch;
  } output_{};  ///< 控制输出
  struct {
    // ABG yaw{0.05f, 0.0005f, 0.000005f};
    // ABG pitch{0.05f, 0.0005f, 0.000005f};
    // Biquad yaw_speed{0.067455f, 0.134910f, 0.067455f, -1.142980f, 0.412801f};
    // Biquad pitch_speed{0.067455f, 0.134910f, 0.067455f, -1.142980f, 0.412801f};
    bool is_initialized{false};
  } filter_;
};