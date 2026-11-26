#pragma once

#include "smc.hpp"
#include <librm.hpp>
#include <limits>

class MotorDriver {
 public:
  MotorDriver() {
    state_.mode = Mode::kPosToSpeed;
    pid_.position.SetDiffFirst(false)
        .SetCircular(true)
        .SetCircularCycle(M_PI * 2)
        .SetFuzzy(true)
        .SetFuzzyErrorScale(M_PI)
        .SetDiffLpfAlpha(0.2f);
    smc_.SetCircularCycle(M_PI * 2, true);
  };

  enum class Mode {
    kPos,         // 单位置环
    kSpeed,       // 单速度环
    kPosToSpeed,  // 位置速度串级双环
    kSMC          // 滑模控制
  };

  void Update(const float raw_position, const float raw_speed, const float dt = 1.f);
  void UpdateWithImu(const float raw_imu_position, const float raw_enc_position, const float input_speed,
                     const float dt);

  void SetMode(const Mode mode);
  void SetSpeed(float target_speed, float speed_feedforward = 0.f);
  void SetPosition(float target_position);
  void SetFeedforward(float speed_feedforward = 0.f, float output_feedforward = 0.f);
  void SetOutputLimits(const float min, const float max, const bool is_enabled = true);  // 最终输出限幅
  void SetPositionLimits(const float min, const float max,
                         const bool is_enabled = true);  // 机械位置限幅（无imu，情况下和目标限幅类似）
  void SetTargetLimits(const float min, const float max, const bool is_enabled = true);  // 目标值限幅

  void Enable(const bool is_enable) { is_enable_ = is_enable; }
  void EnableOutputLimits(const bool is_enable) { is_output_limits_ = is_enable; }
  void EnablePositionLimits(const bool is_enable) { is_position_limits_ = is_enable; }
  void EnableTargetLimits(const bool is_enable) { is_target_limits_ = is_enable; }

  // 对外接口
  [[nodiscard]] auto &smc() { return smc_; }
  [[nodiscard]] auto &pid() { return pid_; }
  [[nodiscard]] const auto &state() const { return state_; }
  [[nodiscard]] const auto &target() const { return target_; }
  [[nodiscard]] const auto &output() const { return output_; }

 private:
  bool is_enable_{false};           // 启用开关
  bool is_output_limits_{false};    // 输出限位
  bool is_target_limits_{false};    // 目标限位，仅对位置模式有效，速度模式无效
  bool is_position_limits_{false};  // 机械限位，仅对位置模式有效，速度模式无效

  SMC<SMCMode::kSMC> smc_;

  struct {
    rm::modules::PID speed, position;
  } pid_{};

  struct {
    float speed{}, position{};
    Mode mode{};
  } state_{};

  struct {
    ///< 物理电机限位
    float min_position{-std::numeric_limits<float>::infinity()};
    float max_position{std::numeric_limits<float>::infinity()};
    ///< 目标值限位
    float min_target{-std::numeric_limits<float>::infinity()};
    float max_target{std::numeric_limits<float>::infinity()};
    ///< 电机输出限幅
    float min_out{-std::numeric_limits<float>::infinity()};
    float max_out{std::numeric_limits<float>::infinity()};
  } limits_{};

  struct {
    float speed, position;
    float speed_ff;   ///< 速度前馈
    float output_ff;  ///< 控制量前馈，控制量具体是力矩、电流或者什么，取决于电机驱动
  } target_{};

  float output_{};

  void CalcOutput(float target_position, const float dt);
  float ApplyPositionLimits(float target_world_pos, float current_world_pos, float encoder_pos) const;
};
