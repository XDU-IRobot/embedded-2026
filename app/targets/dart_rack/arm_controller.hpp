#pragma once

#include <array>

#include <librm.hpp>

class ArmController {
 public:
  struct Output {
    static Output Zero() {
      Output out;
      out.positions.fill(0.f);
      out.velocities.fill(0.f);
      return out;
    }
    std::array<float, 4> positions;   ///< 关节位置输出，单位：rad
    std::array<float, 4> velocities;  ///< 关节速度输出，单位：rad/s
  };

  enum Status {
    kIdle,
    kMoving,
  };

  ArmController() = delete;
  explicit ArmController(std::array<std::pair<float, float>, 4> joint_limits,  //
                         std::array<rm::modules::TrajectoryLimiter, 4> trajectory_limiters)
      : trajectory_limiter_(std::move(trajectory_limiters)), joint_limits_(std::move(joint_limits)) {}

  void GoTo(const std::array<float, 4> &target_positions) {
    for (size_t i = 0; i < 4; ++i) {
      trajectory_limiter_[i].SetTarget(
          std::clamp(target_positions[i], joint_limits_[i].first, joint_limits_[i].second));
    }
    UpdateStatus();
  }

  void Update(const std::array<float, 4> &current_positions, float dt = 0.001f) {
    current_positions_ = current_positions;

    for (size_t i = 0; i < 4; ++i) {
      trajectory_limiter_[i].Update(dt);
      output_.positions[i] = trajectory_limiter_[i].current_position();
      output_.velocities[i] = trajectory_limiter_[i].current_velocity();
    }

    UpdateStatus();
  }

  void ResetAt(const std::array<float, 4> &positions) {
    current_positions_ = positions;
    for (size_t i = 0; i < 4; ++i) {
      trajectory_limiter_[i].ResetAt(positions[i]);
    }
  }

  const auto &joint_limits() { return joint_limits_; }
  const auto &current_positions() const { return current_positions_; }
  const auto &output() const { return output_; }
  Status status() const { return status_; }

 private:
  void UpdateStatus() {
    const bool all_limiters_converged = std::ranges::all_of(
        trajectory_limiter_, [](const rm::modules::TrajectoryLimiter &limiter) { return limiter.IsAtTarget(); });

    if (all_limiters_converged) {
      // 方便调试先禁用下面的判断逻辑
      status_ = kIdle;
      return;
    }
    status_ = kMoving;
  }

  std::array<float, 4> current_positions_{};
  Output output_{};
  std::array<rm::modules::TrajectoryLimiter, 4> trajectory_limiter_;
  std::array<std::pair<float, float>, 4> joint_limits_;
  Status status_{kIdle};
  int fault_counter_{0};
};