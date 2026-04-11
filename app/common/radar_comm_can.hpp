#pragma once

#include <librm.hpp>

namespace rm::device {
class RadarCanCommunicator final : public CanDevice {
 public:
  explicit RadarCanCommunicator(rm::hal::CanInterface &can);
  RadarCanCommunicator(RadarCanCommunicator &&other) noexcept = default;
  RadarCanCommunicator() = delete;
  ~RadarCanCommunicator() override = default;

  [[nodiscard]] u16 distance_mm() const { return distance_mm_; };
  [[nodiscard]] u16 planner_distance_mm() const { return planner_distance_mm_; };
  [[nodiscard]] float yaw_mard() const { return static_cast<float>(yaw_mard_/1000.0); };
  [[nodiscard]] u8 status() const { return status_; };
  [[nodiscard]] bool vaild() const { return vaild_; };
  [[nodiscard]] bool fresh() const { return fresh_; };
  [[nodiscard]] u8 counter() const { return counter_; };

  void RxCallback(const hal::CanFrame *msg) override;

 private:
  // 收nuc
  u16 distance_mm_{};
  u16 planner_distance_mm_{};
  i16 yaw_mard_{};
  u8 status_{};
  bool vaild_{};
  bool fresh_{};
  u8 counter_{};
  // 缓冲区
  // u8 tx_buf_[8]{};
};
}  // namespace rm::device
