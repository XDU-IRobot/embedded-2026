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
  [[nodiscard]] i16 yaw_mard() const { return yaw_mard_; };
  [[nodiscard]] u8 status() const { return status_; };
  [[nodiscard]] bool vaild() const { return vaild_; };
  [[nodiscard]] bool fresh() const { return fresh_; };
  [[nodiscard]] u8 counter() const { return counter_; };

  // void UpdateControl(f32 yaw, f32 pitch, f32 roll, u8 robot_id, u8 mode, u16 imu_count, f32 bullet_speed);
  // void UpdateQuaternion(f32 w, f32 x, f32 y, f32 z);
  // void UpdateControlFlag(u8 robot_id, u8 mode, u16 imu_count, u32 imu_time);
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
  u8 tx_buf_[8]{};
};
}  // namespace rm::device
