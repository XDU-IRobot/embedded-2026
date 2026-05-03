#ifndef NAVIGATE_CAN_HPP
#define NAVIGATE_CAN_HPP

#include <librm.hpp>

namespace rm::device {
class NavigateCanCommunicator final : public CanDevice {
 public:
  explicit NavigateCanCommunicator(rm::hal::CanInterface &can);
  NavigateCanCommunicator(NavigateCanCommunicator &&other) noexcept = default;
  NavigateCanCommunicator() = delete;
  ~NavigateCanCommunicator() override = default;

  [[nodiscard]] f32 chassis_target_x() const;
  [[nodiscard]] f32 chassis_target_y() const;
  [[nodiscard]] f32 chassis_target_w() const;
  [[nodiscard]] f32 target_yaw_speed() const;
  [[nodiscard]] u8 scan_mode() const;
  [[nodiscard]] u8 perception_flag() const;
  [[nodiscard]] u8 aimbot_mode() const;
  [[nodiscard]] u8 outpost_mode() const;

  void Update();
  void RxCallback(const hal::CanFrame *msg) override;

 private:
  // 收nuc
  f32 chassis_target_x_{};
  f32 chassis_target_y_{};
  f32 chassis_target_w_{};
  f32 target_yaw_speed_{};
  u8 scan_mode_{};
  u8 perception_flag_{};
  u8 aimbot_mode_{};
  u8 outpost_mode_{};
  // 缓冲区
  u8 tx_buf_[8]{};
};
}  // namespace rm::device

#endif  // NAVIGATE_CAN_HPP
