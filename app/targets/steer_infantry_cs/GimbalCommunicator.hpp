#ifndef GIMBAL_COMMUNICATOR_HPP
#define GIMBAL_COMMUNICATOR_HPP

#include <librm.hpp>

namespace rm::device {
class GimbalCommunicator final : public CanDevice {
 public:
  explicit GimbalCommunicator(hal::CanInterface &can);
  GimbalCommunicator() = delete;
  ~GimbalCommunicator() override = default;

  [[nodiscard]] f32 remote_speed_x() const { return remote_speed_x_; }
  [[nodiscard]] f32 remote_speed_y() const { return remote_speed_y_; }
  [[nodiscard]] u8 chassis_mode() const { return chassis_mode_; }
  [[nodiscard]] u8 UI_show_flag() const { return UI_show_flag_; }
  [[nodiscard]] u8 get_target_flag() const { return get_target_flag_; }
  [[nodiscard]] u8 suggest_fire_flag() const { return suggest_fire_flag_; }
  [[nodiscard]] i8 aim_speed_change() const { return aim_speed_change_; }
  [[nodiscard]] auto robot_hp() const { return robot_hp_; }

  void RxCallback(const hal::CanFrame *msg) override;
  void SendGimbalCommand(u16 current_heat, u16 heat_limit, float ammo_speed, u8 power_state, u8 robot_id);

 private:
  f32 remote_speed_x_{};
  f32 remote_speed_y_{};
  u8 chassis_mode_{};
  u8 UI_show_flag_{};
  u8 get_target_flag_{};
  u8 suggest_fire_flag_{};
  i8 aim_speed_change_{};
  u16 robot_hp_[5]{};
  u8 tx_buf_[8]{};
};
}  // namespace rm::device

#endif  // GIMBAL_COMMUNICATOR_HPP
