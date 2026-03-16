#ifndef GIMBAL_COMMUNICATOR_HPP
#define GIMBAL_COMMUNICATOR_HPP

#include <librm.hpp>

namespace rm::device {
class GimbalCommunicator final : public CanDevice {
 public:
  explicit GimbalCommunicator(hal::CanInterface &can);
  GimbalCommunicator() = delete;
  ~GimbalCommunicator() override = default;

  f32 remote_speed_x() const { return remote_speed_x_; }
  f32 remote_speed_y() const { return remote_speed_y_; }
  u8 chassis_mode() const { return chassis_mode_; }
  u8 UI_show_flag() const { return UI_show_flag_; }
  u8 get_target_flag() const { return get_target_flag_; }
  u8 suggest_fire_flag() const { return suggest_fire_flag_; }
  u8 aim_speed_change() const { return aim_speed_change_; }

  void RxCallback(const hal::CanFrame *msg) override;
  void SendGimbalCommand(i8 chassis_move_x, i8 chassis_move_y, u8 chassis_state, u8 ui_refresh_flag,
                          u8 get_target_flag, u8 suggest_fire_flag, i8 aim_speed_change);

 private:
  f32 remote_speed_x_{};
  f32 remote_speed_y_{};
  u8 chassis_mode_{};
  u8 UI_show_flag_{};
  u8 get_target_flag_{};
  u8 suggest_fire_flag_{};
  i8 aim_speed_change_{};
  u8 tx_buf_[8]{};

  void ParseRxData(const hal::CanFrame *msg);
};
}  // namespace rm::device

#endif  // GIMBAL_COMMUNICATOR_HPP
