#ifndef CHASSIS_COMMUNICATOR_HPP
#define CHASSIS_COMMUNICATOR_HPP

#include <librm.hpp>

namespace rm::device {
class ChassisCommunicator final : public CanDevice {
 public:
  explicit ChassisCommunicator(hal::CanInterface &can);
  ChassisCommunicator() = delete;
  ~ChassisCommunicator() override = default;

  [[nodiscard]] u16 heat_real() const { return heat_real_; }
  [[nodiscard]] u16 heat_limit() const { return heat_limit_; }
  [[nodiscard]] f32 ammo_speed() const { return ammo_speed_; }
  [[nodiscard]] u8 robot_id() const { return robot_id_; }
  [[nodiscard]] u8 gimbal_power_state() const { return gimbal_power_state_; }
  [[nodiscard]] u8 chassis_power_state() const { return chassis_power_state_; }
  [[nodiscard]] u8 ammo_power_state() const { return ammo_power_state_; }

  void RxCallback(const hal::CanFrame *msg) override;
  void SendChassisCommand(f32 chassis_move_x, f32 chassis_move_y, u8 chassis_state, u8 ui_refresh_flag,
                          u8 get_target_flag, u8 suggest_fire_flag, i8 aim_speed_change);

 private:
  u16 heat_real_{};
  u16 heat_limit_{};
  f32 ammo_speed_{};
  u8 robot_id_{};
  u8 gimbal_power_state_{};
  u8 chassis_power_state_{};
  u8 ammo_power_state_{};
  u8 tx_buf_[8]{};
};
}  // namespace rm::device

#endif  // CHASSIS_COMMUNICATOR_HPP
