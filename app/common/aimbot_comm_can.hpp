#ifndef CAN_HPP
#define CAN_HPP

#include <librm.hpp>

namespace rm::device {
class CanCommunicator final : public CanDevice {
 public:
  explicit CanCommunicator(rm::hal::CanInterface &can);
  CanCommunicator(CanCommunicator &&other) noexcept = default;
  CanCommunicator() = delete;
  ~CanCommunicator() override = default;

  [[nodiscard]] u8 aimbot_state() const ;
  [[nodiscard]] u8 aimbot_target() const ;
  [[nodiscard]] f32 yaw() const ;
  [[nodiscard]] f32 pitch() const ;
  [[nodiscard]] u8 nuc_start_flag() const ;

  void UpdateQuaternion(f32 w, f32 x, f32 y, f32 z);
  void UpdateControlFlag(u8 robot_id, u8 mode, u16 imu_count);
  void RxCallback(const hal::CanFrame *msg) override;

 private:
  // 收nuc
  u8 aimbot_state_{};
  u8 aimbot_target_{};
  f32 yaw_{};
  f32 pitch_{};
  u8 nuc_start_flag_{};
  // 缓冲区
  u8 tx_buf_[8]{};
};
}  // namespace rm::device

#endif  // CAN_HPP
