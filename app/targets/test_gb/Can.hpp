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

  // 发nuc
  u8 mode{};
  u16 imu_count{};
  // 收nuc
  u8 AimbotState{};
  u8 AimbotTarget{};
  f32 Yaw{};
  f32 Pitch{};
  u8 NucStartFlag{};

  void RxCallback(const hal::CanFrame *msg) override;
  void SendMessage();

 private:
  u8 tx_buf1_[8]{};
  u8 tx_buf2_[8]{};
};
}  // namespace rm::device

#endif  // CAN_HPP
