//
// Created by 34236 on 2025/12/1.
//

#ifndef BOARDC_DEVICE_HPP
#define BOARDC_DEVICE_HPP

#include "librm/device/can_device.hpp"
#include "librm/core/typedefs.hpp"

namespace rm::device {

/**
 * @brief ME02 编码器设备
 *
 * 数据格式示例: 55 55 aa bb cc dd ee ff
 * - 角度寄存器 = (0xbb << 8) | 0xaa
 * - 角速度寄存器 = (0xdd << 8) | 0xcc
 * - 转数寄存器 = (0xff << 8) | 0xee
 *
 * 角度(°) = reg * 360 / 32768
 * 角速度(°/s) = reg * 360 / 32768 / sample_time_s (默认 sample_time_s = 0.1s)
 */
class ME02 final : public CanDevice {
 public:
  ME02() = delete;
  ME02(ME02 &&) noexcept = default;
  ~ME02() override = default;

  // can: CAN 总线对象引用；rx_id: 本设备接收报文的 CAN 标识符
  ME02(hal::CanInterface &can, u16 rx_id, f32 sample_time_s = 0.1f)
      : CanDevice(can, rx_id), sample_time_s_(sample_time_s) {}

  // 回调中解析原始寄存器与计算值
  void RxCallback(const hal::CanMsg *msg) override {
    Heartbeat();
    // 按照给定字节序，msg->data: [0]=0x55 [1]=0x55 [2]=aa [3]=bb [4]=cc [5]=dd [6]=ee [7]=ff
    angle_reg_ = static_cast<u16>((msg->data[3] << 8) | msg->data[2]);
    speed_reg_ = static_cast<u16>((msg->data[5] << 8) | msg->data[4]);
    rotations_ = static_cast<u16>((msg->data[7] << 8) | msg->data[6]);

    // 计算物理量
    angle_deg_ = static_cast<f32>(angle_reg_) * 360.f / 32768.f;
    angular_speed_dps_ = static_cast<f32>(speed_reg_) * 360.f / 32768.f / sample_time_s_;
  }

  // 访问器
  [[nodiscard]] u16 angle_reg() const { return angle_reg_; }
  [[nodiscard]] f32 angle_deg() const { return angle_deg_; }

  [[nodiscard]] u16 speed_reg() const { return speed_reg_; }
  [[nodiscard]] f32 angular_speed_dps() const { return angular_speed_dps_; }

  [[nodiscard]] u16 rotations() const { return rotations_; }

 private:
  u16 angle_reg_{};  // 原始角度寄存器
  f32 angle_deg_{};  // 计算得到的角度(度)

  u16 speed_reg_{};          // 原始角速度寄存器
  f32 angular_speed_dps_{};  // 计算得到的角速度(°/s)

  u16 rotations_{};  // 原始转数寄存器

  f32 sample_time_s_{};  // 角速度采样时间，秒
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_ACTUATOR_ME02_HPP