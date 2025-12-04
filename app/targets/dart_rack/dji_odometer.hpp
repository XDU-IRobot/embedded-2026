//
// Created by 34236 on 2025/12/1.
//

#ifndef DJI_ODOMETER_HPP
#define DJI_ODOMETER_HPP

namespace rm::device {

/*
DjiOdometer
  - ecd: 0..8191
  - Update(ecd, current_ma) 按用户规定的规则更新圈数与堵转时间
*/
class DjiOdometer {
 public:
  DjiOdometer() = default;

  void Reset(const int64_t revolutions = 0, const uint16_t last_ecd = 0) {
    revolutions_ = revolutions;
    last_ecd_ = last_ecd;
    initialized_ = false;  // 下次 Update 会用传入的 ecd 初始化
    stall_time_ = 0;
  }

  void Update(const uint16_t ecd, const int32_t current_ma) {
    if (!initialized_) {
      last_ecd_ = ecd;
      initialized_ = true;
      stall_time_ = 0;
      return;
    }

    if (const int32_t delta = static_cast<int32_t>(ecd) - static_cast<int32_t>(last_ecd_); delta > 0) {
      if (delta > 4096) {
        // 反向跨圈：圈数减1
        --revolutions_;
        last_ecd_ = ecd;
        stall_time_ = 0;
      } else {
        // 正常小幅正转
        last_ecd_ = ecd;
        stall_time_ = 0;
      }
    } else if (delta < 0) {
      if (delta < -4096) {
        // 正向跨圈：圈数加1
        ++revolutions_;
        last_ecd_ = ecd;
        stall_time_ = 0;
      } else {
        // 正常小幅反转
        last_ecd_ = ecd;
        stall_time_ = 0;
      }
    } else {
      // delta == 0
      if (current_ma > 1000) {
        ++stall_time_;
      } else {
        stall_time_ = 0;
      }
    }
  }

  [[nodiscard]] int64_t revolutions() const { return revolutions_; }
  [[nodiscard]] uint16_t last_ecd() const { return last_ecd_; }
  [[nodiscard]] uint32_t stall_time() const { return stall_time_; }

 private:
  int64_t revolutions_{0};  // 整数圈数
  uint16_t last_ecd_{0};
  bool initialized_{false};
  uint32_t stall_time_{0};  // 堵转时间计数（调用次数或采样次数）
};

}  // namespace rm::device
#endif
