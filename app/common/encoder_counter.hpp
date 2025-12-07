
#pragma once

#include <librm.hpp>

/**
 * @brief 带堵转检测功能的编码器计圈器，用于跟踪没有绝对位置的编码器的过圈数
 */
class EncoderCounter {
 public:
  EncoderCounter() = default;

  /**
   * @brief 重置计圈器状态
   * @param cycles        重置的圈数
   * @param value         重置的编码器读数
   */
  void Reset(int cycles = 0, float value = 0) {
    revolutions_ = cycles;
    last_ecd_ = value;
    initialized_ = false;  // 下次 Update 会用传入的 ecd 初始化
    stall_time_ = 0;
    linear_ticks_=0;
  }

  /**
   * @brief 更新编码器计圈器状态
   * @param ecd          当前编码器读数
   * @param current_ma   当前电机电流，单位毫安，不使用堵转检测功能的话可不传
   */
  void Update(const uint16_t ecd, const int16_t current_ma = 0.f) {
    if (!initialized_) {
      last_ecd_ = ecd;
      initialized_ = true;
      stall_time_ = 0;
      base_ecd_=ecd;
      return;
    }

    // 计算这次相较于上次的增量
    const int32_t delta = static_cast<int32_t>(ecd) - static_cast<int32_t>(last_ecd_);

    if (delta > 0) {
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
    }

    // 如果电机几乎没动，则检查堵转情况
    if (rm::modules::IsNear(delta, 0, 1)) {
      if (current_ma > 1000||current_ma<=-1000) {  // 电流大于1A，认为可能堵转，递增堵转时间计数
        ++stall_time_;
      } else {
        stall_time_ = 0;
      }
    }
    linear_ticks_ = static_cast<int64_t>(revolutions_) * 8192 +
                (static_cast<int32_t>(last_ecd_) - static_cast<int32_t>(base_ecd_));
  }

  [[nodiscard]] int64_t revolutions() const { return revolutions_; }
  [[nodiscard]] uint16_t last_ecd() const { return last_ecd_; }
  [[nodiscard]] uint32_t stall_time() const { return stall_time_; }
  [[nodiscard]] int64_t linear_ticks() const { return linear_ticks_; }

 private:
  int revolutions_{0};  ///< 圈数
  uint16_t last_ecd_{0};///传入的编码器值
  uint16_t base_ecd_{0};         ///< Reset 时的基准编码器读数
  int32_t linear_ticks_{0};      ///< 展开的直线里程（以编码器刻度为单位）
  bool initialized_{false};/// 是否已初始化
  size_t stall_time_{0};  ///< 堵转时间计数（按调用Update次数）
};