#pragma once

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief 有限傅里叶级数轨迹发生器
 *
 * 轨迹方程为：
 * q(t) = q0 + sum_{k=1}^N [ (a_k / (k*wf)) * sin(k*wf*t) - (b_k / (k*wf)) * cos(k*wf*t) ]
 * dq(t) = sum_{k=1}^N [ a_k * cos(k*wf*t) + b_k * sin(k*wf*t) ]
 * ddq(t) = sum_{k=1}^N [ -a_k * k*wf * sin(k*wf*t) + b_k * k*wf * cos(k*wf*t) ]
 *
 * @tparam N 傅里叶谐波项数（通常选 5 ）
 */
template <size_t N>
class FourierTrajectoryGenerator {
 private:
  float wf_;    // 基频 (rad/s)
  float q0_;    // 位置偏置 (rad)
  float a_[N];  // 傅里叶系数 a
  float b_[N];  // 傅里叶系数 b

 public:
  /**
   * @brief 构造函数
   * @param base_freq_hz 基频(Hz)，例如 0.1Hz 即周期10s
   * @param offset 初始位置偏置
   * @param a_coeffs 长度为 N 的系数组 a
   * @param b_coeffs 长度为 N 的系数组 b
   */
  FourierTrajectoryGenerator(float base_freq_hz, float offset, const float a_coeffs[N], const float b_coeffs[N]) {
    wf_ = 2.0f * M_PI * base_freq_hz;
    q0_ = offset;
    for (size_t i = 0; i < N; ++i) {
      a_[i] = a_coeffs[i];
      b_[i] = b_coeffs[i];
    }
  }

  struct Output {
    float q;
    float dq;
    float ddq;
  };

  /**
   * @brief 计算给定时间 t 的理想位置、速度、加速度
   * @param t 当前时间(秒)
   * @param target_q 输出: 目标位置(rad)
   * @param target_dq 输出: 目标速度(rad/s)
   * @param target_ddq 输出: 目标加速度(rad/s^2)
   */
  Output evaluate(float t) const {
    Output output;
    output.q = q0_;
    output.dq = 0.0f;
    output.ddq = 0.0f;

    for (size_t i = 0; i < N; ++i) {
      float k = static_cast<float>(i + 1);
      float kwf = k * wf_;
      float phase = kwf * t;

      // 注意: 在极端性能敏感时，你可以将这里的 std::sin/cos 替换为 arm_sin_f32/arm_cos_f32
      float s = std::sin(phase);
      float c = std::cos(phase);

      // 叠加轨迹
      output.q += (a_[i] / kwf) * s - (b_[i] / kwf) * c;
      output.dq += a_[i] * c + b_[i] * s;
      output.ddq += -a_[i] * kwf * s + b_[i] * kwf * c;
    }
    return output;
  }
};