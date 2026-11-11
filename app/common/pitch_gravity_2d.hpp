#pragma once

#include <cmath>
#include <array>
#include <cstddef>

class PitchGravity {
 public:
  // ==========================================
  // 传统物理模型法 (A * cos(θ + φ))
  // ==========================================
  class Model {
   public:
    float gain_nm = 0.0f;
    float phase_rad = 0.0f;

    static float Calc(float gain, float phase, float angle) {
      if (!std::isfinite(angle)) return 0.0f;
      return gain * std::cos(angle + phase);
    }

    static void ABToGainPhase(float a, float b, float &out_gain, float &out_phase) {
      out_gain = std::hypot(a, b);
      out_phase = out_gain > 0.0f ? std::atan2(-b, a) : 0.0f;
    }

    float Calculate(float angle) const {
      return Calc(gain_nm, phase_rad, angle);
    }
  };

  // ==========================================
  // 高阶傅里叶拟合法
  // ==========================================
  template <std::size_t Order>
  class Fourier {
   public:
    static_assert(Order >= 0, "Fourier Order must be >= 0!");

    std::array<float, Order + 1> cos_coeffs_{};
    std::array<float, Order + 1> sin_coeffs_{};

    Fourier &SetCoeff(std::size_t i, float a_i, float b_i) {
      if (i <= Order) {
        cos_coeffs_[i] = a_i;
        sin_coeffs_[i] = b_i;
      }
      return *this;
    }

    float Calculate(float angle_rad) const {
      if (!std::isfinite(angle_rad)) return 0.0f;

      float result = cos_coeffs_[0]; // a0
      for (std::size_t i = 1; i <= Order; ++i) {
        result += cos_coeffs_[i] * std::cos(i * angle_rad);
        result += sin_coeffs_[i] * std::sin(i * angle_rad);
      }
      return result;
    }
  };


  // ==========================================
  // 采样器
  // ==========================================
  template <std::size_t kMaxSamples>
  class Sampler {
   public:
    static_assert(kMaxSamples >= 2, "Max samples must be >= 2!");
    Sampler() = delete;
    explicit Sampler(float begin_rad, float end_rad) : begin_(begin_rad), end_(end_rad), target_(begin_rad) {}

    struct SampleData {
      float angle;
      float output;
    };

    void Start() {
      index_ = 0;
      target_ = begin_;
      is_sampling_ = true;
      ClearWindow();
    }

    void Stop() {
      is_sampling_ = false;
    }

    void Record(float angle, float output) {
      if (!is_sampling_) return;
      buffer_[index_++] = {angle, output};
      if (index_ == 2 * kMaxSamples) Stop();
    }

    float Update(float angle, float speed, float torque, float dt = 0.002f) {
      if (!is_sampling_) return target_;

      if (!std::isfinite(angle) || !std::isfinite(speed) ||
          !std::isfinite(torque) || !(dt > 0.0f)) {
        return target_;
      }

      const std::size_t point = index_ < kMaxSamples ? index_ : 2 * kMaxSamples - 1 - index_;
      const float goal = begin_ + (end_ - begin_) * point / (kMaxSamples - 1);

      // 后续采样点之间缓慢移动，0.08 rad/s
      const float delta = goal - target_;
      const float step = 0.08f * dt;

      target_ += delta > step ? step : (delta < -step ? -step : delta);

      // 尚未停稳，重新等待
      if (std::fabs(target_ - goal) > 1e-5f ||
          std::fabs(angle - goal) > 0.10f ||
          std::fabs(speed) > 0.03f) {
        ClearWindow();
        return target_;
      }

      stable_time_ += dt;

      // 连续停稳0.8秒后开始采样
      if (stable_time_ < 0.8f) return target_;

      angle_sum_ += angle;
      torque_sum_ += torque;
      ++sample_count_;

      // 再采约1秒，得到这一次的平均值
      if (stable_time_ < 1.8f) return target_;

      Record(static_cast<float>(angle_sum_ / sample_count_), static_cast<float>(torque_sum_ / sample_count_));
      ClearWindow();
      return target_;
    }

    const SampleData *GetBuffer() const { return buffer_.data(); }
    std::size_t GetIndex() const { return index_; }
    bool IsSampling() const { return is_sampling_; }

   private:
    void ClearWindow() {
      stable_time_ = 0.0f;
      angle_sum_ = torque_sum_ = 0.0;
      sample_count_ = 0;
    }

    float begin_, end_, target_;
    float stable_time_ = 0.0f;

    double angle_sum_ = 0.0;
    double torque_sum_ = 0.0;
    unsigned sample_count_ = 0;

    std::array<SampleData, 2 * kMaxSamples> buffer_{};   // 上下两次采样

    std::size_t index_ = 0;  // 两遍累计进度：0～2N
    bool is_sampling_ = false;
  };
};
