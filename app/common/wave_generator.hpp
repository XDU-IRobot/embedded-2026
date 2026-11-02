#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

/**
 * @brief 波形生成器
 * @note  扁平化设计：所有参数直接暴露为 public 成员
 * @note  每周期调用 Update(dt)，然后 GetValue() 读值
 * @note  dt 必须等于调用间隔（1kHz → dt = 0.001f）
 */
struct WaveGenerator {
  enum class Type : uint8_t {
    kSine = 0,
    kTriangle,
    kSquare,
    kSawtooth,
    kPulse,      // 脉冲波：一个周期内只有一个窄脉冲
    kTrapezoid,  // 梯形波：上升 -> 平台 -> 下降 -> 低电平
  };

  // ==================== 通用参数 ====================
  Type type = Type::kSine;
  float freq = 1.0f;       // 频率 Hz
  float amplitude = 1.0f;  // 幅值
  float offset = 0.0f;     // 偏置

  // ==================== 各波形特有参数 ====================
  float tri_width = 0.5f;  // 三角波上升沿占比 [0.01, 0.99]
  float sq_duty = 0.5f;    // 方波高电平占比 [0.01, 0.99]
  float saw_width = 0.9f;  // 锯齿波上升沿占比 [0.01, 0.99]

  // 脉冲波参数
  float pulse_width = 0.2f;
  float pulse_skew = 0.5f;

  // 🎯 梯形波参数：三个阶段宽度占比
  // 剩余部分自动作为低电平（0 电平）
  // 若三者之和 > 1.0，内部会自动等比缩放
  float trap_rise = 0.2f;  // 上升沿占比
  float trap_high = 0.4f;  // 高电平平台占比
  float trap_fall = 0.2f;  // 下降沿占比
  // 低电平占比 = 1.0f - (trap_rise + trap_high + trap_fall)

  // ==================== 构造 ====================
  WaveGenerator() = default;

  WaveGenerator(Type t, float f, float a, float o = 0.0f) : type(t), freq(f), amplitude(a), offset(o) {}

  // ==================== 核心接口 ====================
  void Update(float dt) {
    if (dt <= 0.0f || !std::isfinite(dt) || freq <= 0.0f) {
      value_ = offset;
      return;
    }

    const float period = 1.0f / freq;
    time_ += dt;
    if (time_ >= period) time_ -= period * std::floor(time_ / period);

    const float phase = time_ * freq;  // [0, 1)

    switch (type) {
      case Type::kSine:
        value_ = offset + amplitude * std::sin(phase * 2.0f * M_PI);
        break;

      case Type::kTriangle: {
        const float w = std::clamp(tri_width, 0.01f, 0.99f);
        float v;
        if (phase < w) {
          v = -1.0f + 2.0f * (phase / w);
        } else {
          v = 1.0f - 2.0f * ((phase - w) / (1.0f - w));
        }
        value_ = offset + amplitude * v;
        break;
      }

      case Type::kSquare: {
        const float d = std::clamp(sq_duty, 0.01f, 0.99f);
        value_ = offset + ((phase < d) ? amplitude : -amplitude);
        break;
      }

      case Type::kSawtooth: {
        const float w = std::clamp(saw_width, 0.01f, 0.99f);
        float v;
        if (phase < w) {
          v = -1.0f + 2.0f * (phase / w);
        } else {
          v = 1.0f - 2.0f * ((phase - w) / (1.0f - w));
        }
        value_ = offset + amplitude * v;
        break;
      }

      case Type::kPulse: {
        const float w = std::clamp(pulse_width, 0.01f, 0.99f);
        if (phase >= w) {
          value_ = offset;
          break;
        }
        const float skew = std::clamp(pulse_skew, 0.01f, 0.99f);
        const float u = phase / w;
        float v;
        if (u < skew) {
          v = u / skew;
        } else {
          v = 1.0f - (u - skew) / (1.0f - skew);
        }
        value_ = offset + amplitude * v;
        break;
      }

      case Type::kTrapezoid: {
        float r = std::max(0.0f, trap_rise);
        float h = std::max(0.0f, trap_high);
        float f = std::max(0.0f, trap_fall);
        float sum = r + h + f;

        // 如果三者之和大于1，进行归一化
        if (sum > 1.0f) {
          r /= sum;
          h /= sum;
          f /= sum;
        }

        const float t1 = r;       // 上升结束
        const float t2 = t1 + h;  // 高电平结束
        const float t3 = t2 + f;  // 下降结束

        float v;
        if (phase < t1) {
          // 上升沿 (0 -> 1)
          v = (t1 > 1e-6f) ? (phase / t1) : 1.0f;
        } else if (phase < t2) {
          // 高电平 (1)
          v = 1.0f;
        } else if (phase < t3) {
          // 下降沿 (1 -> 0)
          v = (f > 1e-6f) ? (1.0f - (phase - t2) / f) : 0.0f;
        } else {
          // 低电平 (0)
          v = 0.0f;
        }
        value_ = offset + amplitude * v;
        break;
      }
    }
  }

  [[nodiscard]] float GetValue() const { return value_; }

  void Reset() {
    time_ = 0.0f;
    value_ = offset;
  }

 private:
  float time_ = 0.0f;
  float value_ = 0.0f;
};

namespace WaveFactory {

inline WaveGenerator Sine(float freq, float amp, float offset = 0.0f) {
  WaveGenerator w;
  w.type = WaveGenerator::Type::kSine;
  w.freq = freq;
  w.amplitude = amp;
  w.offset = offset;
  return w;
}

inline WaveGenerator Triangle(float freq, float amp, float width = 0.5f, float offset = 0.0f) {
  WaveGenerator w;
  w.type = WaveGenerator::Type::kTriangle;
  w.freq = freq;
  w.amplitude = amp;
  w.tri_width = width;
  w.offset = offset;
  return w;
}

inline WaveGenerator Square(float freq, float amp, float duty = 0.5f, float offset = 0.0f) {
  WaveGenerator w;
  w.type = WaveGenerator::Type::kSquare;
  w.freq = freq;
  w.amplitude = amp;
  w.sq_duty = duty;
  w.offset = offset;
  return w;
}

inline WaveGenerator Sawtooth(float freq, float amp, float width = 0.9f, float offset = 0.0f) {
  WaveGenerator w;
  w.type = WaveGenerator::Type::kSawtooth;
  w.freq = freq;
  w.amplitude = amp;
  w.saw_width = width;
  w.offset = offset;
  return w;
}

inline WaveGenerator Pulse(float freq, float amp, float width = 0.2f, float skew = 0.5f, float offset = 0.0f) {
  WaveGenerator w;
  w.type = WaveGenerator::Type::kPulse;
  w.freq = freq;
  w.amplitude = amp;
  w.offset = offset;
  w.pulse_width = width;
  w.pulse_skew = skew;
  return w;
}

// 梯形波：rise=上升沿占比，high=高电平占比，fall=下降沿占比
inline WaveGenerator Trapezoid(float freq, float amp, float rise = 0.2f, float high = 0.4f, float fall = 0.2f,
                               float offset = 0.0f) {
  WaveGenerator w;
  w.type = WaveGenerator::Type::kTrapezoid;
  w.freq = freq;
  w.amplitude = amp;
  w.offset = offset;
  w.trap_rise = rise;
  w.trap_high = high;
  w.trap_fall = fall;
  return w;
}

}  // namespace WaveFactory