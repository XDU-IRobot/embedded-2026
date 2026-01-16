#ifndef BOARDC_VOFA_HPP
#define BOARDC_VOFA_HPP

#include "usart.h"  // 包含 huart1 声明
#include <cstdio>
#include <cmath>

struct Biquad {
  double b0 = 0, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
  double x1 = 0, x2 = 0, y1 = 0, y2 = 0;

  // 初始化带阻(notch)滤波器
  // fs: 采样频率(Hz)
  // f0: 陷波中心频率(Hz)
  // Q: 品质因数（越大越窄）
  void initNotch(double fs, double f0, double Q) {
    double w0 = 2.0 * M_PI * f0 / fs;
    double cosw0 = cos(w0);
    double sinw0 = sin(w0);
    double alpha = sinw0 / (2.0 * Q);

    double b0u = 1.0;
    double b1u = -2.0 * cosw0;
    double b2u = 1.0;
    double a0u = 1.0 + alpha;
    double a1u = -2.0 * cosw0;
    double a2u = 1.0 - alpha;

    b0 = b0u / a0u;
    b1 = b1u / a0u;
    b2 = b2u / a0u;
    a1 = a1u / a0u;
    a2 = a2u / a0u;

    // 清空状态
    x1 = x2 = y1 = y2 = 0.0;
  }

  // 处理单点
  double process(double x) {
    double y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1;
    x1 = x;
    y2 = y1;
    y1 = y;
    return y;
  }
};

enum class ChirpType { kLinear, kLog };

// 可复用的 Chirp 发生器（头文件实现）
struct ChirpGenerator {
  // 配置
  double f0 = 1.0;   // 起始频率 Hz
  double f1 = 10.0;  // 结束频率 Hz
  double T = 10.0;   // 总时长 s
  double A = 0.01;   // 幅值（单位与控制量一致，例如 rad）
  ChirpType type = ChirpType::kLog;
  double fs = 250.0;  // 采样率 Hz（SetPosition 下发频率）

  // 状态
  uint32_t sample_idx = 0;
  uint32_t total_samples = 0;
  bool running = true;

  // 初始化（必须在启用前调用）
  void init(double _fs, double _f0, double _f1, double _T, double _A, ChirpType _type = ChirpType::kLog) {
    fs = _fs;
    f0 = _f0;
    f1 = _f1;
    T = _T;
    A = _A;
    type = _type;
    sample_idx = 0;
    total_samples = (uint32_t)std::max<uint32_t>(1, (uint32_t)std::ceil(fs * T));
    running = false;
  }

  // 启动
  void start() {
    sample_idx = 0;
    running = true;
  }

  // 停止
  void stop() { running = false; }

  bool isRunning() const { return running && (sample_idx < total_samples); }

  // 每周期调用：返回当前样点（rad 或与 A 单位一致）
  double step() {
    if (!running) return 0.0;
    if (sample_idx >= total_samples) {
      running = false;
      return 0.0;
    }

    double t = (double)sample_idx / fs;  // 当前时间 s
    double s = 0.0;

    if (type == ChirpType::kLinear) {
      double f_t = f0 + (f1 - f0) * (t / T);
      double phi = 2.0 * M_PI * (f0 * t + 0.5 * (f1 - f0) * (t * t) / T);
      s = A * sin(phi);
    } else {
      // 对数（指数）chirp
      if (f0 <= 0.0 || f1 <= 0.0) {
        s = 0.0;
      } else {
        double K = std::log(f1 / f0);
        if (std::fabs(K) < 1e-12) {
          double phi = 2.0 * M_PI * f0 * t;
          s = A * sin(phi);
        } else {
          double expTerm = std::exp(K * (t / T));
          double phi = 2.0 * M_PI * f0 * T / K * (expTerm - 1.0);
          s = A * sin(phi);
        }
      }
    }

    sample_idx++;
    if (sample_idx >= total_samples) running = false;
    return s;
  }
};

void VOFA_SendPitch_Blocking(uint32_t t_ms, double pitch_rad, double pitch_current);

#endif  // BOARDC_VOFA_HPP