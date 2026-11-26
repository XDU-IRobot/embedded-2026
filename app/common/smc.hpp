#pragma once

#include "math.h"
#include <librm.hpp>

enum class SMCMode {
  kSMC,     // 线性滑膜面
  kTSMC,    // 终端滑模面
  kISMC     // 积分滑膜面
};

// SMC YawSMC(20, 120, 0, 0.001, 21, 27, 25000, 0.8, 0.5, 1.0);
template <SMCMode Mode>
class SMC {
 public:
  float C;
  float K;
  float J;
  float epsilon;
  float error_eps;
  float u_max;
  float angle{0.f};
  float ang_vel{0.f};
  float dt{1.f};

  float target = 0.f;
  float u = 0.f;

  // TSMC 专属
  float q;
  float p;

  // ISMC 专属
  float C1, C2;
  float z_integral_ = 0.f; // 积分累加项

  SMC() = default;

  // ================= 初始化函数 =================
  void SMCInit(float C, float K, float J, float target, float epsilon, float error_eps, float u_max,  float dt) {
    this->C = C; this->K = K; this->J = J;
    this->target = target; this->epsilon = epsilon; this->error_eps = error_eps;
    this->u_max = u_max;  this->dt = dt;
  }

  void TSMCInit(float C, float K,  float J, float q, float p, float target, float epsilon, float error_eps, float u_max, float dt) {
    this->C = C; this->K = K; this->q = q; this->p = p; this->J = J;
    this->target = target; this->epsilon = epsilon; this->error_eps = error_eps;
    this->u_max = u_max; this->dt = dt;
  }

  void ISMCInit(float C1, float C2, float K, float J, float target, float epsilon, float error_eps, float u_max, float dt) {
    this->C1 = C1; this->C2 = C2; this->K = K; this->J = J;
    this->target = target; this->epsilon = epsilon; this->error_eps = error_eps;
    this->u_max = u_max; this->dt = dt;
  }

  // ================= 核心更新函数 =================
  void Update(float target_pos, float angle_now, float angle_vel) {
    target = target_pos;

    if (reset_pending_) {
      Reset(angle_now);
      reset_pending_ = false;
    }

    angle = angle_now;
    ang_vel = angle_vel;

    float target_delta = target - target_last_;

    if (is_circular_) {
      error_ = rm::modules::Wrap(angle - target, -circular_cycle_ / 2, circular_cycle_ / 2);
      target_delta = rm::modules::Wrap(target - target_last_, -circular_cycle_ / 2, circular_cycle_ / 2) / dt;
    } else {
      error_ = angle - target;
      target_delta /= dt;
    }

    target_ddot_ = (target_delta - target_dot_) / dt;
    target_dot_ = target_delta;
    target_last_ = target;
    float error_dot = ang_vel - target_dot_;

    // 强制死区
    // if (fabs(error_) < error_eps) {
    //   u = 0;
    //   return;
    // }

    if constexpr (Mode == SMCMode::kSMC) {  // 纯线性滑模
      s_ = error_dot + C * error_;
      ds_ = -epsilon * std::tanh(s_) - K * s_;
      u = J * (target_ddot_ - C * error_dot + ds_);
    }
    else if constexpr (Mode == SMCMode::kTSMC) {  // 终端滑膜
      const float qp = q / p;
      const float e_qp = (error_ < 0) ? -std::pow(std::abs(error_), qp) : std::pow(std::abs(error_), qp);

      s_ = error_dot + C * e_qp;
      ds_ = -epsilon * std::tanh(s_) - K * s_;
      u = J * (target_ddot_ + ds_ - C * qp * error_dot * e_qp / (error_));

      // // 近端切换线性区
      // if (std::abs(error_) < 1.0f) {
      //   s_ = error_dot + C * error_;
      //   u = J * (target_ddot_ - C * error_dot - epsilon * std::tanh(s_) - K * s_);
      // }
    }
    else if constexpr (Mode == SMCMode::kISMC) {   // 积分滑膜
      const float sigma = error_dot + C1 * error_;

      if (std::abs(u) < u_max * 0.9f) {
        z_integral_ += error_ * dt;
      }

      s_ = sigma + C2 * z_integral_;
      ds_ = -epsilon * std::tanh(s_) - K * s_;
      u = J * (target_ddot_ - C1 * error_dot - C2 * error_ + ds_);
    }

    // 统一限幅
    if (u > u_max) u = u_max;
    if (u < -u_max) u = -u_max;
  }

  void Reset(float angle_now) {
    angle = angle_now;
    ang_vel = 0.0f;

    target_last_ = target;
    target_dot_ = 0.0f;
    target_ddot_ = 0.0f;

    error_ = 0.0f;
    error_last_ = 0.0f;
    s_ = 0.0f;
    ds_ = 0.0f;
    u = 0.0f;
  }

  SMC &SetCircularCycle(const float cycle, const bool is_enabled = true) {
    is_circular_ = is_enabled;
    circular_cycle_ = cycle;
    return *this;
  };

  // Getter
  float GetOutput() const { return u; }
  float GetError() const { return error_; }
  float GetSlidingSurface() const { return s_; }

 private:
  bool is_circular_{false};
  bool reset_pending_{true};

  float circular_cycle_{std::numeric_limits<float>::infinity()};

  float error_ = 0.f;
  float error_last_ = 0.f;
  float target_dot_ = 0.f;
  float target_ddot_ = 0.f;
  float target_last_ = 0.f;

  float s_ = 0.f;
  float ds_ = 0.f;

  static float Sat(float y) {
    if (fabs(y) <= 1)
      return y;
    else
      return Signal(y);
  }

  static int8_t Signal(float y) {
    if (y > 0)
      return 1;
    else if (y == 0)
      return 0;
    else
      return -1;
  }
};

// extern SMC YawSMC;

// 滤波器
// class TrackingDifferentiator {
// public:
//   float v1 = 0.0f;   // 跟踪信号 (平滑后的目标)
//   float v2 = 0.0f;   // 微分信号 (平滑后的目标速度)
//   float r = 1000.0f; // 跟踪速度因子 (越大跟得越快，但也越抖)
//   float h = 0.001f;  // 采样周期 dt
//
//   void Update(float input) {
//     // 计算 fhan (最速控制综合函数)
//     float d = r * h;
//     float d0 = d * h;
//     float x = v1 - input + h * v2;
//     float y = x + h * v2;
//     float a0 = std::sqrt(d * d + 8.0f * r * std::abs(y));
//
//     float a;
//     if (std::abs(y) <= d0) {
//       a = x + h * y; // 线性区
//     } else {
//       float sign_y = (y > 0) ? 1.0f : (y < 0 ? -1.0f : 0.0f);
//       a = v2 + (a0 - d) * 0.5f * sign_y;
//     }
//
//     float sign_a;
//     if (std::abs(a) <= d) {
//       sign_a = (a > 0) ? 1.0f : (a < 0 ? -1.0f : 0.0f);
//     } else {
//       sign_a = (a > 0) ? 1.0f : -1.0f;
//     }
//
//     // 更新状态
//     v1 += h * v2;
//     v2 += h * (-r * (a > d ? 1.0f : (a < -d ? -1.0f : a / d)) ); // 限幅
//   }
// };
//
// class ABG {
// public:
//   // 内部状态
//   float p = 0.0f; // 估算位置
//   float v = 0.0f; // 估算速度
//   float a = 0.0f; // 估算加速度
//
//   // 调参因子
//   float alpha = 0.5f;
//   float beta = 0.2f;
//   float gamma = 0.05f;
//
//   // 初始化（极其重要，避免启动时暴力收敛）
//   void Init(float init_pos) {
//     p = init_pos;
//     v = 0.0f;
//     a = 0.0f;
//   }
//
//   void Update(float measured_pos, float dt) {
//     if (dt <= 0.0f) return;
//
//     // 1. 物理模型预测
//     float pred_p = p + v * dt + 0.5f * a * dt * dt;
//     float pred_v = v + a * dt;
//
//     // 2. 计算残差（传感器测到的 与 预测的 差值）
//     float residual = measured_pos - pred_p;
//
//     // 3. 增益修正（更新内部状态）
//     p = pred_p + alpha * residual;
//     v = pred_v + (beta / dt) * residual;
//     a = a + (2.0f * gamma / (dt * dt)) * residual;
//   }
// };
//
// class Biquad {
// public:
//   // b0, b1, b2: 前馈系数 (分子)
//   // a1, a2: 反馈系数 (分母，a0 通常为 1，所以不存)
//   float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f;
//   float a1 = 0.0f, a2 = 0.0f;
//
//   // 内部状态变量
//   float x1 = 0.0f, x2 = 0.0f; // 过去的输入
//   float y1 = 0.0f, y2 = 0.0f; // 过去的输出
//
//   float out = 0.0f;
//
//   // 初始化系数
//   void SetCoeffs(float _b0, float _b1, float _b2, float _a1, float _a2) {
//     b0 = _b0; b1 = _b1; b2 = _b2;
//     a1 = _a1; a2 = _a2;
//   }
//
//   // 重置滤波器状态（防止断电重启时跳变）
//   void Reset(float current_val = 0.0f) {
//     x1 = current_val; x2 = current_val;
//     y1 = current_val; y2 = current_val;
//   }
//
//   // 每个周期调用一次，输入原始值，返回滤波后的值
//   float Update(float x) {
//     // 标准的二阶 IIR 差分方程 (Direct Form I)
//     out = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
//
//     // 更新状态
//     x2 = x1;
//     x1 = x;
//     y2 = y1;
//     y1 = out;
//
//     return out;
//   }
// };