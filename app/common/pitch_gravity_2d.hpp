#pragma once

#include <cmath>

// 单轴重力补偿 + 自动四点静态标定。角度 rad，速度 rad/s，力矩 N*m。
// 无动态内存，不依赖 PID、SMC、HAL 或电机驱动。
// 初始化 SetAngleLimits；正常使用 Calculate；标定时逐拍 Calibrate；退出时 CancelCalibration。
class PitchGravity {
 public:
  enum class CalibrationState { kIdle, kRunning, kDone, kFailed };
  enum class CalibrationError { kNone, kNotConfigured, kInvalidFeedback, kTimeout, kFitRejected };
  struct CalibrationStep {
    float target;         // 本拍的 pitch 位置目标，交给原有闭环
    bool reset_feedback;  // 进入标定或拟合成功时，清除原控制器的目标差分历史
    bool stop;            // 标定失败，调用方应输出零力矩
  };

  static float Calculate(float gain_nm, float phase_rad, float angle_rad) {
    if (!std::isfinite(angle_rad)) return 0.0f;
    const float torque = gain_nm * std::cos(angle_rad + phase_rad);
    return torque;
  }

  // a*cos(angle) + b*sin(angle)
  // 转换为 gain_nm*cos(angle + phase_rad)
  static void ABToGainPhase(float a, float b, float &gain_nm, float &phase_rad) {
    gain_nm = std::hypot(a, b);
    phase_rad = gain_nm > 0.0f ? std::atan2(-b, a) : 0.0f;
  }

  // 使用与控制反馈相同坐标系的软件角度限位。四个采样点在范围内侧 20%~80%。
  // 配置成功会复位标定进度，保留已有补偿；运行中拒绝更改限位。
  bool SetAngleLimits(float lower_rad, float upper_rad) {
    if (progress_.state == CalibrationState::kRunning || !std::isfinite(lower_rad) || !std::isfinite(upper_rad) ||
        !(upper_rad > lower_rad))
      return false;
    lower_rad_ = lower_rad;
    upper_rad_ = upper_rad;
    limits_set_ = true;
    CancelCalibration();
    return true;
  }

  // 每个控制周期调用一次；第一次自动开始，完成或失败后不会自行重试。
  // applied_torque_nm 是与当前反馈对应的、已经施加的最终总力矩，包含已有补偿。
  // controller_ok/saturated 由调用方按当前反馈控制器与最终输出限幅判断。
  CalibrationStep Calibrate(float angle_rad, float speed_rad_s, float applied_torque_nm, float dt,
                            bool controller_ok = true, bool saturated = false) {
    if (progress_.state == CalibrationState::kFailed) return Fail(progress_.error);
    if (!limits_set_ || !(limit_nm_ > 0.0f)) return Fail(CalibrationError::kNotConfigured);
    if (!controller_ok || !std::isfinite(angle_rad) || !std::isfinite(speed_rad_s) ||
        !std::isfinite(applied_torque_nm) || !(dt > 0.0f && dt <= 0.02f) || angle_rad < lower_rad_ - 0.03f ||
        angle_rad > upper_rad_ + 0.03f) {
      return Fail(CalibrationError::kInvalidFeedback);
    }

    bool reset_feedback = false;
    if (progress_.state == CalibrationState::kIdle) {
      progress_ = {};
      progress_.state = CalibrationState::kRunning;
      progress_.reference = angle_rad;
      ClearSamples();
      reset_feedback = true;
    }
    if (progress_.state == CalibrationState::kDone) return {progress_.reference, false, false};

    progress_.point_time += dt;
    if (progress_.point_time >= kPointTimeout) return Fail(CalibrationError::kTimeout);

    const float fraction = 0.2f + 0.6f * static_cast<float>(progress_.point) / (kPointCount - 1);
    const float goal = lower_rad_ + fraction * (upper_rad_ - lower_rad_);
    const float difference = goal - progress_.reference;
    const float max_step = kMoveSpeed * dt;
    progress_.reference += difference > max_step ? max_step : (difference < -max_step ? -max_step : difference);

    const bool steady = std::fabs(progress_.reference - goal) < 1e-5f && std::fabs(speed_rad_s) < kStableSpeed &&
                        std::fabs(angle_rad - goal) < kPositionTolerance && !saturated;
    if (!steady) {
      ResetWindow();
    } else {
      progress_.settle_time += dt;
      if (progress_.settle_time >= kSettleTime) {
        if (progress_.samples == 0) progress_.torque_min = progress_.torque_max = applied_torque_nm;
        if (applied_torque_nm < progress_.torque_min) progress_.torque_min = applied_torque_nm;
        if (applied_torque_nm > progress_.torque_max) progress_.torque_max = applied_torque_nm;
        if (progress_.torque_max - progress_.torque_min > kTorqueVariation) {
          ResetWindow();
        } else {
          progress_.angle_sum += angle_rad;
          progress_.torque_sum += applied_torque_nm;
          ++progress_.samples;
          progress_.sample_time += dt;
          if (progress_.sample_time >= kSampleTime) {
            const float mean_angle = static_cast<float>(progress_.angle_sum / progress_.samples);
            const float mean_torque = static_cast<float>(progress_.torque_sum / progress_.samples);
            if (!AddSample(mean_angle, mean_torque)) return Fail(CalibrationError::kInvalidFeedback);
            ++progress_.point;
            progress_.point_time = 0.0f;
            ResetWindow();
            if (progress_.point == kPointCount) {
              if (!Fit()) return Fail(CalibrationError::kFitRejected);
              progress_.state = CalibrationState::kDone;
              progress_.reference = angle_rad;
              reset_feedback = true;
            }
          }
        }
      }
    }
    return {progress_.reference, reset_feedback, false};
  }

  // 离开标定模式或进入无力时调用。只清进度，不清已有 a、b。
  void CancelCalibration() { progress_ = {}; }
  CalibrationState calibration_state() const { return progress_.state; }
  CalibrationError calibration_error() const { return progress_.error; }
  unsigned calibration_point() const { return progress_.point; }
  // 补偿幅值上限，不是电机总力矩上限。初始系数为 0。
  explicit PitchGravity(float limit_nm) : limit_nm_(std::isfinite(limit_nm) && limit_nm > 0.0f ? limit_nm : 0.0f) {}

  float Calculate(float angle_rad) const {
    if (!std::isfinite(angle_rad)) return 0.0f;
    const float torque = a_nm_ * std::cos(angle_rad) + b_nm_ * std::sin(angle_rad);
    return torque > limit_nm_ ? limit_nm_ : (torque < -limit_nm_ ? -limit_nm_ : torque);
  }

  // 手动设置参数，或上电加载上次保存的参数；失败保留旧参数。
  bool SetCoefficients(float a_nm, float b_nm) {
    if (!std::isfinite(a_nm) || !std::isfinite(b_nm)) return false;
    if (std::hypot(a_nm, b_nm) > limit_nm_) return false;
    a_nm_ = a_nm;
    b_nm_ = b_nm;
    return true;
  }

  // 开始新一轮标定；不清除正在使用的补偿系数。
  void ClearSamples() {
    cc_ = cs_ = ss_ = ct_ = st_ = 0.0;
    count_ = 0;
  }

  // holding_torque_nm：该姿态下最终施加的总保持力矩，已经包含已有补偿。
  // 可以输入静止时的逐拍数据，也可以输入每个姿态的平均角度和平均力矩。
  bool AddSample(float angle_rad, float holding_torque_nm) {
    if (!std::isfinite(angle_rad) || !std::isfinite(holding_torque_nm)) return false;
    const double c = std::cos(static_cast<double>(angle_rad));
    const double s = std::sin(static_cast<double>(angle_rad));
    cc_ += c * c;
    cs_ += c * s;
    ss_ += s * s;
    ct_ += c * holding_torque_nm;
    st_ += s * holding_torque_nm;
    ++count_;
    return true;
  }

  // 收集多个不同姿态的数据后调用一次。成功才更新 a、b。
  // false：数据不足、角度太集中，或解出的补偿幅值超过上限。
  bool Fit() {
    if (count_ < 3) return false;
    const double det = cc_ * ss_ - cs_ * cs_;
    const double trace = cc_ + ss_;
    if (det <= 0.005 * trace * trace) return false;
    const float a = static_cast<float>((ct_ * ss_ - st_ * cs_) / det);
    const float b = static_cast<float>((st_ * cc_ - ct_ * cs_) / det);
    return SetCoefficients(a, b);
  }

  float cos_coefficient() const { return a_nm_; }
  float sin_coefficient() const { return b_nm_; }
  unsigned sample_count() const { return count_; }

 private:
  // 扫描和稳态判断的默认参数集中在类内部，需要时修改这里。
  static constexpr unsigned kPointCount = 4;
  static constexpr float kMoveSpeed = 0.08f;
  static constexpr float kStableSpeed = 0.025f;
  static constexpr float kPositionTolerance = 0.10f;
  static constexpr float kSettleTime = 0.6f;
  static constexpr float kSampleTime = 1.0f;
  static constexpr float kPointTimeout = 15.0f;
  static constexpr float kTorqueVariation = 0.30f;

  struct CalibrationProgress {
    CalibrationState state = CalibrationState::kIdle;
    CalibrationError error = CalibrationError::kNone;
    unsigned point = 0, samples = 0;
    float reference = 0.0f, point_time = 0.0f;
    float settle_time = 0.0f, sample_time = 0.0f;
    float torque_min = 0.0f, torque_max = 0.0f;
    double angle_sum = 0.0, torque_sum = 0.0;
  } progress_;

  void ResetWindow() {
    progress_.settle_time = progress_.sample_time = 0.0f;
    progress_.samples = 0;
    progress_.angle_sum = progress_.torque_sum = 0.0;
  }

  CalibrationStep Fail(CalibrationError error) {
    progress_.state = CalibrationState::kFailed;
    progress_.error = error;
    return {progress_.reference, false, true};
  }

  bool limits_set_ = false;
  float lower_rad_ = 0.0f, upper_rad_ = 0.0f;
  float limit_nm_;
  float a_nm_ = 0.0f;
  float b_nm_ = 0.0f;
  double cc_ = 0.0, cs_ = 0.0, ss_ = 0.0;
  double ct_ = 0.0, st_ = 0.0;
  unsigned count_ = 0;
};
