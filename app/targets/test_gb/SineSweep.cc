// SineSweep.cc
#include "SineSweep.hpp"
#include <cmath>
#include <algorithm>
SineSweep::SineSweep(double amp_min, double amp_max, double f_start, double f_end, double duration, double sample_rate)
    : amp_min_(amp_min),
      amp_max_(amp_max),
      f0_(f_start),
      f1_(f_end),
      duration_(duration),
      fs_(sample_rate),
      t_(0.0),
      phase_(0.0),
      finished_(false) {
  dt_ = (fs_ > 0.0) ? (1.0 / fs_) : 0.0;
  finished_ = (duration_ <= 0.0) || (fs_ <= 0.0);
}

double SineSweep::Next() {
  if (finished_) return 0.0;
  double alpha = (duration_ > 0.0) ? (t_ / duration_) : 1.0;
  if (alpha > 1.0) alpha = 1.0;
  double finst = f0_ + (f1_ - f0_) * alpha;
  double amp = amp_min_ + (amp_max_ - amp_min_) * alpha;
  phase_ += 2.0 * M_PI * finst * dt_;
  if (phase_ > 1e6 || phase_ < -1e6) phase_ = std::fmod(phase_, 2.0 * M_PI);
  double out = amp * std::sin(phase_);
  t_ += dt_;
  if (t_ >= duration_) finished_ = true;
  return out;
}

bool SineSweep::Finished() const { return finished_; }

void SineSweep::Reset() {
  t_ = 0.0;
  phase_ = 0.0;
  finished_ = (duration_ <= 0.0) || (fs_ <= 0.0);
}
MultiFreqSine::MultiFreqSine(const std::vector<double> &freqs, int cycles_per_freq, double amplitude,
                             double sample_rate)
    : amplitude_(amplitude), fs_(sample_rate), phase_(0.0), idx_(0), finished_(false) {
  items_.clear();
  if (freqs.empty() || cycles_per_freq <= 0 || fs_ <= 0.0) {
    finished_ = true;
    return;
  }

  for (double f : freqs) {
    if (f <= 0.0) continue;
    // 每周期的采样点数取整（>=1）
    int spc = std::max(1, static_cast<int>(std::lround(fs_ / f)));
    double actual_f = fs_ / static_cast<double>(spc);
    Item it;
    it.freq = actual_f;
    it.samples_per_cycle = spc;
    it.samples_total = spc * cycles_per_freq;
    it.samples_elapsed = 0;
    items_.push_back(it);
  }

  if (items_.empty()) finished_ = true;
}

double MultiFreqSine::Next() {
  if (finished_ || items_.empty()) return 0.0;
  Item &it = items_[idx_];
  // 在每个频点开始时重置相位，保证从零相位开始
  if (it.samples_elapsed == 0) phase_ = 0.0;

  double out = amplitude_ * std::sin(phase_);
  // 相位增量
  phase_ += 2.0 * M_PI * it.freq / fs_;
  // 防止相位无限增长
  if (phase_ > 1e6 || phase_ < -1e6) phase_ = std::fmod(phase_, 2.0 * M_PI);

  it.samples_elapsed++;
  if (it.samples_elapsed >= it.samples_total) {
    idx_++;
    if (idx_ >= items_.size()) {
      finished_ = true;
    } else {
      // 下一频点从零相位开始，phase_ 已在下一次 Next() 调用时被重置
      phase_ = 0.0;
    }
  }
  return out;
}

bool MultiFreqSine::Finished() const { return finished_; }

void MultiFreqSine::Reset() {
  phase_ = 0.0;
  idx_ = 0;
  finished_ = items_.empty();
  for (auto &it : items_) it.samples_elapsed = 0;
}

// 生成 MATLAB 风格的频率序列： [1:0.5:22, 24:2:40, 50:10:120, 200,250,333,500]
std::vector<double> MultiFreqSine::DefaultFrequencies() {
  std::vector<double> v;
  // 1:0.5:22
  for (double f = 1.0; f <= 22.0001; f += 0.5) v.push_back(f);
  // 24:2:40
  for (double f = 24.0; f <= 40.0001; f += 2.0) v.push_back(f);
  // 50:10:120
  for (double f = 50.0; f <= 120.0001; f += 10.0) v.push_back(f);
  // append discrete values
  // v.push_back(200.0);
  // v.push_back(250.0);
  // v.push_back(333.0);
  // v.push_back(500.0);
  return v;
}