//
// Created by 34236 on 2026/1/16.
//

#ifndef BOARDC_SINESWEEP_HPP
#define BOARDC_SINESWEEP_HPP
#include <vector>
class SineSweep {
public:
  // amp_min, amp_max: 振幅范围（输出大约在 [-amp, +amp]）
  // f_start, f_end: 起始和结束频率 (Hz)
  // duration: 扫频总时长 (秒)
  // sample_rate: 采样率 (Hz)
  SineSweep(double amp_min, double amp_max, double f_start, double f_end, double duration, double sample_rate);

  double Next();    // 返回下一个样点
  bool Finished() const;
  void Reset();

private:
  double amp_min_, amp_max_;
  double f0_, f1_;
  double duration_, fs_, dt_;
  double t_;
  double phase_;
  bool finished_;
};

class MultiFreqSine {
public:
  // freqs: 目标频率列表 (Hz)
  // cycles_per_freq: 每个频率持续的周期数（整数），默认 20
  // amplitude: 输出振幅（正弦幅值），默认 1.0
  // sample_rate: 采样率 (Hz)，默认 1000
  MultiFreqSine(const std::vector<double> &freqs,
                int cycles_per_freq = 20,
                double amplitude = 1.0,
                double sample_rate = 1000.0);

  double Next();       // 返回下一个采样点
  bool Finished() const;
  void Reset();

  // 生成与 MATLAB: F = ([1:0.5:22, 24:2:40, 50:10:120,200,250,333,500]); 等价的频率向量
  static std::vector<double> DefaultFrequencies();

private:
  struct Item {
    double freq;        // 实际使用的频率 (Hz) = fs / samples_per_cycle
    int samples_per_cycle;
    int samples_total;  // samples_per_cycle * cycles_per_freq
    int samples_elapsed;
  };

  std::vector<Item> items_;
  double amplitude_;
  double fs_;
  double phase_;
  size_t idx_;
  bool finished_;
};
#endif  // BOARDC_SINESWEEP_HPP
