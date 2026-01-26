#pragma once

#include "librm.hpp"

using namespace rm;

class FirstOrderFilter {
public:
  FirstOrderFilter(f32 frame_period, f32 num) {
    frame_period_ = frame_period;
    num_ = num;
    input_ = 0;
    out_ = 0;
  }

  [[nodiscard]] f32 value() { return out_; }

  void Update(f32 input) {
    input_ = input;
    out_ = num_ / (num_ + frame_period_) * out_ + frame_period_ / (num_ + frame_period_) * input;
  }

private:
  f32 input_;
  f32 out_;
  f32 num_;
  f32 frame_period_;
};
