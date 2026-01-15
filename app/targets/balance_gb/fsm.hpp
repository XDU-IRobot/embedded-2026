
#pragma once
#include <librm.hpp>
#include "librm/core/typedefs.hpp"

using namespace rm;

class Fsm {
 public:
  enum class State {
    kNoForce,  // 无力模式
    kInit,     // 云台初始化
    kTest,     // 测试

  };
  void Transit(State new_mode);
  void Update();

  State mode() const { return mode_; }

 private:
  State mode_{State::kNoForce};

  i16 init_count_{0};
};