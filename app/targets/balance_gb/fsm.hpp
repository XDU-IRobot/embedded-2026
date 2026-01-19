
#pragma once
#include <librm.hpp>
#include "librm/core/typedefs.hpp"

using namespace rm;

class Fsm {
 public:
  enum class State {
    kNoForce,  // 无力模式
    kTest,     // 测试
    kShoot,    // 使能发射机构

  };
  void Transit(State new_mode);
  void Update_State();
  void Update_Control();
  void Update_500HZ();
  void Update_250HZ();
  void Update_100HZ();
  void Update_25HZ();
  void Update_10HZ();

  State mode() const { return mode_; }

 private:
  State mode_{State::kNoForce};

  i16 init_count_{0};
};