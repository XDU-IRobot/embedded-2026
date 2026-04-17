
#pragma once
#include <librm.hpp>
#include "librm/core/typedefs.hpp"

using namespace rm;

class Fsm {
 public:
  enum class State {
    kNoForce,    // 无力模式
    kTest,       // 测试
    kShoot,      // 自瞄+打弹
    kAutoShoot,  // 自动开火
    kHigh,       // 上台阶高腿长
    kAutoFu,     //  自动打符
  };
  void Transit(State new_mode);
  void Update_State();
  void Update_Chassis_Request();
  void Update_Test();
  void Update_Control();
  void Update_500HZ();
  void Update_250HZ();
  void Update_100HZ();
  void Update_25HZ();
  void Update_10HZ();

  State mode() const { return mode_; }

  i16 init_count_{0};

  bool inited_{false};
  bool high_mode_{false};

  State mode_{State::kNoForce};

  u8 auto_mode_{1};

 private:
};