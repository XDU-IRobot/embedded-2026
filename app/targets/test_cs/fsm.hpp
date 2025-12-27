
#pragma once

class Fsm {
public:
  enum class State {
    kNoForce,  // 无力模式
    kManual,   // 遥控器控制
    kAuto,     // 执行上位机下发的速度命令
  };
  void Transit(State new_mode);
  void Update();

  State mode() const { return mode_; }

private:
  State mode_{State::kNoForce};
};