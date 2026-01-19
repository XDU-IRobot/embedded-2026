
#pragma once

#include "fsm.hpp"
#include "boardc.hpp"
#include "motor.hpp"

struct Global {
 public:
  BoardC *bc{nullptr};    ///< c板object
  Motor *motor{nullptr};  ///< 电机object

  Fsm fsm{};  ///< 状态机控制行为模式
 public:
  i16 divide_count = 0;
};

extern Global global;  ///< 全局变量，包含所有设备、算法等实例