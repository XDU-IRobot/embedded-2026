
#pragma once

#include "fsm.hpp"
#include "remote_control.hpp"
#include "chassis.hpp"
#include "minipc.hpp"

struct Global {
  RemoteControl const *rc{nullptr};  ///< 遥控器object
  Chassis *chassis{nullptr};         ///< 底盘object
  MiniPC *minipc{nullptr};           ///< minipc object

  Fsm fsm{};  ///< 状态机控制行为模式
};

extern Global global;  ///< 全局变量，包含所有设备、算法等实例