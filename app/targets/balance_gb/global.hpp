
#pragma once

#include "fsm.hpp"
#include "boardc.hpp"
#include "motor.hpp"
#include "communiate.hpp"
#include "minipc.hpp"

struct Global {
 public:
  BoardC *bc{nullptr};                    ///< c板object
  Motor *motor{nullptr};                  ///< 电机object
  ChassisCommunicator *chassis_communicator;  ///<  底盘控制器object
  ChassisCommunicator * chassis_receive;   ///<接受底盘指令
  MiniPC *minipc{nullptr};                ///< minipc object

  Fsm fsm{};  ///< 状态机控制行为模式
 public:
  i16 divide_count = 0;
};

extern Global global;  ///< 全局变量，包含所有设备、算法等实例