//
// Created by 34236 on 2025/11/24.
#include <librm.hpp>
#include "StateMachine.h"

#ifndef BOARDC_MAIN_H
#define BOARDC_MAIN_H
 class DartRack {;
 public:
  rm::hal::Can *can1{nullptr};  //< CAN 总线接口
  rm::hal::Serial *dbus{nullptr};//< 遥控器串口接口
  rm::device::DR16 *rc{nullptr};//< 遥控器
  dart_rack_state::DartState dart_state;//状态机
  void Init() {
    can1 = new rm::hal::Can{hcan1};
    dbus = new rm::hal::Serial{huart3, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
    rc = new rm::device::DR16{*dbus};
    can1->SetFilter(0, 0);
    can1->Begin();
  }
} ;
class main {};

#endif  // BOARDC_MAIN_H
