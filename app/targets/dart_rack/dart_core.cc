//
// Created by 34236 on 2025/11/25.
//
#include "dart_core.hpp"
#include "can.h"
#include "usart.h"
DartRack *dart_rack;

void DartRack::Init() {
  dart_rack->can1 = new rm::hal::Can{hcan1};
  dart_rack->state = new DartState;
  dart_rack->dbus = new rm::hal::Serial{huart3, 18, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
   dart_rack->rc = new rm::device::DR16{*dart_rack->dbus};
   dart_rack->rc->Begin();

}