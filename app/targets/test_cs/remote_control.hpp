#pragma once
#ifndef REMOTE_CONTROL_HPP
#define REMOTE_CONTROL_HPP

#include <librm.hpp>

#include "device_manager.hpp"
#include "usart.h"

class RemoteControl {
 public:
  rm::hal::Serial *dbus{nullptr};  ///< 遥控器串口接口
  rm::device::DR16 *rc{nullptr};   ///< 遥控器
  DeviceManager<1> device_rc;      ///< 设备管理器，维护所有设备在线状态

  void RemoteInit() {
    dbus = new rm::hal::Serial{huart3, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
    rc = new rm::device::DR16{*dbus};
    device_rc << rc;
    rc->Begin();
  }
};

#endif  // REMOTE_CONTROL_HPP