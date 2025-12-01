//
// Created by 34236 on 2025/11/25.
//
#include "dart_core.hpp"
#include "can.h"
#include "usart.h"
DartRack *dart_rack;

void DartRack::Init() {
  // 硬件接口初始化
  dart_rack->can1 = new rm::hal::Can{hcan1};
  dart_rack->state = new DartState;
  dart_rack->dbus = new rm::hal::Serial{huart3, 18, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
  dart_rack->rc = new rm::device::DR16{*dart_rack->dbus};
  dart_rack->rc->Begin();
  // 电机初始化
  dart_rack->load_motor_l = new rm::device::M3508{*dart_rack->can1, 3};
  dart_rack->load_motor_r = new rm::device::M3508{*dart_rack->can1, 4};
  dart_rack->trigger_motor = new rm::device::M2006{*dart_rack->can1, 5};
  dart_rack->trigger_motor_force = new rm::device::M2006{*dart_rack->can1, 6};
  dart_rack->yaw_motor = new rm::device::M2006{*dart_rack->can1, 7};
  dart_rack->can1->SetFilter(0, 0);
  dart_rack->can1->Begin();

  // PID初始化
  dart_rack->load_motor_l_speed_pid = new rm::modules::PID(5, 0, 0, 10000, 0);
  dart_rack->load_motor_r_speed_pid = new rm::modules::PID(5, 0, 0, 10000, 0);
  dart_rack->trigger_motor_speed_pid = new rm::modules::PID(5, 0, 0, 10000, 0);
  dart_rack->trigger_motor_force_pid = new rm::modules::PID(5, 0, 0, 10000, 0);
  dart_rack->yaw_motor_speed_pid = new rm::modules::PID(5, 0, 0, 10000, 0);

  // 编码器初始化
  dart_rack->yaw_encoder = new rm::device::ME02{*dart_rack->can1, 0x50, 0.1f};
}