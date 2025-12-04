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

  // 编码器初始化
  dart_rack->yaw_encoder = new rm::device::ME02{*dart_rack->can1, 0x50, 1.0f};

  dart_rack->can1->SetFilter(0, 0);
  dart_rack->can1->Begin();

  // PID初始化
  dart_rack->load_motor_l_speed_pid = new rm::modules::PID(5, 0, 0, 10000, 0);
  dart_rack->load_motor_r_speed_pid = new rm::modules::PID(5, 0, 0, 10000, 0);
  dart_rack->trigger_motor_speed_pid = new rm::modules::PID(5, 0, 0, 16384, 0);
  dart_rack->trigger_motor_force_pid = new rm::modules::PID(5, 0, 0, 16384, 0);
  dart_rack->yaw_motor_speed_pid = new rm::modules::PID(5, 0, 0, 16384, 0);

  // 计圈器初始化
  dart_rack->load_motor_l_odometer = new rm::device::DjiOdometer();
  dart_rack->load_motor_r_odometer = new rm::device::DjiOdometer();
  dart_rack->trigger_motor_odometer = new rm::device::DjiOdometer();
}
// 数据更新
void DartRack::DataUpdate() {
  dart_rack->load_motor_l_odometer->Update(dart_rack->load_motor_l->encoder(), dart_rack->load_motor_l->current());
  dart_rack->load_motor_r_odometer->Update(dart_rack->load_motor_r->encoder(), dart_rack->load_motor_r->current());
  dart_rack->trigger_motor_odometer->Update(dart_rack->trigger_motor->encoder(), dart_rack->trigger_motor->current());
  dart_rack->trigger_motor_force_odometer->Update(dart_rack->trigger_motor_force->encoder(),
                                                  dart_rack->trigger_motor_force->current());
}
//   数据发送
void DartRack::DataSend() { rm::device::DjiMotor<>::SendCommand(); }