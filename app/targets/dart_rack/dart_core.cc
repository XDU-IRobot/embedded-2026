
#include "can.h"
#include "usart.h"

#include "dart_core.hpp"

DartRack *dart_rack;
float yaw;
void DartRack::Init() {
  // PID初始化
  load_motor_l_speed_pid_.SetKp(5).SetKi(1).SetKd(0).SetMaxOut(10000).SetMaxIout(0);
  load_motor_r_speed_pid_.SetKp(5).SetKi(1).SetKd(0).SetMaxOut(10000).SetMaxIout(0);
  trigger_motor_speed_pid_.SetKp(5).SetKi(0).SetKd(0).SetMaxOut(10000).SetMaxIout(0);
  trigger_motor_force_pid_.SetKp(-5).SetKi(0).SetKd(0).SetMaxOut(10000).SetMaxIout(0);
  yaw_motor_speed_pid_.SetKp(5).SetKi(0).SetKd(0).SetMaxOut(16384).SetMaxIout(0);

  // 硬件接口初始化
  can1_ = new rm::hal::Can{hcan1};
  dbus_ = new rm::hal::Serial{huart3, 18, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
  rc_ = new rm::device::DR16{*dbus_};
  rc_->Begin();
  // 电机初始化
  load_motor_l_ = new rm::device::M3508{*can1_, 3};
  load_motor_r_ = new rm::device::M3508{*can1_, 4};
  trigger_motor_ = new rm::device::M2006{*can1_, 5};
  trigger_motor_force_ = new rm::device::M2006{*can1_, 6};
  yaw_motor_ = new rm::device::M2006{*can1_, 7};

  vision_data_ = new USBVisionReceive_SCM_t;

  // 编码器初始化
  yaw_encoder_ = new rm::device::JyMe02Can{*can1_, 0x50, 1.0f};

  can1_->SetFilter(0, 0);
  can1_->Begin();
}

// 数据更新
void DartRack::Update() {
  load_motor_l_odometer_.Update(load_motor_l_->encoder(), load_motor_l_->current());
  load_motor_r_odometer_.Update(load_motor_r_->encoder(), load_motor_r_->current());
  trigger_motor_odometer_.Update(trigger_motor_->encoder(), trigger_motor_->current());
  trigger_motor_force_odometer_.Update(trigger_motor_force_->encoder(), trigger_motor_force_->current());

}