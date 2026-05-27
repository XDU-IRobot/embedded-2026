#include "dart_core.hpp"

#include "can.h"
#include "usart.h"

#include "dart_core.hpp"

DartRack *dart_rack;
float yaw;

// 硬件未使用 74HC126 收发器，提供空操作 GPIO 引脚
static NopPin nop_tx_en;
static NopPin nop_rx_en;

void DartRack::Init() {
  // PID初始化
  load_motor_l_speed_pid_.SetKp(15).SetKi(0).SetKd(0).SetMaxOut(10000).SetMaxIout(20);
  load_motor_r_speed_pid_.SetKp(15).SetKi(0).SetKd(0).SetMaxOut(10000).SetMaxIout(20);
  trigger_motor_speed_pid_.SetKp(5).SetKi(0).SetKd(0).SetMaxOut(10000).SetMaxIout(0);
  trigger_motor_force_pid_.SetKp(-20).SetKi(0).SetKd(0).SetMaxOut(15000).SetMaxIout(0);
  add_motor_speed_pid_.SetKp(5).SetKi(0).SetMaxOut(10000).SetMaxIout(0);
  yaw_motor_speed_pid_.SetKp(20).SetKi(0).SetKd(0).SetMaxOut(8000).SetMaxIout(0);
  yaw_motor_angle_pid_.SetKp(30).SetKi(0).SetKd(0).SetMaxOut(8000).SetMaxIout(0);

  referee_data_buffer = new rm::device::Referee<rm::device::RefereeRevision::kNewV120>;

  // 硬件接口初始化
  can1_ = new rm::hal::Can{hcan1};
  can2_ = new rm::hal::Can{hcan2};

  // UART1 用于 DR16 遥控器接收
  dbus_ = new rm::hal::Serial<128>{huart1, true,true};
  rc_ = new rm::device::DR16{*dbus_};
  rc_->Begin();

  // UART2 用于 HiwonderServo 串口舵机控制（总线舵机，ID 1 和 2 共享同一条 UART）
  servo_uart = new rm::hal::Serial<128>{huart2, true, true};
  add_servo_1_ = new rm::device::HiWonderServo{*servo_uart, nop_tx_en, nop_rx_en, 1};
  add_servo_2_ = new rm::device::HiWonderServo{*servo_uart, nop_tx_en, nop_rx_en, 2};
  servo_uart->Start();

  // UART3 用于 RxReferee 裁判系统
  referee_uart = new rm::hal::Serial<128>{huart3, true,true};
  rx_referee = new rm::device::RxReferee{*referee_uart};
  rx_referee->Begin();

  // 电机初始化
  load_motor_l_ = new rm::device::M3508{*can1_, 3};
  load_motor_r_ = new rm::device::M3508{*can1_, 4};
  trigger_motor_ = new rm::device::M2006{*can1_, 5};
  add_motor_ = new rm::device::M2006{*can1_, 6};
  yaw_motor_ = new rm::device::M2006{*can1_, 7};
  trigger_motor_force_ = new rm::device::M2006{*can1_, 8};

  vision_data_ = new USBVisionReceive_SCM_t;
  vision_data_->Yaw = 0.0f;

  // 编码器初始化
  yaw_encoder_ = new rm::device::JyMe02Can{*can1_, 0x50, 1.0f};

  can1_->SetFilter(0, 0);
  can1_->Begin();

  // 达妙电机配置
  rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit> dm_settings = {
      .master_id = 0x11,                              // 取决于达妙上位机里设置的反馈ID
      .slave_id = 0x01,                               // 取决于达妙上位机里设置的目标ID
      .p_max = 3.0f,                                  // 最大位置范围
      .v_max = 30.0f,                                 // 最大速度
      .t_max = 10.0f,                                 // 最大扭矩
      .kp_range = std::make_pair(0.0f, 500.0f),        // Kp取值范围
      .kd_range = std::make_pair(0.0f, 5.0f)          // Kd取值范围
  };

  // 实例化达妙电机 (内部会将 dm_settings.master_id 取出并传给 CanDevice 基类)
  dm_motor_ = new rm::device::DmMotor{*can1_, dm_settings, true};
  // 上电必须使能
  dm_motor_->SendInstruction(rm::device::DmMotorInstructions::kEnable);
}

// 数据更新
void DartRack::Update() {
  load_motor_l_odometer_.Update(load_motor_l_->encoder(), load_motor_l_->current());
  load_motor_r_odometer_.Update(load_motor_r_->encoder(), load_motor_r_->current());
  trigger_motor_odometer_.Update(trigger_motor_->encoder(), trigger_motor_->current());
  trigger_motor_force_odometer_.Update(trigger_motor_force_->encoder(), trigger_motor_force_->current());
  add_motor_odometer_.Update(add_motor_->encoder(), add_motor_->current());
}
