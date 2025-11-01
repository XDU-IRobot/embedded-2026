#include <librm.hpp>

#include "can.h"
#include "usart.h"
#include "spi.h"

#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "timer_task.hpp"
#include "device_manager.hpp"
#include "controllers/gimbal_double_yaw.hpp"
#include "controllers/shoot_3fric.hpp"

struct GlobalWarehouse {
  AsyncBuzzer *buzzer{nullptr};  ///< 蜂鸣器
  LED *led{nullptr};             ///< RGB LED灯

  // 硬件接口 //
  rm::hal::Can *can1{nullptr}, *can2{nullptr};  ///< CAN 总线接口
  rm::hal::Serial *dbus{nullptr};               ///< 遥控器串口接口

  // 设备 //
  DeviceManager<10> device_manager;           ///< 设备管理器，维护所有设备在线状态
  rm::device::DR16 *rc{nullptr};              ///< 遥控器
  rm::device::GM6020 *up_yaw_motor{nullptr};  ///< 云台 Yaw 上电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *down_yaw_motor{nullptr};  ///< 云台 Yaw 下电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr};     ///< 云台 Pitch 电机
  rm::device::BMI088 *imu{nullptr};

  // 控制器 //
  GimbalDoubleYaw gimbal_controller;  ///< 二轴双 Yaw 云台控制器
  Shoot3Fric shoot_controller{9};     ///< 三摩擦轮发射机构控制器，9发拨盘
  rm::modules::MahonyAhrs ahrs{1000.0f};

  // 变量 //
  int DM_enable_flag = 0;
  // const float yaw_gyro_bias = 0.0015f; // 偏航角（角度值）的陀螺仪偏移量

  void Init();
} *globals;

void MainLoop() {
  globals->imu->Update();
  globals->ahrs.Update(  //
      rm::modules::ImuData6Dof{globals->imu->gyro_y(), globals->imu->gyro_z(), globals->imu->gyro_x(),
                               globals->imu->accel_y(), globals->imu->accel_z(), globals->imu->accel_x()});
  if (globals->DM_enable_flag == 0) {  // 使达妙电机使能
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    // globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    globals->DM_enable_flag = 1;
  } else {
    globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
  }
}

extern "C" [[noreturn]] void AppMain(void) {
  globals = new GlobalWarehouse;
  globals->Init();

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_1000hz.SetPrescalerAndPeriod(84 - 1, 1000 - 1);  // 84MHz / 84 / 1000 = 1kHz
  mainloop_1000hz.Start();

  globals->buzzer->Beep(2, 40);
  (*globals->led)(0xff00ff00);

  for (;;) {
    __WFI();
  }
}

void GlobalWarehouse::Init() {
  buzzer = new AsyncBuzzer;
  led = new LED;

  can1 = new rm::hal::Can{hcan1};
  can2 = new rm::hal::Can{hcan2};
  dbus = new rm::hal::Serial{huart3, 18, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};

  rc = new rm::device::DR16{*dbus};
  up_yaw_motor = new rm::device::GM6020{*can1, 5};
  down_yaw_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
      *can2, {0x05, 0x04, 12.5f, 30.0f, 10.0f, std::make_pair(0.0f, 500.0f), std::make_pair(0.0f, 5.0f)}};
  pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
      *can1, {0x03, 0x02, 12.5f, 30.0f, 10.0f, std::make_pair(0.0f, 500.0f), std::make_pair(0.0f, 5.0f)}};
  imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};

  device_manager << rc << up_yaw_motor << down_yaw_motor << pitch_motor;

  can1->SetFilter(0, 0);
  can1->Begin();
  can2->SetFilter(0, 0);
  can2->Begin();
  rc->Begin();
  buzzer->Init();
  led->Init();
}