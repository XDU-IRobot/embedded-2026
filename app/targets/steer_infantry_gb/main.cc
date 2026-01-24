#include <librm.hpp>

#include "can.h"
#include "usart.h"
#include "spi.h"

#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "timer_task.hpp"
#include "controllers/gimbal_2dof.hpp"
#include "controllers/shoot_3fric.hpp"

struct GlobalWarehouse {
  AsyncBuzzer *buzzer{nullptr};  ///< 蜂鸣器
  LED *led{nullptr};             ///< RGB LED灯

  // 硬件接口 //
  rm::hal::Can *can1{nullptr}, *can2{nullptr};  ///< CAN 总线接口
  rm::hal::Serial *dbus{nullptr};               ///< 遥控器串口接口

  // 设备 //
  rm::device::DeviceManager<10> device_manager;        ///< 设备管理器，维护所有设备在线状态
  rm::device::DR16 *rc{nullptr};           ///< 遥控器
  rm::device::GM6020 *yaw_motor{nullptr};  ///< 云台 Yaw 电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr};  ///< 云台 Pitch 电机
  rm::device::BMI088 *imu{nullptr};                                                 ///< BMI088 IMU

  // 控制器 //
  Gimbal2Dof gimbal_controller;  ///< 二轴云台控制器
  Shoot3Fric shoot_controller{
      8,
      1.f  // placeholder
  };  ///< 三摩擦轮发射机构控制器，9发拨盘
  rm::modules::MahonyAhrs ahrs{1000.f};  ///< mahony 姿态解算器，频率 1000Hz

  void Init() {
    buzzer = new AsyncBuzzer;
    led = new LED;

    can1 = new rm::hal::Can{hcan1};
    can2 = new rm::hal::Can{hcan2};
    dbus = new rm::hal::Serial{huart3, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};

    rc = new rm::device::DR16{*dbus};
    yaw_motor = new rm::device::GM6020{*can1, 1};
    pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{*can2, {}};
    imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};

    device_manager << rc << yaw_motor << pitch_motor;

    can1->SetFilter(0, 0);
    can1->Begin();
    can2->SetFilter(0, 0);
    can2->Begin();
    buzzer->Init();
    led->Init();
    rc->Begin();  // 启动遥控器接收，这行或许比较适合放到AppMain里面？
  }
} *globals;

void MainLoop() {
  globals->imu->Update();
  globals->ahrs.Update(rm::modules::ImuData6Dof{-globals->imu->gyro_y(),   //
                                                -globals->imu->gyro_x(),   //
                                                -globals->imu->gyro_z(),   //
                                                -globals->imu->accel_y(),  //
                                                -globals->imu->accel_x(),  //
                                                -globals->imu->accel_z()});
  // globals->yaw_motor->IsAlive();
  // globals->rc->IsAlive();
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
