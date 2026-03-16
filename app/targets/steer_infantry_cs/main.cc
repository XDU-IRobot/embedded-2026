#include <librm.hpp>

#include "can.h"
#include "usart.h"
#include "spi.h"

#include "timer_task.hpp"

#include "main.hpp"
#include "Chassis.hpp"

using namespace rm;

void MainLoop() {
  globals->time++;
  globals->SubLoop500Hz();
  globals->SubLoop250Hz();
  globals->SubLoop100Hz();
  globals->SubLoop50Hz();
  globals->SubLoop10Hz();
}

extern "C" [[noreturn]] void AppMain(void) {
  rm::Sleep(std::chrono::milliseconds(100));  // 等待设备初始化完成
  // globals = new GlobalWarehouse;
  chassis = new Chassis;
  globals->Init();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_1000hz.SetPrescalerAndPeriod(168 - 1, 1000 - 1);  // 84MHz / 168 / 1000 = 500Hz
  mainloop_1000hz.Start();

  for (;;) {
    // __WFI();
  }
}

void GlobalWarehouse::Init() {
  buzzer = new Buzzer;
  led = new LED;

  can1 = new rm::hal::Can{hcan1};
  can2 = new rm::hal::Can{hcan2};
  gimbal_communicator = new rm::device::GimbalCommunicator(*can1);
  imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
  dbus = new rm::hal::Serial{huart3, 18, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
  referee_uart = new rm::hal::Serial{huart6, 128, hal::stm32::UartMode::kNormal, hal::stm32::UartMode::kDma};
  rx_referee = new rm::device::RxReferee{*globals->referee_uart};

  referee_data = new rm::device::Referee<rm::device::RefereeRevision::kNewV110>;

  yaw_motor = new rm::device::GM6020{*can1, 1};
  steer_lf = new rm::device::GM6020{*can1, 1};
  steer_rf = new rm::device::GM6020{*can1, 3};
  steer_lb = new rm::device::GM6020{*can1, 4};
  steer_rb = new rm::device::GM6020{*can1, 2};
  wheel_lf = new rm::device::M3508{*can1, 1};
  wheel_rf = new rm::device::M3508{*can1, 3};
  wheel_lb = new rm::device::M3508{*can1, 4};
  wheel_rb = new rm::device::M3508{*can1, 2};

  device_chassis << steer_lf << steer_rf << steer_lb << steer_rb   // 底盘舵电机
                 << wheel_lf << wheel_rf << wheel_lb << wheel_rb;  // 底盘轮电机

  can1->SetFilter(0, 0);
  can1->Begin();
  can2->SetFilter(0, 0);
  can2->Begin();
  rx_referee->Begin();
  buzzer->Init();
  led->Init();

  led_controller.SetPattern<modules::led_pattern::GreenBreath>();
  buzzer_controller.Play<modules::buzzer_melody::Startup>();

  globals->ChassisPIDInit();
  chassis->ChassisInit();
}

void ChassisStateUpdate(){}

void GlobalWarehouse::ChassisPIDInit() {
  chassis_controller.pid().lf_steer_position.SetKp(0.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().lf_steer_speed.SetKp(0.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rf_steer_position.SetKp(0.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rf_steer_speed.SetKp(0.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().lb_steer_position.SetKp(0.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().lb_steer_speed.SetKp(0.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rb_steer_position.SetKp(0.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rb_steer_speed.SetKp(0.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().lf_wheel.SetKp(5.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(6000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rf_wheel.SetKp(5.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(6000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().lb_wheel.SetKp(5.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(6000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rb_wheel.SetKp(5.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(6000.0f).SetMaxIout(0.0f);
}

void GlobalWarehouse::SubLoop500Hz() {
  globals->imu->Update();
  globals->ahrs.Update(rm::modules::ImuData6Dof{globals->imu->gyro_x(), globals->imu->gyro_y(),
                                                globals->imu->gyro_z() - 0.0015f, globals->imu->accel_x(),
                                                globals->imu->accel_y(), globals->imu->accel_z()});
  chassis->ChassisTask();

  rm::device::DjiMotorBase::SendCommand(*can1);
  rm::device::DjiMotorBase::SendCommand(*can2);
}

void GlobalWarehouse::SubLoop250Hz() {
  if (globals->time % 2 == 0) {
  }
}

void GlobalWarehouse::SubLoop100Hz() {
  if (globals->time % 5 == 0) {
    globals->device_chassis.Update();
  }
}

void GlobalWarehouse::SubLoop50Hz() {
  if (globals->time % 10 == 0) {
    const auto &[led_r, led_g, led_b] = globals->led_controller.Update();
    (*globals->led)(0xff000000 | led_r << 16 | led_g << 8 | led_b);
    buzzer->SetFrequency(globals->buzzer_controller.Update().frequency);
  }
}

void GlobalWarehouse::SubLoop10Hz() {
  if (globals->time % 50 == 0) {
    globals->time = 0;
  }
}
