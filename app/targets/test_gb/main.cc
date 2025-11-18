#include <librm.hpp>

#include "can.h"
#include "usart.h"
#include "spi.h"

#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "timer_task.hpp"
#include "sparse_value_watcher.hpp"
#include "device_manager.hpp"
#include "controllers/quad_omni_chassis.hpp"

#include "test_gb/gimbal.hpp"

struct GlobalWarehouse {
  Buzzer *buzzer{nullptr};  ///< 蜂鸣器
  LED *led{nullptr};        ///< RGB LED灯

  // 硬件接口 //
  rm::hal::Can *can2{nullptr};     ///< CAN 总线接口
  rm::hal::Serial *dbus{nullptr};  ///< 遥控器串口接口

  // 设备 //
  DeviceManager<10> device_manager;  ///< 设备管理器，维护所有设备在线状态
  rm::device::DR16 *rc{nullptr};     ///< 遥控器
  rm::device::M3508 *lf_motor{nullptr}, *rf_motor{nullptr}, *lb_motor{nullptr}, *rb_motor{nullptr};  ///< 四个底盘电机
  rm::device::BMI088 *imu{nullptr};

  // 控制器 //
  QuadOmniChassis chassis_controller;
  SparseValueWatcher<rm::device::DR16::SwitchPosition> rc_l_switch_watcher, rc_r_switch_watcher;
  rm::modules::MahonyAhrs ahrs{500.f};  ///< mahony 姿态解算器，频率 1000Hz

  void Init() {
    buzzer = new Buzzer;
    led = new LED;

    can2 = new rm::hal::Can{hcan2};
    dbus = new rm::hal::Serial{huart3, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};

    rc = new rm::device::DR16{*dbus};
    lf_motor = new rm::device::M3508{*can2, 4};
    rf_motor = new rm::device::M3508{*can2, 1};
    lb_motor = new rm::device::M3508{*can2, 3};
    rb_motor = new rm::device::M3508{*can2, 2};
    imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};

    device_manager << rc << lf_motor << rf_motor << lb_motor << rb_motor;

    can2->SetFilter(0, 0);
    can2->Begin();
    buzzer->Init();
    led->Init();
    rc->Begin();  // 启动遥控器接收，这行或许比较适合放到AppMain里面？
  }
} *globals;

void MainLoop() { rm::device::DjiMotor<>::SendCommand(); }

extern "C" [[noreturn]] void AppMain(void) {
  globals = new GlobalWarehouse;
  globals->Init();

  globals->rc_l_switch_watcher.OnValueChange(
      etl::delegate<void(const rm::device::DR16::SwitchPosition &, const rm::device::DR16::SwitchPosition &)>::create(
          [&](const rm::device::DR16::SwitchPosition &old_value, const rm::device::DR16::SwitchPosition &new_value) {

          }));
  globals->rc_r_switch_watcher.OnValueChange(
      etl::delegate<void(const rm::device::DR16::SwitchPosition &, const rm::device::DR16::SwitchPosition &)>::create(
          [&](const rm::device::DR16::SwitchPosition &old_value, const rm::device::DR16::SwitchPosition &new_value) {

          }));

  auto &pids = globals->chassis_controller.pid();
  pids.lf_wheel.SetKp(2400.f).SetMaxOut(16384.f);
  pids.rf_wheel.SetKp(2400.f).SetMaxOut(16384.f);
  pids.lb_wheel.SetKp(2400.f).SetMaxOut(16384.f);
  pids.rb_wheel.SetKp(2400.f).SetMaxOut(16384.f);

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_1000hz.SetPrescalerAndPeriod(84 - 1, 1000 - 1);  // 84MHz / 84 / 1000 = 1kHz
  mainloop_1000hz.Start();

  for (;;) {
    __WFI();
  }
}