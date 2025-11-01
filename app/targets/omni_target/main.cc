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

struct GlobalWarehouse {
  AsyncBuzzer *buzzer{nullptr};  ///< 蜂鸣器
  LED *led{nullptr};             ///< RGB LED灯

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
  rm::modules::MahonyAhrs ahrs{1000.f};  ///< mahony 姿态解算器，频率 1000Hz

  enum ControlMode {
    kNoForce,
    kNormal,
    kHoldSpin,
    kHeadless,
  } control_mode;
  ;
  float hold_spin_speed = 0.f;
  float heading_offset = 0.f;

  void Init() {
    buzzer = new AsyncBuzzer;
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

void MainLoop() {
  if (globals->device_manager.all_device_ok()) {
    (*globals->led)(0xff00ff00);  // 绿灯代表所有设备在线
  } else {
    (*globals->led)(0xffff0000);  // 红灯代表有设备离线
    globals->chassis_controller.Enable(false);
  }

  globals->imu->Update();
  globals->ahrs.Update(rm::modules::ImuData6Dof{globals->imu->gyro_x(),   //
                                                globals->imu->gyro_y(),   //
                                                globals->imu->gyro_z(),   //
                                                globals->imu->accel_x(),  //
                                                globals->imu->accel_y(),  //
                                                globals->imu->accel_z()});
  const auto &[roll, pitch, yaw] = globals->ahrs.euler_angle();

  globals->rc_l_switch_watcher.Update(globals->rc->switch_l());
  globals->rc_r_switch_watcher.Update(globals->rc->switch_r());

  if (globals->control_mode == GlobalWarehouse::ControlMode::kHoldSpin) {
    globals->chassis_controller.SetTarget(globals->rc->left_x() / 660.f * 20.f, globals->rc->left_y() / 660.f * 20.f,
                                          globals->hold_spin_speed);
  } else if (globals->control_mode == GlobalWarehouse::ControlMode::kHeadless) {
    float vx = globals->rc->left_x() / 660.f * 20.f;
    float vy = globals->rc->left_y() / 660.f * 20.f;
    float current_yaw = yaw;
    float cos_theta = cos(globals->heading_offset - current_yaw);
    float sin_theta = sin(globals->heading_offset - current_yaw);
    float vx_rotated = vx * cos_theta - vy * sin_theta;
    float vy_rotated = vx * sin_theta + vy * cos_theta;
    globals->chassis_controller.SetTarget(vx_rotated, vy_rotated,
                                          globals->rc->right_x() / 660.f * 23.f - globals->rc->dial() / 660.f * 23.f);
  } else {
    globals->chassis_controller.SetTarget(globals->rc->left_x() / 660.f * 20.f, globals->rc->left_y() / 660.f * 20.f,
                                          globals->rc->right_x() / 660.f * 23.f - globals->rc->dial() / 660.f * 23.f);
  }
  globals->chassis_controller.Update(globals->lf_motor->rpm() * (2.f * M_PI / 60.f) / 19.f,  //
                                     globals->rf_motor->rpm() * (2.f * M_PI / 60.f) / 19.f,  //
                                     globals->lb_motor->rpm() * (2.f * M_PI / 60.f) / 19.f,  //
                                     globals->rb_motor->rpm() * (2.f * M_PI / 60.f) / 19.f   //
  );
  globals->lf_motor->SetCurrent(globals->chassis_controller.output().lf_wheel);
  globals->rf_motor->SetCurrent(globals->chassis_controller.output().rf_wheel);
  globals->lb_motor->SetCurrent(globals->chassis_controller.output().lb_wheel);
  globals->rb_motor->SetCurrent(globals->chassis_controller.output().rb_wheel);
  rm::device::DjiMotor<>::SendCommand();
}

extern "C" [[noreturn]] void AppMain(void) {
  globals = new GlobalWarehouse;
  globals->Init();

  globals->rc_l_switch_watcher.OnValueChange(
      etl::delegate<void(const rm::device::DR16::SwitchPosition &, const rm::device::DR16::SwitchPosition &)>::create(
          [&](const rm::device::DR16::SwitchPosition &old_value, const rm::device::DR16::SwitchPosition &new_value) {
            if (globals->control_mode == GlobalWarehouse::ControlMode::kNormal) {
              if (new_value == rm::device::DR16::SwitchPosition::kUp) {
                globals->buzzer->Beep(2, 35);
                globals->hold_spin_speed = globals->rc->right_x() / 660.f * 23.f - globals->rc->dial() / 660.f * 23.f;
                globals->control_mode = GlobalWarehouse::ControlMode::kHoldSpin;
              }
            }
          }));
  globals->rc_r_switch_watcher.OnValueChange(
      etl::delegate<void(const rm::device::DR16::SwitchPosition &, const rm::device::DR16::SwitchPosition &)>::create(
          [&](const rm::device::DR16::SwitchPosition &old_value, const rm::device::DR16::SwitchPosition &new_value) {
            if (new_value == rm::device::DR16::SwitchPosition::kMid) {
              globals->buzzer->Beep(2, 35);
              globals->control_mode = GlobalWarehouse::ControlMode::kNormal;
              globals->chassis_controller.Enable(true);
            } else if (new_value == rm::device::DR16::SwitchPosition::kUp) {
              if (globals->control_mode != GlobalWarehouse::ControlMode::kHeadless) {
                globals->buzzer->Beep(2, 35);
                globals->control_mode = GlobalWarehouse::ControlMode::kHeadless;
                globals->chassis_controller.Enable(true);
              }
              globals->heading_offset = globals->ahrs.euler_angle().yaw;
            } else {
              globals->buzzer->Beep(1);
              globals->control_mode = GlobalWarehouse::ControlMode::kNoForce;
              globals->chassis_controller.Enable(false);
            }
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