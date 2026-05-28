#include <librm.hpp>

#include "can.h"
#include "usart.h"
#include "spi.h"

#include "timer_task.hpp"

#include "main.hpp"
#include "Chassis.hpp"
#include "subReferee/TaskScheduler.hpp"
#include "UI/UIuser1.hpp"
#include "UI/UIDrone.hpp"

using namespace rm;

static auto schedule = device::UITaskScheduler(30);

extern u16 robotID;

void static_UI_add();
void UiRefresh();

void MainLoop() {
  globals->time++;
  globals->SubLoop500Hz();
  globals->SubLoop250Hz();
  globals->SubLoop100Hz();
  globals->SubLoop50Hz();
  globals->SubLoop30Hz();
  globals->SubLoop10Hz();
}

extern "C" [[noreturn]] void AppMain(void) {
  rm::Sleep(std::chrono::milliseconds(100));  // 等待设备初始化完成
  globals = new GlobalWarehouse;
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
  super_cap = new rm::device::GkSupercap(*can1);
  imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
  referee_uart = new rm::hal::Serial<128>{huart6, false, true};
  subReferee = new rm::device::RefereeUser(*referee_data);
  referee_data = new rm::device::Referee<rm::device::RefereeRevision::kNewV120>;
  rx_referee = new rm::device::RxReferee{*referee_uart, *referee_data};
  referee_data->AttachCallback([this]<typename T0, typename T1>(T0 &&PH1, T1 &&PH2) {
    subReferee->AttachCallback(std::forward<T0>(PH1), std::forward<T1>(PH2));
  });

  yaw_motor = new rm::device::GM6020{*can1, 4};

  steer_lf = new rm::device::GM6020{*can2, 1};
  steer_rf = new rm::device::GM6020{*can2, 2};
  steer_lb = new rm::device::GM6020{*can2, 4};
  steer_rb = new rm::device::GM6020{*can2, 3};
  wheel_lf = new rm::device::M3508{*can2, 1};
  wheel_rf = new rm::device::M3508{*can2, 2};
  wheel_lb = new rm::device::M3508{*can2, 4};
  wheel_rb = new rm::device::M3508{*can2, 3};

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

void GlobalWarehouse::ChassisPIDInit() {
  chassis_controller.pid().lf_steer_position.SetKp(1000.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(
      0.0f);
  chassis_controller.pid().lf_steer_speed.SetKp(60.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rf_steer_position.SetKp(1000.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(
      0.0f);
  chassis_controller.pid().rf_steer_speed.SetKp(60.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().lb_steer_position.SetKp(1000.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(
      0.0f);
  chassis_controller.pid().lb_steer_speed.SetKp(60.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rb_steer_position.SetKp(1000.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(
      0.0f);
  chassis_controller.pid().rb_steer_speed.SetKp(60.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().lf_wheel.SetKp(5.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(6000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rf_wheel.SetKp(5.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(6000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().lb_wheel.SetKp(5.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(6000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rb_wheel.SetKp(5.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(6000.0f).SetMaxIout(0.0f);
}

void GlobalWarehouse::SubLoop500Hz() {
  globals->imu->Update();
  globals->ahrs.Update(rm::modules::ImuData6Dof{globals->imu->gyro_y(), globals->imu->gyro_z(),
                                                globals->imu->gyro_x() + 0.0015f, globals->imu->accel_y(),
                                                globals->imu->accel_z(), globals->imu->accel_x()});
  globals->gimbal_communicator->SendGimbalCommand(
      globals->referee_data->data().power_heat_data.shooter_17mm_1_barrel_heat,
      globals->referee_data->data().robot_status.shooter_barrel_heat_limit,
      globals->referee_data->data().shoot_data.initial_speed,
      globals->referee_data->data().robot_status.power_management_gimbal_output |
          globals->referee_data->data().robot_status.power_management_gimbal_output << 1 |
          globals->referee_data->data().robot_status.power_management_gimbal_output << 2,
      globals->referee_data->data().robot_status.robot_id);
  globals->super_cap_tx.feedback_referee_energy_buffer = globals->referee_data->data().power_heat_data.buffer_energy;
  globals->super_cap->Update(globals->super_cap_tx);
  chassis->ChassisTask();
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
    UiRefresh();
    // globals->time = 0;
  }
}

void GlobalWarehouse::SubLoop30Hz() {
  if (globals->time % 34 == 0) {
    schedule.schedule();
  }
}

void UiRefresh() {
  // 接收机器人ID
  robotID = globals->referee_data->data().robot_status.robot_id;
  if (globals->gimbal_communicator->UI_show_flag() == 1) {
    static_UI_add();
  }
}

static auto UIRobotHeaderBlueADD = device::UITask(UITextHeaderRobotBlue_add);

static auto UIhpBlueADD = device::UITask(UITextHeaderHPBlue_add);
static auto UIhpBlueEDIT = device::UITask(UITextHeaderHPBlue_edit, 2);

static auto UIalBlueADD = device::UITask(UITextHeaderAllowBlue_add);
static auto UIalBlueEDIT = device::UITask(UITextHeaderAllowBlue_edit, 2);

static auto UIDroneHeroADD1 = device::UITask(UIInfantryAdd1);
static auto UIDroneHeroADD2 = device::UITask(UIInfantryAdd2);
static auto UIDroneHeroADD3 = device::UITask(UIInfantryAdd3);
static auto UIDroneHeroADD4 = device::UITask(UIInfantryAdd4);
static auto UIDroneHeroEDIT = device::UITask(UIInfantryEdit, 10);

void static_UI_add() {
  schedule.addTaskStatic(&UIDroneHeroADD1);
  schedule.addTaskStatic(&UIDroneHeroADD2);
  schedule.addTaskStatic(&UIDroneHeroADD3);
  schedule.addTaskStatic(&UIDroneHeroADD4);
  schedule.addTaskStatic(&UIRobotHeaderBlueADD);
  schedule.addTaskStatic(&UIalBlueADD);
  schedule.addTaskStatic(&UIhpBlueADD);

  schedule.addTask(&UIDroneHeroEDIT);
  schedule.addTask(&UIhpBlueEDIT);
  schedule.addTask(&UIalBlueEDIT);
}