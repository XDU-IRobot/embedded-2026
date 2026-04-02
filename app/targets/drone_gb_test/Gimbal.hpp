#ifndef BOARDC_GIMBAL_HPP
#define BOARDC_GIMBAL_HPP

#define TEST_GIMBAL 1

#include <librm.hpp>
#include <utility>

#include "can.h"
#include "usart.h"
#include "spi.h"
#include "timer_task.hpp"
#include "controllers/gimbal_2dof.hpp"
#include "controllers/shoot_2firc.hpp"
#include "Usb.hpp"
// #include "old_sentry/main.hpp"

double yaw_ = 0;
double pitch_ = 0;
double roll_ = 0;

int16_t rc_left_x = 0;
int16_t rc_left_y = 0;
uint8_t rc_switch_l = 0;

double rc_yaw = 0;
double rc_pitch = 0;

float pitch_torque = 0.0f;
float pitch_torque_kp = 0.0f;

class Gimbal {
 public:
  int aaaa = 0;

  rm::hal::Can *can1{nullptr};
  rm::hal::Serial *dbus{nullptr};
  rm::device::DeviceManager<1> device_rc;
  rm::device::DeviceManager<2> device_gimbal;
  int time_ = 0;

  rm::device::BMI088 *imu{nullptr};
  rm::modules::MahonyAhrs ahrs{490.0f};
  rm::modules::MahonyAhrs ahrs_auto{490.0f};

  double pitch = 0;
  double roll = 0;
  double yaw = 0;

  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr};
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *yaw_motor{nullptr};
  rm::device::M3508 *friction_left{nullptr};

  rm::device::DR16 *rc{nullptr};

  int mode_change_time = 0;

  typedef enum {
    kNoForce,  // 无力
    kManual,   // 手动

    // kStop,
    // kReady,
  } StateMachineType;
  StateMachineType GimbalState_ = {kNoForce};

  double rc_pitch_date = 0;
  double rc_yaw_date = 0;

  bool DM_is_enable = false;
  Gimbal2Dof gimbal_controller;

#if TEST_GIMBAL
  float pitch_min_pos = 2.48;  // 预定义宏可以快速转换限位
  float pitch_max_pos = 3.8;
#else
  float pitch_min_pos = 1.6;
  float pitch_max_pos = 2.75;
#endif

  double err_average = 0;  // 误差平均值

  void GimbalInit() {
    can1 = new rm::hal::Can{hcan1};
    dbus = new rm::hal::Serial{huart3, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};

    imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};

    rc = new rm::device::DR16{*dbus};

    yaw_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
        *can1, {0x12, 0x02, 10.0f, 20.0f, 10.0f, {0.0f, 10.0f}, {0.0f, 5.0f}}};  // 设置对于can设备的报文
    pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
        *can1, {0x11, 0x01, 10.0f, 20.0f, 10.0f, {0.0f, 10.0f}, {0.0f, 5.0f}}};
    friction_left = new rm::device::M3508{*can1, 5};

    device_rc << rc;
    device_gimbal << pitch_motor << yaw_motor;  // 设备管理器，可以一次性管理大多数设备

    can1->SetFilter(0, 0);
    can1->Begin();
    rc->Begin();

    time_ = 0;

    GimbalPIDInit();

    gimbal_controller.Enable(false);
    pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
  }
  // pid初始化

  void GimbalPIDInit() {
    gimbal_controller.pid().pitch_position.SetKp(15.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(500.0f).SetMaxIout(10.0f);
    gimbal_controller.pid().pitch_speed.SetKp(0.6f).SetKi(0.0f).SetKd(0.001f).SetMaxOut(10.0f).SetMaxIout(5.0f);

    gimbal_controller.pid().yaw_position.SetKp(18.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10.0f).SetMaxIout(1000.0f);
    gimbal_controller.pid().yaw_speed.SetKp(0.5f).SetKi(0.0f).SetKd(0.001f).SetMaxOut(10.0f).SetMaxIout(1000.0f);
  }

  void RCStateUpdate() {
    switch (rc->switch_l()) {
      case rm::device::DR16::SwitchPosition::kUp:
        GimbalState_ = kNoForce;
        break;
      case rm::device::DR16::SwitchPosition::kDown:
        GimbalState_ = kManual;  // 上打无力下打受控
        break;
    }
  }

  // 控制逻辑
  void GimbalControl() {
    if (GimbalState_ == kManual) {
      if (DM_is_enable == false) {
        pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
        yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
        DM_is_enable = true;
        gimbal_controller.Enable(true);

        rc_yaw_date = yaw;
        rc_pitch_date = rm::modules::Wrap(pitch + err_average, 0, 2 * M_PI);
      }

      rc_yaw_date -= rm::modules::Map(rc->left_x(), -660, 660, -0.005f, 0.005f);
      rc_yaw_date = rm::modules::Wrap(rc_yaw_date, 0, 2 * M_PI);

      rc_pitch_date -= rm::modules::Map(rc->left_y(), -660, 660, -0.005f, 0.005f);
      rc_pitch_date = rm::modules::Clamp(rc_pitch_date, pitch_min_pos, pitch_max_pos);

      gimbal_controller.SetTarget(rc_yaw_date, rc_pitch_date);

      gimbal_controller.Update(yaw, yaw_motor->vel(), pitch, pitch_motor->vel(), 2.f);
      // friction_left->SetCurrent(1000);

      pitch_torque = pitch_torque_kp * sin(pitch - 3.7);
      pitch_torque = rm::modules::Clamp(pitch_torque, -3, 3);

    } else {
      if (DM_is_enable == true) {  // 使达妙电机使能
        pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
        yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
        DM_is_enable = false;
        gimbal_controller.Enable(false);
      }
    }
  }

  void SubLoop500Hz() {
    RCStateUpdate();  // 遥控器更新
    GimbalControl();  // 云台控制更新
    // imu处理
    imu->Update();

#if NEW_DRONE_GB
    ahrs_auto.Update(rm::modules::ImuData6Dof{imu->gyro_y(), imu->gyro_x(), -imu->gyro_z(), imu->accel_y(),
                                              imu->accel_x(), -imu->accel_z()});
#else
    ahrs_auto.Update(rm::modules::ImuData6Dof{-imu->gyro_x(), imu->gyro_y(), -imu->gyro_z(), -imu->accel_x(),
                                              imu->accel_y(), -imu->accel_z()});
#endif

    pitch = ahrs_auto.euler_angle().pitch + M_PI;
    yaw = ahrs_auto.euler_angle().yaw + M_PI;
    roll = ahrs_auto.euler_angle().roll + M_PI;

    yaw_ = yaw;  // imu测试数据
    pitch_ = pitch;

    rc_left_x = rc->left_x();  // rc测试数据
    rc_left_y = rc->left_y();
    rc_switch_l = static_cast<uint8_t>(GimbalState_);
  }

  // damiao电机控制信号
  void SubLoop250Hz() {
    if (time_ % 2 == 0) {
      double pitch_torque_cmd = gimbal_controller.output().pitch + pitch_torque;
      pitch_motor->SetMitCommand(0, 0, pitch_torque_cmd, 0, 0);
      yaw_motor->SetMitCommand(0, 0, gimbal_controller.output().yaw, 0, 0);
      rc_yaw = rc_yaw_date;
      rc_pitch = rc_pitch_date;

      // rm::device::DjiMotorBase::SendCommand(*can1);
    }
  }

  void SubLoop10Hz() {
    if (time_ % 50 == 0) time_ = 0;
  }
};

#endif  // BOARDC_GIMBAL_HPP
