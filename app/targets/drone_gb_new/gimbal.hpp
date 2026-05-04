#ifndef BOARDC_GIMBAL_HPP
#define BOARDC_GIMBAL_HPP
// 纯手瞄测试
#include <librm.hpp>
#include "can.h"
#include "usart.h"
#include "spi.h"
#include "timer_task.hpp"
#include "ControllerPidGimbal.hpp"
#include "ControllerPidAmmo.hpp"
#include "FreemasterDbug.hpp"
#include "Usb.hpp"
#include "Referee.hpp"
#include "vt03.hpp"

extern void FreemasterDebug();
extern AimbotFrame_SCM_t Aimbot;  // 自瞄数据引出
class Gimbal {
 public:
  double yaw = 0;    // imu yaw数据
  double roll = 0;   // imu roll数据
  double pitch = 0;  // imu pitch数据

  double yaw_ = 0;    // imu yaw数据(-pi到pi)
  double roll_ = 0;   // imu roll数据(-pi到pi)
  double pitch_ = 0;  // imu pitch数据(-pi到pi)

  double rc_yaw_data = 0;    // 遥控器yaw数据
  double rc_pitch_data = 0;  // 遥控器pitch数据

  bool DM_is_enable = false;  // 达秒使能标志位

  float pitch_min_pos = 3.00;        // pitch电机最小限位
  float pitch_max_pos = 4.00;        // pitch电机最大限位
                                     // 机械限位
  float yaw_center_encoder = 5.200;  // TODO云台机械中位对应的编码器角度
  float yaw_relative = 0.0f;         // TODO 当前云台相对机架夹角
  float yaw_min_limit = -2.30;       // TODO 左限位
  float yaw_max_limit = 2.30;        // TODO 右限位
  float yaw_delta = 0.0f;            // rc增加总量

  float dirl_speed = 5000;      // TODO 拨盘转速
  float redirl_speed = 1000;    // TODO 拨盘反转速
  float friction_speed = 6500;  // TODO 摩擦轮转速
  float shootstep = 100;        // TODO 手动调速步长

  // pitch补偿系数
  float pitch_torque = 0.0f;     // pitch电机力矩重力补偿量
  float pitch_torque_kp = 0.5f;  // TODO 重力补偿参数

  float pitch_cmd = 0.0f;       // pitch合输出
  float pitch_speed_tf = 0.0f;  // 速度正向输出
  float pitch_speed_kp = 0.1f;  // 速度输出比例系数

  int robot_id = 0;  // 裁判系统测试
  float rc_vt03_left_x = 0.0f;

  rm::hal::ThrottledCan<128> *can1{nullptr};  // CAN 总线接口
  rm::hal::Serial *dbus{nullptr};             // 遥控器串口接口
  rm::device::VT03 *vt03{nullptr};            // 图传对象

  rm::hal::SerialInterface *referee_uart;                                          // 裁判系统串口
  rm::device::RxReferee *rx_referee{nullptr};                                      // 裁判系统收发类
  rm::device::Referee<rm::device::RefereeRevision::kNewV120> referee_data_buffer;  // 裁判系统数据缓冲区

  rm::hal::SerialInterface *vt03_uart;   // 图传串口
  rm::device::Rxvt03 *rx_vt03{nullptr};  // 图传收发类

  rm::device::DeviceManager<1> device_rc;      // 遥控管理器，维护所有设备在线状态
  rm::device::DeviceManager<2> device_gimbal;  // 云台管理器
  rm::device::DeviceManager<3> device_shoot;   // 发射管理器

  int time_ = 0;  // 系统心跳

  rm::device::BMI088 *imu{nullptr};      // IMU
  rm::modules::MahonyAhrs ahrs{500.0f};  // TODO Mahony滤波控制频率
  rm::device::DR16 *rc{nullptr};         // 遥控器

  rm::device::GM6020 *yaw_motor{nullptr};                                           // 云台 Yaw 上电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr};  // 云台 Pitch 电机
  rm::device::M3508 *friction_left{nullptr};                                        // 左侧摩擦轮电机
  rm::device::M3508 *friction_right{nullptr};                                       // 右侧摩擦轮电机
  rm::device::M2006 *dial_motor{nullptr};                                           // 拨盘电机

  typedef enum {
    kNoForce,  // 云台无力
    kManual,   // 云台手动
    kAuto,     // 云台自瞄

    kStop,             // 发射机构无力
    kReady,            // 发射机构准备开火
    kFire              // 发射机构开火
  } StateMachineType;  // 遥控器状态机

  typedef struct {
    int16_t mouse_x = 0;
    int16_t mouse_y = 0;
    bool mouse_button_left = false;
    bool mouse_button_right = false;
    float rc_left_x = 0.0f;
    float rc_left_y = 0.0f;
  } vt03_date;

  StateMachineType AmmoState_ = {kStop};       // 初始化发射机构状态
  StateMachineType GimbalState_ = {kNoForce};  // 初始化云台运动状态
  vt03_date vt03_date_;                        // vt03信号结构体

  Gimbal2Dof gimbal_controller;  // 二轴云台PID控制器
  Shoot2Fric shoot_controller;   // 双摩擦轮发射机构控制器

  void GimbalInit() {
    time_ = 0;  // 系统心跳置0
    can1 = new rm::hal::ThrottledCan<128>{3000, hcan1};
    dbus = new rm::hal::Serial{huart3, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};

    imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
    rc = new rm::device::DR16{*dbus};
    vt03 = new rm::device::VT03;

    referee_uart = new rm::hal::Serial{huart6, 128, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
    rx_referee = new rm::device::RxReferee{*referee_uart};

    vt03_uart = new rm::hal::Serial{huart1, 128, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
    rx_vt03 = new rm::device::Rxvt03{*vt03_uart};

    yaw_motor = new rm::device::GM6020{*can1, 2};
    pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
        *can1, {0x05, 0x06, 10.0f, 20.0f, 10.0f, {0.0f, 10.0f}, {0.0f, 5.0f}}};

    friction_left = new rm::device::M3508{
        *can1,
        4,
    };
    friction_right = new rm::device::M3508{
        *can1,
        3,
    };
    dial_motor = new rm::device::M2006{*can1, 5};

    device_rc << rc;                                                // 遥控器
    device_gimbal << yaw_motor << pitch_motor;                      // 云台电机
    device_shoot << friction_left << friction_right << dial_motor;  // 发射机构电机

    can1->SetFilter(0, 0);  // 设置滤波器
    can1->Begin();
    rc->Begin();
    rx_referee->Begin();  // 启动裁判系统
    rx_vt03->Begin();     // 启动图传串口

    GimbalPIDInit();
    AmmoPIDInit();

    gimbal_controller.Enable(false);  // 云台控制器
    pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);

    shoot_controller.Enable(false);                   // 控制器初始化
    shoot_controller.Arm(false);                      // 摩擦轮武装（允许转动）
    shoot_controller.SetMode(Shoot2Fric::kFullAuto);  // 连发模式
    shoot_controller.SetLoaderSpeed(0.0f);            // 拨盘目标线速度
    shoot_controller.SetArmSpeed(0.0f);               // 摩擦轮目标线速度
  }

  void GimbalPIDInit() {
    gimbal_controller.pid().yaw_position.SetKp(160.0f).SetKi(0.0f).SetKd(0.01f).SetMaxOut(10000.0f).SetMaxIout(
        1000.0f);  // TODO yaw初版函数 350 0.001 0.2 160 100000
    gimbal_controller.pid().yaw_speed.SetKp(350.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(25000.0f).SetMaxIout(1000.0f);
//yaw原始参数 200 0 0.2  350 0 0

    gimbal_controller.pid().pitch_position.SetKp(35.0f).SetKi(0.002f).SetKd(0.01f).SetMaxOut(500.0f).SetMaxIout(
        10.0f);  // TODO pitch初版参数 35
    gimbal_controller.pid().pitch_speed.SetKp(0.8f).SetKi(0.0f).SetKd(0.001f).SetMaxOut(10.0f).SetMaxIout(5.0f);
  }

  void AmmoPIDInit() {
    shoot_controller.pid().fric_1_speed.SetKp(18.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(20000.0f).SetMaxIout(1000.0f);
    shoot_controller.pid().fric_2_speed.SetKp(18.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(20000.0f).SetMaxIout(1000.0f);
    shoot_controller.pid().loader_speed.SetKp(15.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(20000.0f).SetMaxIout(2000.0f);
  }

  void RCStateUpdate() {
    switch (rc->switch_r()) {
      case rm::device::DR16::SwitchPosition::kUp:  // 发射控制逻辑
        AmmoState_ = kFire;
        break;
      case rm::device::DR16::SwitchPosition::kMid:
        AmmoState_ = kReady;
        break;
      default:
        AmmoState_ = kStop;
        break;
    }
    switch (rc->switch_l()) {
      case rm::device::DR16::SwitchPosition::kUp:  // 上打自瞄
        GimbalState_ = kAuto;
        break;
      case rm::device::DR16::SwitchPosition::kMid:  // 手动
        GimbalState_ = kManual;
        break;
      default:
        GimbalState_ = kNoForce;
        break;
    }
  }

  void VT03DateUpdate() {
    vt03_date_.mouse_x = vt03->data().mouse_x;
    vt03_date_.mouse_y = vt03->data().mouse_y;
    vt03_date_.mouse_button_left = vt03->data().mouse_button_left;
    vt03_date_.mouse_button_right = vt03->data().mouse_button_right;
    vt03_date_.rc_left_x = vt03->data().left_x;
    vt03_date_.rc_left_y = vt03->data().left_y;
  }

  float GetYawMotorAngleRad() {  // 编码器返回角度
    return yaw_motor->encoder() * 2.0f * M_PI / 8192.0f;
  }

  void GimbalControl() {
    if (GimbalState_ == kManual) {
      if (DM_is_enable == false) {
        pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
        DM_is_enable = true;
        gimbal_controller.Enable(true);

        rc_yaw_data = yaw;                                      // 第一次进入更新当前位置
        rc_pitch_data = rm::modules::Wrap(pitch, 0, 2 * M_PI);  // 使用 IMU pitch 作为初始姿态
        rc_pitch_data = rm::modules::Clamp(rc_pitch_data, pitch_min_pos, pitch_max_pos);  // 对rc数据进行限位
      }
      yaw_relative = rm::modules::Wrap(GetYawMotorAngleRad() - yaw_center_encoder, -M_PI, M_PI);  // 相对机械中点误差
      yaw_delta = 0.0f;

      // yaw
      yaw_delta -= rm::modules::Map(rc->left_x(), -660, 660, -0.005f, 0.005f);      // dt7手控
      yaw_delta -= rm::modules::Map(vt03_date_.rc_left_y, -1, 1, -0.005f, 0.005f);      // vt03手控备份
      yaw_delta -= rm::modules::Map(rc->mouse_x(), -660, 660, -0.03f, 0.03f);       // dt7备份控制
      yaw_delta -= rm::modules::Map(vt03_date_.mouse_x, -660, 660, -0.03f, 0.03f);  // vt03鼠标控制

      if (yaw_relative >= yaw_max_limit && yaw_delta < 0.0f) {  // 机械限位返回逻辑
        yaw_delta = 0.0f;
      }
      if (yaw_relative <= yaw_min_limit && yaw_delta > 0.0f) {
        yaw_delta = 0.0f;
      }

      rc_yaw_data = rm::modules::Wrap(rc_yaw_data + yaw_delta, 0, 2 * M_PI);

      // pitch
      rc_pitch_data -= rm::modules::Map(rc->left_y(), -660, 660, -0.005f, 0.005f);      // dt7手控
      rc_pitch_data -= rm::modules::Map(vt03_date_.rc_left_x, -1, 1, -0.005f, 0.005f);      // vt03手控备份
      rc_pitch_data -= rm::modules::Map(rc->mouse_y(), -660, 660, -0.03f, 0.03f);       // dt7备份控制
      rc_pitch_data -= rm::modules::Map(vt03_date_.mouse_y, -660, 660, -0.03f, 0.03f);  // vt03鼠标控制
      rc_pitch_data = rm::modules::Clamp(rc_pitch_data, pitch_min_pos, pitch_max_pos);

      gimbal_controller.SetTarget(rc_yaw_data, rc_pitch_data, 0, 0);
      gimbal_controller.Update(yaw, -yaw_motor->rpm(), rm::modules::Wrap(pitch, 0, 2 * M_PI), pitch_motor->vel(), 2.f);
      yaw_motor->SetCurrent(rm::modules::Clamp(-gimbal_controller.output().yaw, -25000, 25000));  // 设置输出电流并输出

      pitch_torque = pitch_torque_kp * cos(pitch - 3.14);  // 这里输出的力矩是反向
      pitch_torque = rm::modules::Clamp(pitch_torque, -3, 3);

    } else if (GimbalState_ == kAuto) {  // 自瞄模式控制
      if (DM_is_enable == false) {       // 使达妙电机使能
        pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
        DM_is_enable = true;
        gimbal_controller.Enable(true);
        rc_yaw_data = yaw;
        rc_pitch_data = rm::modules::Wrap(pitch, 0, 2 * M_PI);  // 使用 IMU pitch 作为初始姿态
        rc_pitch_data = rm::modules::Clamp(rc_pitch_data, pitch_min_pos, pitch_max_pos);
      }

      if (Aimbot.AimbotState==2) {
        rc_yaw_data = Aimbot.TargetYawAngle + M_PI;
        rc_yaw_data = rm::modules::Wrap(rc_yaw_data, 0, 2 * M_PI);

        rc_pitch_data = rm::modules::Wrap(-Aimbot.TargetPitchAngle + M_PI, 0, 2 * M_PI);
        rc_pitch_data = rm::modules::Clamp(rc_pitch_data, pitch_min_pos, pitch_max_pos);
      } else {  // 非自瞄状态自动切入手控
        // yaw
        yaw_relative = rm::modules::Wrap(GetYawMotorAngleRad() - yaw_center_encoder, -M_PI, M_PI);  // 相对机械中点误差
        yaw_delta = 0.0f;                                                                           // 合输出

        yaw_delta -= rm::modules::Map(rc->left_x(), -660, 660, -0.005f, 0.005f);      // dt7手控
        yaw_delta -= rm::modules::Map(vt03_date_.rc_left_y, -1, 1, -0.005f, 0.005f);      // vt03手控备份
        yaw_delta -= rm::modules::Map(rc->mouse_x(), -660, 660, -0.03f, 0.03f);       // dt7备份控制
        yaw_delta -= rm::modules::Map(vt03_date_.mouse_x, -660, 660, -0.03f, 0.03f);  // vt03鼠标控制

        if (yaw_relative >= yaw_max_limit && yaw_delta < 0.0f) {  // 机械限位返回逻辑
          yaw_delta = 0.0f;
        }
        if (yaw_relative <= yaw_min_limit && yaw_delta > 0.0f) {
          yaw_delta = 0.0f;
        }
        rc_yaw_data = rm::modules::Wrap(rc_yaw_data + yaw_delta, 0, 2 * M_PI);

        // pitch
        rc_pitch_data -= rm::modules::Map(rc->left_y(), -660, 660, -0.005f, 0.005f);      // dt7手控
        rc_pitch_data -= rm::modules::Map(vt03_date_.rc_left_x, -1, 1, -0.005f, 0.005f);      // vt03手控备份
        rc_pitch_data -= rm::modules::Map(rc->mouse_y(), -660, 660, -0.03f, 0.03f);       // dt7备份控制
        rc_pitch_data -= rm::modules::Map(vt03_date_.mouse_y, -660, 660, -0.03f, 0.03f);  // vt03鼠标控制
        rc_pitch_data = rm::modules::Clamp(rc_pitch_data, pitch_min_pos, pitch_max_pos);
      }
      gimbal_controller.SetTarget(rc_yaw_data, rc_pitch_data);
      gimbal_controller.Update(yaw, -yaw_motor->rpm(), rm::modules::Wrap(pitch, 0, 2 * M_PI), pitch_motor->vel(), 2.f);
      yaw_motor->SetCurrent(rm::modules::Clamp(-gimbal_controller.output().yaw, -25000, 25000));

      pitch_torque = pitch_torque_kp * cos(pitch - 3.14);  // 这里输出的力矩是反向
      pitch_torque = rm::modules::Clamp(pitch_torque, -3, 3);
    } else {
      if (DM_is_enable == true) {
        pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
        DM_is_enable = false;
        gimbal_controller.Enable(false);
        yaw_motor->SetCurrent(0);
      }
    }
  }

  void AmmoControl() {
    // 发射状态
    if (AmmoState_ == kFire) {
      shoot_controller.Enable(true);
      shoot_controller.Arm(true);
      shoot_controller.SetMode(Shoot2Fric::kFullAuto);

      if (rc->dial() >= 550 || vt03_date_.mouse_button_left) {
        shoot_controller.SetLoaderSpeed(dirl_speed);
      } else if (rc->dial() <= -600) {
        shoot_controller.SetLoaderSpeed(-redirl_speed);
      } else {
        shoot_controller.SetLoaderSpeed(0);
      }

      shoot_controller.SetArmSpeed(friction_speed);  // 摩擦轮目标线速度（rad/s 或你的系统单位）
      shoot_controller.Update(friction_left->rpm(), friction_right->rpm(), dial_motor->rpm());

      friction_left->SetCurrent((int16_t)rm::modules::Clamp(shoot_controller.output().fric_1, -10000, 10000));
      friction_right->SetCurrent((int16_t)rm::modules::Clamp(shoot_controller.output().fric_2, -10000, 10000));
      dial_motor->SetCurrent((int16_t)rm::modules::Clamp(shoot_controller.output().loader, -10000, 10000));

    }

    // 准备状态
    else if (AmmoState_ == kReady) {
      shoot_controller.Enable(true);
      shoot_controller.Arm(true);

      shoot_controller.SetMode(Shoot2Fric::kStop);
      shoot_controller.SetArmSpeed(0.0f);

      shoot_controller.Update(friction_left->rpm(), friction_right->rpm(), dial_motor->rpm());

      friction_left->SetCurrent((int16_t)rm::modules::Clamp(shoot_controller.output().fric_1, -10000, 10000));
      friction_right->SetCurrent((int16_t)rm::modules::Clamp(shoot_controller.output().fric_2, -10000, 10000));

    }

    // 停止状态
    else {
      shoot_controller.Enable(false);
      shoot_controller.Arm(false);
      friction_left->SetCurrent(0);
      friction_right->SetCurrent(0);
      dial_motor->SetCurrent(0);
    }
  }

  void ShootSpeedControl() {
    if (vt03->data().keyboard_key)
      ;
    // 弹速控制
  }

  // void Referee_control();  // 裁判系统常规链路

  // 遥控器和imu数据解算+DjiMotor发信息
  void SubLoop500Hz() {
    // imu数据处理
    imu->Update();
    ahrs.Update(rm::modules::ImuData6Dof{imu->gyro_y(), imu->gyro_x(), -imu->gyro_z() + 0.00225f, imu->accel_y(),
                                         imu->accel_x(), -imu->accel_z()});
    pitch = ahrs.euler_angle().pitch + M_PI;
    yaw = ahrs.euler_angle().yaw + M_PI;
    roll = ahrs.euler_angle().roll + M_PI;

    GimbalImuSend(ahrs.quaternion().w, ahrs.quaternion().x, ahrs.quaternion().y, ahrs.quaternion().z,
                  referee_data_buffer.data().shoot_data.initial_speed,
                  referee_data_buffer.data().robot_status.robot_id);  // usb传输数据
    RCStateUpdate();                                                  // DT7遥控器更新
    VT03DateUpdate();                                                 // vt03数据更新
    GimbalControl();                                                  // 云台控制更新
    AmmoControl();                                                    // 发射机构更新
    rm::device::DjiMotorBase::SendCommand(*can1);                     // 向大疆所有电机发数据
    FreemasterDebug();                                                // 调试更新
  }

  // DmMotor电机发信息
  void SubLoop250Hz() {
    if (time_ % 2 == 0) {
      pitch_cmd = rm::modules::Clamp(-pitch_torque + gimbal_controller.output().pitch, -10, 10);  // 发送达秒控制信息
      pitch_motor->SetMitCommand(0, 0, pitch_cmd, 0, 0);                                          // 合输出

      // pitch_speed_tf = rm::modules::Clamp(pitch_speed_kp * tanh(pitch_motor->vel()),-10,10);

      // pitch_motor->SetMitCommand(0, 0, gimbal_controller.output().pitch, 0, 0);
      // pitch_motor->SetMitCommand(0, 0,-pitch_torque, 0, 0);

      // if (pitch<=3.82&&pitch>=3.00) {//摩擦补偿测试
      //   pitch_motor->SetMitCommand(0, 0,pitch_speed_tf-pitch_torque, 0, 0);
      // }
      // else {
      //   pitch_motor->SetMitCommand(0,0,0,0,0);
      // }
    }
  }

  void SubLoop100Hz() {
    if (time_ % 5 == 0) {
      // FreemasterDebug();
    }
  }
  void SubLoop50Hz() {
    if (time_ % 10 == 0) {
      // Referee_control();
      robot_id = referee_data_buffer.data().robot_status.robot_id;  // 裁判系统测试
    }
  }
  void SubLoop10Hz() {
    if (time_ % 50 == 0) {
      time_ = 0;
    }
  }
};

#endif  // BOARDC_GIMBAL_HPP
