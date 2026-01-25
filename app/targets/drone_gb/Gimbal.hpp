#ifndef BOARDC_GIMBAL_HPP
#define BOARDC_GIMBAL_HPP

#include <librm.hpp>
#include <utility>

#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "can.h"
#include "usart.h"
#include "spi.h"
#include "timer_task.hpp"
#include "controllers/gimbal_2dof.hpp"
#include "controllers/shoot_2firc.hpp"
#include "Usb.hpp"
#include "VOFA.hpp"
#include "Referee.hpp"
#include "FreertosDbug.hpp"
#include "WS2812b.hpp"

extern double Ayaw;
extern double Apitch;
extern double Aroll;
extern double Aoutputyaw;
extern double Aoutputpitch;
extern double Arcyawdata;
extern double Arcpitchdata;
extern double Agx;
extern double Agy;
extern double Agz;
extern double Apitchpose;
extern double Atotalpitch;
extern double Atorque;
extern double vofa_pitch;
extern double vofa_current;
extern double Adirlrmp;
extern double Adirout;
extern double Aerr;
extern uint16_t Ashooter_17mm_1_barrel_heat;
extern uint8_t Aid;
extern uint8_t Ashoot_hz;
extern float Ashoot_speed;
extern i16 Adrmp;
extern i16 Armp;

extern f32 Ax;
extern f32 Ay;
extern f32 Az;
extern f32 Gx;
extern f32 Gy;
extern f32 Gz;

extern void FreemasterDebug();
extern float Aautopitch;
extern float Aautoyaw;

extern AimbotFrame_SCM_t Aimbot;

class Gimbal {
 public:
  int abcdefg = 0;

  Buzzer *buzzer{nullptr};  // 蜂鸣器
  rm::modules::BuzzerController<rm::modules::buzzer_melody::Silent, rm::modules::buzzer_melody::Startup,
                                rm::modules::buzzer_melody::Success, rm::modules::buzzer_melody::Error,
                                rm::modules::buzzer_melody::SuperMario, rm::modules::buzzer_melody::SeeUAgain,
                                rm::modules::buzzer_melody::TheLick>
      buzzer_controller;  // 蜂鸣器控制器
  LED *led{nullptr};      // RGB LED灯
  rm::modules::RgbLedController<rm::modules::led_pattern::Off, rm::modules::led_pattern::RedFlash,
                                rm::modules::led_pattern::GreenBreath,
                                rm::modules::led_pattern::RgbFlow>
      led_controller;           // RGB LED控制器
  rm::hal::Can *can1{nullptr};  // CAN 总线接口
  rm::hal::SerialInterface *referee_uart;
  rm::device::RxReferee *rx_referee{nullptr};
  rm::hal::Serial *dbus{nullptr};              // 遥控器串口接口
  rm::device::DeviceManager<1> device_rc;      // 遥控管理器，维护所有设备在线状态
  rm::device::DeviceManager<2> device_gimbal;  // 云台管理器
  rm::device::DeviceManager<3> device_shoot;   // 发射管理器
  int time_ = 0;                               // 系统心跳

  rm::device::BMI088 *imu{nullptr};           // IMU
  rm::modules::MahonyAhrs ahrs{490.0f};       // TODO Mahony滤波控制频率
  rm::modules::MahonyAhrs ahrs_auto{490.0f};  // TODO Mahony滤波控制频率

  double yaw = 0;    // imu yaw数据
  double roll = 0;   // imu roll数据
  double pitch = 0;  // imu pitch数据

  rm::device::GM6020 *yaw_motor{nullptr};                                           // 云台 Yaw 上电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr};  // 云台 Pitch 电机
  rm::device::M3508 *friction_left{nullptr};                                        // 左侧摩擦轮电机
  rm::device::M3508 *friction_right{nullptr};                                       // 右侧摩擦轮电机
  rm::device::M2006 *dial_motor{nullptr};                                           // 拨盘电机

  rm::device::DR16 *rc{nullptr};  // 遥控器
  typedef enum {
    kNoForce,  // 无力
    kManual,   // 手动
    kAuto,     // 自瞄

    kStop,                                   // 无力
    kReady,                                  // 准备开火
    kFire                                    // 开火
  } StateMachineType;                        // 遥控器状态机
  StateMachineType AmmoState_ = {kNoForce};  // 发射机构状态
  StateMachineType GimbalState_ = {kStop};   // 云台运动状态
  double rc_yaw_data = 0;                    // 遥控器yaw数据
  double rc_pitch_data = 0;                  // 遥控器pitch数据

  bool DM_is_enable = false;     // 达秒使能标志位
  Gimbal2Dof gimbal_controller;  // 二轴双 Yaw 云台控制器
  Shoot2Fric shoot_controller;   // 双摩擦轮发射机构控制器
  i16 encoder_dirl = 0;
  bool single_shoot_mode = true;  // TODO 是否打开单发模式
  int single_shoot_time = 28;     // TODO 单发时间
  // int single_shoot_time=56;    //TODO 单发时间
  int single_shoot_mid = 0;  // 单发中间变量
  bool single_flag = 0;      // 单发射击标志位
  float dirl_speed = 5000;   // TODO 拨盘转速
  // float dirl_speed = 2500;
  float redirl_speed = 1000;    // TODO 拨盘反转速
  float friction_speed = 6600;  // TODO 摩擦轮转速
  // 拨盘自动反转
  float auto_reverse_buffer[5] = {1.f, 2.f, 3.f, 4.f, 5.f};  // TODO 缓存区大小
  int auto_reverse_time_max = 150;                           // TODO 反转持续时间
  int auto_reverse_time = 0;                                 // 持续时间变量
  bool auto_reverse_flag = false;                            // 反转标志位

  // 滚转补偿参数（用 yaw/pitch 组合抵消小角度 roll）
  bool roll_comp_enable = true;  // TODO 滚转补偿开关
  float roll_comp_kp = 0.1f;     // TODO 补偿系数，rad_pitch_per_rad_roll
  float roll_comp_limit = 0.3f;  // TODO 最大补偿幅度（rad）
  // imu和电机相位补偿滤波参数
  int err_buffer_size = 8;     // TODO 缓存大小
  int err_buffer_ptr = 0;      // 滤波器指针
  double err_imu_pitch = 0;    // 滤波器计算误差
  double err_sum = 0;          // 误差和
  double err_buffer[8] = {0};  // TODO 伪环形缓存
  double err_average = 0;      // 误差平均值
  // pitch补偿系数
  float pitch_torque = 0.0f;     // pitch电机力矩重力补偿量
  float pitch_torque_kp = 0.9f;  // TODO 重力补偿参数
  // // pitch滤波器（效果不好，未启用）
  // Biquad pitch_cmd_notch;
  // ChirpGenerator pitch_chirp;

  rm::device::Referee<rm::device::RefereeRevision::kV170> referee_data_buffer;  ///< 裁判系统数据缓冲区

  // 小角度 roll 补偿：将 roll 误差分解到 yaw/pitch
  std::pair<double, double> ApplyRollComp(double yaw_target, double pitch_target) {
    if (!roll_comp_enable) {
      return {yaw_target, pitch_target};
    }
    // 水平姿态 roll ≈ M_PI（上方 SubLoop500Hz 中做了 +M_PI）
    double roll_err = roll - 6.25;
    roll_err = rm::modules::Clamp(roll_err, -roll_comp_limit, roll_comp_limit);

    // 近似分解：机体 roll 对于当前朝向 yaw，投影到 yaw/pitch
    double yaw_correction = roll_comp_kp * roll_err * std::sin(yaw_target);
    double pitch_correction = -roll_comp_kp * roll_err * std::cos(yaw_target);

    double new_yaw = rm::modules::Wrap(yaw_target + yaw_correction, 0, 2 * M_PI);
    double new_pitch = rm::modules::Clamp(pitch_target + pitch_correction, 1.6, 2.9);
    return {new_yaw, new_pitch};
  }

  // 结构体初始化
  void GimbalInit() {
    buzzer = new Buzzer;
    led = new LED;

    can1 = new rm::hal::Can{hcan1};
    dbus = new rm::hal::Serial{huart3, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};

    imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};

    referee_uart = new rm::hal::Serial{huart6, 128, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
    rx_referee = new rm::device::RxReferee{*referee_uart};

    rc = new rm::device::DR16{*dbus};
    yaw_motor = new rm::device::GM6020{*can1, 6};
    pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
        *can1, {0x01, 0x07, 10.0f, 20.0f, 10.0f, {0.0f, 10.0f}, {0.0f, 5.0f}}};
    friction_left = new rm::device::M3508{*can1, 1};
    friction_right = new rm::device::M3508{*can1, 3};
    dial_motor = new rm::device::M2006{*can1, 5};

    device_rc << rc;                                                // 遥控器
    device_gimbal << yaw_motor << pitch_motor;                      // 云台电机
    device_shoot << friction_left << friction_right << dial_motor;  // 发射机构电机

    can1->SetFilter(0, 0);
    can1->Begin();
    rc->Begin();
    buzzer->Init();
    led->Init();
    rx_referee->Begin();
    led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
    buzzer_controller.Play<rm::modules::buzzer_melody::TheLick>();

    time_ = 0;

    GimbalPIDInit();
    AmmoPIDInit();

    gimbal_controller.Enable(false);
    pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);

    shoot_controller.Enable(false);                   // 开启控制器
    shoot_controller.Arm(false);                      // 摩擦轮武装（允许转动）
    shoot_controller.SetMode(Shoot2Fric::kFullAuto);  // 连发模式
    shoot_controller.SetLoaderSpeed(0.0f);            // 拨盘目标线速度
    shoot_controller.SetArmSpeed(0.0f);               // 摩擦轮目标线速度

    // pitch_cmd_notch.initNotch(250.0, 3.15, 15);//控制频率 陷频频率 品质因数
    // pitch_chirp.init(250, 0.5, 100.0, 20.0, 0.01, ChirpType::kLog);// 控制频率 扫频区间f0-f1 周期 幅度
  }

  // 云台pid初始化
  void GimbalPIDInit() {
    gimbal_controller.pid().yaw_position.SetKp(160.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(100000.0f).SetMaxIout(
        1000.0f);  // 200
    gimbal_controller.pid().yaw_speed.SetKp(350.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(25000.0f).SetMaxIout(
        1000.0f);  // 250

    gimbal_controller.pid().pitch_position.SetKp(30.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(500.0f).SetMaxIout(10.0f);
    gimbal_controller.pid().pitch_speed.SetKp(1.1f).SetKi(0.001f).SetKd(0.002f).SetMaxOut(10.0f).SetMaxIout(5.0f);
  }

  // 发射机构pid初始化
  void AmmoPIDInit() {
    shoot_controller.pid().fric_1_speed.SetKp(18.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(20000.0f).SetMaxIout(1000.0f);
    shoot_controller.pid().fric_2_speed.SetKp(18.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(20000.0f).SetMaxIout(1000.0f);
    shoot_controller.pid().loader_speed.SetKp(15.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(20000.0f).SetMaxIout(2000.0f);
  }

  // 遥控器状态更新
  void RCStateUpdate() {
    switch (rc->switch_r()) {
      case rm::device::DR16::SwitchPosition::kUp:
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
      case rm::device::DR16::SwitchPosition::kUp:
        GimbalState_ = kAuto;
        break;
      case rm::device::DR16::SwitchPosition::kMid:
        GimbalState_ = kManual;
        break;
      default:
        GimbalState_ = kNoForce;
        break;
    }
  }

  // 云台控制
  void GimbalControl() {
    // 手动控制
    if (GimbalState_ == kManual) {
      if (DM_is_enable == false) {  // 使达妙电机使能
        pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
        DM_is_enable = true;
        gimbal_controller.Enable(true);
        rc_yaw_data = yaw;
        rc_pitch_data = rm::modules::Wrap(pitch + err_average, 0, 2 * M_PI);  // 使用 IMU pitch 作为初始姿态
      }
      rc_yaw_data -= rm::modules::Map(rc->left_x(), -660, 660, -0.005f, 0.005f);
      rc_yaw_data = rm::modules::Wrap(rc_yaw_data, 0, 2 * M_PI);

      rc_pitch_data -= rm::modules::Map(rc->left_y(), -660, 660, -0.005f, 0.005f);
      rc_pitch_data = rm::modules::Clamp(rc_pitch_data, 1.6, 2.75);

      auto [comp_yaw, comp_pitch] = ApplyRollComp(rc_yaw_data, rc_pitch_data);
      gimbal_controller.SetTarget(comp_yaw, comp_pitch);

      gimbal_controller.Update(yaw, -yaw_motor->rpm(), rm::modules::Wrap(pitch + err_average, 0, 2 * M_PI),
                               pitch_motor->vel(), 2.f);

      pitch_torque = pitch_torque_kp * sin(pitch - 3.7);
      pitch_torque = rm::modules::Clamp(pitch_torque, -3, 3);
      yaw_motor->SetCurrent(rm::modules::Clamp(-gimbal_controller.output().yaw, -25000, 25000));

    }

    // // 自瞄控制 偏差角
    // else if (GimbalState_ == kAuto) {
    //   if (DM_is_enable == false) {  // 使达妙电机使能
    //     pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    //     DM_is_enable = true;
    //     gimbal_controller.Enable(true);
    //     rc_yaw_data = yaw;
    //     rc_pitch_data = rm::modules::Wrap(pitch + err_average, 0, 2 * M_PI);  // 使用 IMU pitch 作为初始姿态
    //   }
    //
    //   if (Aimbot.AimbotState) {
    //     rc_yaw_data += Aimbot.TargetYawAngle;
    //     rc_yaw_data = rm::modules::Wrap(rc_yaw_data, 0, 2 * M_PI);
    //
    //     rc_pitch_data += Aimbot.TargetPitchAngle;
    //     rc_pitch_data = rm::modules::Clamp(rc_pitch_data, 1.6, 2.7);
    //   } else {
    //     rc_yaw_data += rm::modules::Map(rc->left_x(), -660, 660, -0.005f, 0.005f);
    //     rc_yaw_data = rm::modules::Wrap(rc_yaw_data, 0, 2 * M_PI);
    //
    //     rc_pitch_data -= rm::modules::Map(rc->left_y(), -660, 660, -0.005f, 0.005f);
    //     rc_pitch_data = rm::modules::Clamp(rc_pitch_data, 1.6, 2.9);
    //   }
    //
    //   auto [comp_yaw, comp_pitch] = ApplyRollComp(rc_yaw_data, rc_pitch_data);
    //
    //   gimbal_controller.SetTarget(comp_yaw, comp_pitch);
    //   gimbal_controller.Update(yaw, yaw_motor->rpm(), rm::modules::Wrap(pitch + err_average, 0, 2 * M_PI),
    //                            pitch_motor->vel(), 2.f);
    //
    //   pitch_torque = -pitch_torque_kp * sin(pitch -3.7);
    //   pitch_torque = rm::modules::Clamp(pitch_torque, -3, 3);
    //   yaw_motor->SetCurrent(rm::modules::Clamp(gimbal_controller.output().yaw, -30000, 30000));
    // }

    // 自瞄控制 直接控制
    else if (GimbalState_ == kAuto) {
      if (DM_is_enable == false) {  // 使达妙电机使能
        pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
        DM_is_enable = true;
        gimbal_controller.Enable(true);
        rc_yaw_data = yaw;
        rc_pitch_data = rm::modules::Wrap(pitch + err_average, 0, 2 * M_PI);  // 使用 IMU pitch 作为初始姿态
      }

      if (Aimbot.AimbotState) {
        rc_yaw_data = Aimbot.TargetYawAngle;
        rc_yaw_data = rm::modules::Wrap(rc_yaw_data, 0, 2 * M_PI);

        rc_pitch_data = rm::modules::Wrap(Aimbot.TargetPitchAngle + err_average, 0, 2 * M_PI);
        rc_pitch_data = rm::modules::Clamp(rc_pitch_data, 1.6, 2.75);
      } else {
        rc_yaw_data -= rm::modules::Map(rc->left_x(), -660, 660, -0.005f, 0.005f);
        rc_yaw_data = rm::modules::Wrap(rc_yaw_data, 0, 2 * M_PI);

        rc_pitch_data -= rm::modules::Map(rc->left_y(), -660, 660, -0.005f, 0.005f);
        rc_pitch_data = rm::modules::Clamp(rc_pitch_data, 1.6, 2.75);
      }

      auto [comp_yaw, comp_pitch] = ApplyRollComp(rc_yaw_data, rc_pitch_data);
      gimbal_controller.SetTarget(comp_yaw, comp_pitch);

      gimbal_controller.Update(yaw, -yaw_motor->rpm(), rm::modules::Wrap(pitch + err_average, 0, 2 * M_PI),
                               pitch_motor->vel(), 2.f);

      pitch_torque = pitch_torque_kp * sin(pitch - 3.7);
      pitch_torque = rm::modules::Clamp(pitch_torque, -3, 3);
      yaw_motor->SetCurrent(rm::modules::Clamp(-gimbal_controller.output().yaw, -25000, 25000));
    }

    // 无力或遥控器无信号
    else {
      if (DM_is_enable == true) {  // 使达妙电机使能
        pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
        DM_is_enable = false;
        gimbal_controller.Enable(false);
      }
      yaw_motor->SetCurrent(0);
      pitch_torque = 0;
    }
  }

  // 发射机构控制
  void AmmoControl() {
    // 发射状态
    if (AmmoState_ == kFire) {
      shoot_controller.Enable(true);
      shoot_controller.Arm(true);
      shoot_controller.SetMode(Shoot2Fric::kFullAuto);

      if (single_shoot_mode)  // 单发模式逻辑
      {
        if (encoder_dirl < 550 && rc->dial() >= 550) single_flag = true;
        encoder_dirl = rc->dial();

        if (single_flag) {
          if (single_shoot_mid >= single_shoot_time) {
            single_flag = false;
            single_shoot_mid = 0;
          } else {
            shoot_controller.SetLoaderSpeed(dirl_speed);
            single_shoot_mid++;
          }
        } else {
          shoot_controller.SetLoaderSpeed(0);
        }
      } else  // 连发模式逻辑
      {
        // 正常控制逻辑
        if (rc->dial() >= 550) {
          if (auto_reverse_flag) {
            shoot_controller.SetLoaderSpeed(-redirl_speed);
            auto_reverse_time--;
            auto_reverse_time < 1 ? auto_reverse_flag = false : auto_reverse_flag = true;
          } else {
            if (GimbalState_ == kAuto) {
              if (Aimbot.AimbotState && Aimbot.AutoFire) {
                shoot_controller.SetLoaderSpeed(dirl_speed);
              } else if (Aimbot.AimbotState && !Aimbot.AutoFire) {
                shoot_controller.SetLoaderSpeed(0.0f);
              } else {
                shoot_controller.SetLoaderSpeed(dirl_speed);
              }
            } else {
              shoot_controller.SetLoaderSpeed(dirl_speed);
            }
          }
        } else if (rc->dial() <= -600) {
          shoot_controller.SetLoaderSpeed(-redirl_speed);
        } else {
          shoot_controller.SetLoaderSpeed(0.0f);
        }

        // 自动反转逻辑
        if (shoot_controller.GetLoaderSpeed() == dirl_speed) {
          auto_reverse_buffer[4] = auto_reverse_buffer[3];
          auto_reverse_buffer[3] = auto_reverse_buffer[2];
          auto_reverse_buffer[2] = auto_reverse_buffer[1];
          auto_reverse_buffer[1] = auto_reverse_buffer[0];
          auto_reverse_buffer[0] = dial_motor->encoder();
          if (auto_reverse_buffer[0] == auto_reverse_buffer[4]) {
            auto_reverse_flag = true;
            auto_reverse_time = auto_reverse_time_max;
          }
        }
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
      dial_motor->SetCurrent(0);
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

  void Referee_control() {}

  // 遥控器和imu数据解算+DjiMotor发信息
  void SubLoop500Hz() {
    RCStateUpdate();  // 遥控器更新
    GimbalControl();  // 云台控制更新
    AmmoControl();    // 发射机构数据更新
    rm::device::DjiMotor<>::SendCommand(*can1);

    // imu处理
    imu->Update();

    ahrs_auto.Update(rm::modules::ImuData6Dof{imu->gyro_y(), imu->gyro_x(), -imu->gyro_z(), imu->accel_y(),
                                              imu->accel_x(), -imu->accel_z()});
    pitch = ahrs_auto.euler_angle().pitch + M_PI;
    yaw = ahrs_auto.euler_angle().yaw + M_PI;
    roll = ahrs_auto.euler_angle().roll + M_PI;

    GimbalImuSend(ahrs_auto.quaternion().w, ahrs_auto.quaternion().x, ahrs_auto.quaternion().y,
                  ahrs_auto.quaternion().z);
  }

  // DmMotor电机发信息
  void SubLoop250Hz() {
    if (time_ % 2 == 0) {
      double pitch_torque_cmd = gimbal_controller.output().pitch + pitch_torque;
      pitch_torque_cmd = rm::modules::Clamp(pitch_torque_cmd, -10.0, 10.0);
      pitch_motor->SetPosition(0, 0, pitch_torque_cmd, 0, 0);

      err_sum -= err_buffer[err_buffer_ptr];
      err_imu_pitch = pitch_motor->pos() - pitch;
      err_imu_pitch = rm::modules::Wrap(err_imu_pitch, 0, 2 * M_PI);
      err_buffer[err_buffer_ptr] = err_imu_pitch;
      err_sum += err_imu_pitch;
      err_buffer_ptr = (err_buffer_ptr + 1) % err_buffer_size;
      err_average = err_sum / err_buffer_size;
      err_average = rm::modules::Wrap(err_average, 0, 2 * M_PI);
    }
  }

  // 调试
  void SubLoop100Hz() {
    if (time_ % 5 == 0) {
      FreemasterDebug();
      // vofa_pitch = Apitchpose;
      // uint32_t t_ms = HAL_GetTick();
      // VOFA_SendPitch_Blocking(t_ms, vofa_pitch, vofa_current);
    }
  }

  // 裁判系统+UI绘制
  void SubLoop50Hz() {
    if (time_ % 10 == 0) {
    }
  }

  // 总循环
  void SubLoop10Hz() {
    if (time_ % 50 == 0) {
      if (abcdefg >= 30) abcdefg = 0;
      if (abcdefg >= 0 && abcdefg < 10) Set_LED(0, 255, 0, 0);
      if (abcdefg >= 10 && abcdefg < 20) Set_LED(0, 0, 0, 255);
      if (abcdefg >= 20 && abcdefg < 30) Set_LED(0, 0, 255, 0);
      abcdefg++;
      Set_Brightness(10);
      WS2812_Send();

      time_ = 0;
    }
  }
};

#endif  // BOARDC_GIMBAL_HPP