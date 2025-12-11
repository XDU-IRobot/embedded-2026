#ifndef MAIN_HPP
#define MAIN_HPP

#include <librm.hpp>

#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "controllers/gimbal_2dof.hpp"

#include "aimbot_comm_can.hpp"
#include "USB.hpp"

// 状态机
typedef enum {
  kNoForce,  // 无力模式
  kTest,     // 调试模式

  kGbRemote,  // 云台遥控模式
  kGbAimbot,  // 云台自瞄模式
} StateMachineType;

inline struct GlobalWarehouse {
 public:
  Buzzer *buzzer{nullptr};  ///< 蜂鸣器
  rm::modules::BuzzerController<rm::modules::buzzer_melody::Silent, rm::modules::buzzer_melody::Startup,
                                rm::modules::buzzer_melody::Success, rm::modules::buzzer_melody::Error,
                                rm::modules::buzzer_melody::SuperMario, rm::modules::buzzer_melody::SeeUAgain,
                                rm::modules::buzzer_melody::TheLick>
      buzzer_controller;
  LED *led{nullptr};  ///< RGB LED灯
  rm::modules::RgbLedController<rm::modules::led_pattern::Off, rm::modules::led_pattern::RedFlash,
                                rm::modules::led_pattern::GreenBreath, rm::modules::led_pattern::RgbFlow>
      led_controller;  ///< RGB LED控制器

  // 硬件接口 //
  rm::hal::Can *can1{nullptr}, *can2{nullptr};             ///< CAN 总线接口
  rm::hal::Serial *dbus{nullptr};                          ///< 遥控器串口接口
  rm::device::CanCommunicator *can_communicator{nullptr};  ///< CAN 通信器
  rm::hal::Serial *imu_uart{nullptr};                      ///< imu串口接口

  // 设备 //
  rm::device::DeviceManager<1> device_rc;  ///< 设备管理器，维护所有设备在线状态
  rm::device::DeviceManager<2> device_gimbal;
  // 云台
  rm::device::BMI088 *imu{nullptr};                                                 ///< IMU
  rm::device::HipnucImu *hipnuc_imu{nullptr};                                       ///< IMU
  rm::device::DR16 *rc{nullptr};                                                    ///< 遥控器
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *yaw_motor{nullptr};    ///< 云台 Yaw 电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr};  ///< 云台 Pitch 电机

  // 控制器 //
  rm::modules::MahonyAhrs ahrs{500.0f};  ///< 姿态解算器
  Gimbal2Dof gimbal_controller;          ///< 二轴双 Yaw 云台控制器

  // USB //
  GimbalDataFrame_SCM_t GimbalData{0, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0};  ///< IMU数据
  AimbotFrame_SCM_t Aimbot{0, 0, 0, 0, 0.0f, 0.0f, 0.0f, 0};                   ///< 自瞄数据

  StateMachineType StateMachine_ = {kNoForce};  // 当前状态
  u_int8_t time_ = 0;                           // 主程序计数器
  u_int8_t aim_mode = 0;                        // 自瞄模式
  u_int8_t time_camera = 0;                     // 摄像头计数器
  u_int16_t imu_count = 0;                      // IMU计数器
  const float yaw_gyro_bias_ = 0.0015f;         // 偏航角（角度值）的陀螺仪偏移量
  const float rc_max_value_ = 660.0f;           // 遥控器最大值
  const float GM6020_encoder_max_ = 8191.0f;    // GM6020 电机编码器最大值

  // 函数 //
 public:
  void Init();

  void RxCallback();

  void SubLoop500Hz();

  void SubLoop250Hz();

  void SubLoop100Hz();

  void SubLoop50Hz();

  void SubLoop10Hz();

 private:
  void GimbalPIDInit();

  void ChassisPIDInit();

  void ShootPIDInit();

  void RCStateUpdate();
} *globals;

#endif  // MAIN_HPP
