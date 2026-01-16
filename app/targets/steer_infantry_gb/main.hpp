#ifndef MAIN_HPP
#define MAIN_HPP

#include <librm.hpp>

#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "encoder_counter.hpp"
#include "controllers/gimbal_2dof.hpp"
#include "controllers/shoot_3fric.hpp"

#include "aimbot_comm_can.hpp"

// 状态机
typedef enum {
  kUnable = 0u,  // 断电模式
  kNoForce,      // 无力模式
  kTest,         // 调试模式

  kGbRemote,  // 云台遥控模式
  kGbAimbot,  // 云台自瞄模式
} StateMachineType;

inline struct GlobalWarehouse {
 public:
  Buzzer *buzzer{nullptr};  ///< 蜂鸣器
  rm::modules::BuzzerController<rm::modules::buzzer_melody::Silent, rm::modules::buzzer_melody::Startup,
                                rm::modules::buzzer_melody::Success, rm::modules::buzzer_melody::Error,
                                rm::modules::buzzer_melody::SuperMario, rm::modules::buzzer_melody::SeeUAgain,
                                rm::modules::buzzer_melody::TheLick, rm::modules::buzzer_melody::Beeps<1>>
      buzzer_controller;
  LED *led{nullptr};  ///< RGB LED灯
  rm::modules::RgbLedController<rm::modules::led_pattern::Off, rm::modules::led_pattern::RedFlash,
                                rm::modules::led_pattern::GreenBreath, rm::modules::led_pattern::RgbFlow>
      led_controller;  ///< RGB LED控制器

  // 硬件接口 //
  rm::hal::Can *can1{nullptr}, *can2{nullptr};                   ///< CAN 总线接口
  rm::hal::Serial *dbus{nullptr};                                ///< 遥控器串口接口
  rm::device::AimbotCanCommunicator *can_communicator{nullptr};  ///< CAN 通信器
  rm::hal::Serial *referee_uart{nullptr};                        ///< 裁判系统串口接口

  // 设备 //
  rm::device::DeviceManager<1> device_rc;  ///< 设备管理器，维护所有设备在线状态
  rm::device::DeviceManager<2> device_gimbal;
  rm::device::DeviceManager<3> device_shoot;
  rm::device::DeviceManager<1> device_nuc;

  // 云台
  rm::device::BMI088 *imu{nullptr};                                                 ///< IMU
  rm::device::DR16 *rc{nullptr};                                                    ///< 遥控器
  rm::device::GM6020 *yaw_motor{nullptr};                                           ///< 云台 Yaw 上电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr};  ///< 云台 Pitch 电机
  rm::device::M3508 *friction_left{nullptr};                                        ///< 左侧摩擦轮电机
  rm::device::M3508 *friction_right{nullptr};                                       ///< 右侧摩擦轮电机
  rm::device::M3508 *dial_motor{nullptr};                                           ///< 拨盘电机

  // 控制器 //
  rm::modules::MahonyAhrs ahrs{500.0f};   ///< 姿态解算器
  Gimbal2Dof gimbal_controller;           ///< 二轴双 Yaw 云台控制器
  Shoot3Fric shoot_controller{8, 36.0f};  ///< 三摩擦轮发射机构控制器，8发拨盘
  EncoderCounter dail_encoder_counter;    ///< 拨盘电机位置计数器

  StateMachineType StateMachine_ = {kNoForce};                                               // 当前状态
  u_int8_t time = 0;                                                                         // 时间
  u_int16_t hurt_time = 0;                                                                   // 受伤小陀螺倒计时
  u_int8_t music_choice = 0;                                                                 // 音乐选择
  u_int8_t aim_mode = 0;                                                                     // 自瞄模式
  u_int8_t time_camera = 0;                                                                  // 摄像头计数器
  u_int16_t imu_count = 0;                                                                   // IMU计数器
  u_int32_t imu_time = 0;                                                                    // INU解算时的时间戳
  bool music = false;                                                                        // 控制音乐播放
  bool music_change_flag = false;                                                            // 音乐改动标识位
  bool USB_selection = false;                                                                // 选择发送不同的usb数据
  rm::device::DR16::SwitchPosition last_switch_l = rm::device::DR16::SwitchPosition::kDown;  // 左拨杆上一次状态
  rm::device::DR16::SwitchPosition last_switch_r = rm::device::DR16::SwitchPosition::kDown;  // 右拨杆上一次状态
  float up_yaw_qw = 0.0f, up_yaw_qx = 0.0f, up_yaw_qy = 0.0f, up_yaw_qz = 0.0f;              // 云台上部Yaw电机的四元数
  const float yaw_gyro_bias_ = 0.0015f;       // 偏航角（角度值）的陀螺仪偏移量
  const float rc_max_value_ = 660.0f;         // 遥控器最大值
  const float GM6020_encoder_max_ = 8191.0f;  // GM6020 电机编码器最大值

  // 函数 //
 public:
  void Init();

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

  void Music();
} *globals;

#endif  // MAIN_HPP
