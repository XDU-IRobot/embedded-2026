#ifndef MAIN_HPP
#define MAIN_HPP

#include <librm.hpp>

#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "device_manager.hpp"
#include "controllers/gimbal_double_yaw.hpp"
#include "controllers/quad_steering_chassis.hpp"
#include "controllers/shoot_3fric.hpp"
#include "controllers/counter.hpp"

#include "USB.hpp"
#include "Referee.hpp"

// 状态机
typedef enum {
  kUnable = 0u,  // 断电模式
  kNoForce,      // 无力模式
  kTest,         // 调试模式
  kMatch,        // 比赛模式

  kGbRemote,    // 云台遥控模式
  kGbScan,      // 扫描模式
  kGbNavigate,  // 云台导航模式
  kGbAimbot,    // 云台自瞄模式

  kCsRemote,    // 底盘遥控模式
  kCsNavigate,  // 底盘导航模式
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
  rm::hal::Can *can1{nullptr}, *can2{nullptr};  ///< CAN 总线接口
  rm::hal::Serial *dbus{nullptr};               ///< 遥控器串口接口
  rm::hal::Serial *referee_uart{nullptr};       ///< 裁判系统串口接口

  // 设备 //
  DeviceManager<1> device_rc;  ///< 设备管理器，维护所有设备在线状态
  DeviceManager<3> device_gimbal;
  DeviceManager<8> device_chassis;
  DeviceManager<3> device_shoot;
  // 云台
  rm::device::BMI088 *imu{nullptr};                                                    ///< IMU
  rm::device::DR16 *rc{nullptr};                                                       ///< 遥控器
  rm::device::RxReferee *rx_referee{nullptr};                                          ///< 裁判系统
  rm::device::GM6020 *up_yaw_motor{nullptr};                                           ///< 云台 Yaw 上电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *down_yaw_motor{nullptr};  ///< 云台 Yaw 下电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr};     ///< 云台 Pitch 电机
  rm::device::M3508 *friction_left{nullptr};                                           ///< 左侧摩擦轮电机
  rm::device::M3508 *friction_right{nullptr};                                          ///< 右侧摩擦轮电机
  rm::device::M2006 *dial_motor{nullptr};                                              ///< 拨盘电机
  // 底盘
  rm::device::GM6020 *steer_lf{nullptr};  ///< 左前舵电机
  rm::device::GM6020 *steer_rf{nullptr};  ///< 右前舵电机
  rm::device::GM6020 *steer_lb{nullptr};  ///< 左后舵电机
  rm::device::GM6020 *steer_rb{nullptr};  ///< 右后舵电机
  rm::device::M3508 *wheel_lf{nullptr};   ///< 左前轮电机
  rm::device::M3508 *wheel_rf{nullptr};   ///< 右前轮电机
  rm::device::M3508 *wheel_lb{nullptr};   ///< 左后轮电机
  rm::device::M3508 *wheel_rb{nullptr};   ///< 右后轮电机

  rm::device::Referee<rm::device::RefereeRevision::kV170> *referee_data_buffer{nullptr};  ///< 裁判系统数据缓冲区

  // 控制器 //
  rm::modules::MahonyAhrs ahrs{1000.0f};                  ///< 姿态解算器
  GimbalDoubleYaw gimbal_controller;                      ///< 二轴双 Yaw 云台控制器
  QuadSteeringChassis chassis_controller{0.0f, 0.4714f};  ///< 四轮转向底盘控制器
  Shoot3Fric shoot_controller{8, 36.0f};                  ///< 三摩擦轮发射机构控制器，8发拨盘
  Counter dail_position_counter{0.0, 8191.0f};            ///< 云台 Yaw 下部电机位置计数器

  // USB //
  GimbalDataFrame_SCM_t GimbalData;    ///< IMU数据
  RefereeDataFrame_SCM_t RefereeData;  ///< 裁判系统数据
  AimbotFrame_SCM_t Aimbot;            ///< 自瞄数据
  NucControlFrame_SCM_t NucControl;    ///< NUC控制数据

  StateMachineType StateMachine_ = {kNoForce};                                               // 当前状态
  u_int8_t time = 0;                                                                         // 时间
  u_int8_t music = false;                                                                    // 控制音乐播放
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
} *globals;

#endif  // MAIN_HPP
