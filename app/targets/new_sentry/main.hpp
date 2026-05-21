#ifndef MAIN_HPP
#define MAIN_HPP

#include <librm.hpp>

#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "encoder_counter.hpp"
#include "aimbot_comm_can.hpp"
#include "navigate_comm_can.hpp"
#include "controllers/gimbal_double_yaw.hpp"
#include "controllers/quad_omni_chassis.hpp"
#include "controllers/shoot_3fric.hpp"

#include "Referee.hpp"
#include "WFLY.hpp"

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
  kGbPercept,   // 云台感知模式

  kCsRemote,    // 底盘遥控模式
  kCsNavigate,  // 底盘导航模式
  kGbIdentify,
  kGbFfVerify,
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
  rm::device::AimbotCanCommunicator *aimbot_communicator{nullptr};                                  ///< CAN 通信器
  rm::device::NavigateCanCommunicator *navigate_communicator{nullptr};                              ///< CAN 通信器
  rm::device::HipnucImuCan *hipnuc_imu{nullptr};                                                    ///< IMU
  rm::device::BMI088 *imu{nullptr};                                                                 ///< IMU

  rm::hal::Serial<128> *ident_uart{nullptr};
  rm::hal::Serial<25> *dbus{nullptr};  ///< 遥控器串口接口
  WflyET16s *wfly_et16s{nullptr};      ///< 天地飞遥控器

  rm::hal::Serial<128> *referee_uart{nullptr};                                        ///< 裁判系统串口接口
  rm::device::RxReferee *rx_referee{nullptr};                                         ///< 裁判系统
  rm::device::Referee<rm::device::RefereeRevision::kNewV110> *referee_data{nullptr};  ///< 裁判系统数据

  // 云台
  rm::device::GM6020 *up_yaw_motor{nullptr};                                           ///< 云台 Yaw 上电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *down_yaw_motor{nullptr};  ///< 云台 Yaw 下电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr};     ///< 云台 Pitch 电机
  rm::device::M3508 *friction_left{nullptr};                                           ///< 左侧摩擦轮电机
  rm::device::M3508 *friction_right{nullptr};                                          ///< 右侧摩擦轮电机
  rm::device::M3508 *dial_motor{nullptr};                                              ///< 拨盘电机
  // 底盘
  rm::device::M3508 *wheel_lf{nullptr};  ///< 左前轮电机
  rm::device::M3508 *wheel_rf{nullptr};  ///< 右前轮电机
  rm::device::M3508 *wheel_lb{nullptr};  ///< 左后轮电机
  rm::device::M3508 *wheel_rb{nullptr};  ///< 右后轮电机

  // 设备 //
  rm::device::DeviceManager<1> device_rc;  ///< 设备管理器，维护所有设备在线状态
  rm::device::DeviceManager<2> device_nuc;
  rm::device::DeviceManager<3> device_gimbal;
  rm::device::DeviceManager<3> device_shoot;
  rm::device::DeviceManager<4> device_chassis;

  // 控制器 //
  rm::modules::MahonyAhrs ahrs{500.0f};            ///< 姿态解算器
  GimbalDoubleYaw gimbal_controller;               ///< 二轴双 Yaw 云台控制器
  QuadOmniChassis chassis_controller;              ///< 四轮转向底盘控制器
  Shoot3Fric shoot_controller{9, 17.0666f, true};  ///< 三摩擦轮发射机构控制器，8发拨盘
  EncoderCounter dail_encoder_counter;             ///< 云台 Yaw 下部电机位置计数器

  StateMachineType StateMachine_ = {kNoForce};  // 当前状态
  SwitchPosition last_switch[7] = {
      SwitchPosition::kDown, SwitchPosition::kDown, SwitchPosition::kDown, SwitchPosition::kDown,
      SwitchPosition::kUp,   SwitchPosition::kUp,   SwitchPosition::kUp,
  };  // 上次拨杆位置

  uint8_t time = 0;                 // 时间
  uint16_t gimbal_init_time = 0;    // 云台初始化时间
  uint16_t shooter_init_time = 0;   // 发射机构初始化时间
  uint16_t chassis_init_time = 0;   // 底盘初始化时间
  uint16_t hurt_time = 0;           // 受伤小陀螺倒计时
  uint8_t time_camera = 0;          // 摄像头计数器
  uint16_t imu_count = 0;           // IMU计数器
  uint8_t aim_mode = 0;             // 自瞄模式
  uint8_t music_choice = 0;         // 音乐选择
  uint8_t music = 0;                // 控制音乐播放
  bool music_change_flag = false;   // 音乐改动标识位
  bool last_gimbal_power = false;   // 上一次云台电机使能状态
  bool last_shooter_power = false;  // 上一次发射机构电机使能状态
  bool last_chassis_power = false;  // 上一次底盘电机使能状态

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
