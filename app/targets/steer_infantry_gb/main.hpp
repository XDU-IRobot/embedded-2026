#ifndef MAIN_HPP
#define MAIN_HPP

#include <librm.hpp>

#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "encoder_counter.hpp"
#include "controllers/gimbal_2dof.hpp"
#include "controllers/shoot_3fric.hpp"

#include "aimbot_comm_can.hpp"
#include "ChassisCommunicator.hpp"
#include "Referee.hpp"

// 状态机
typedef enum {
  kUnable = 0u,  // 断电模式
  kNoForce,      // 无力模式
  kTest,         // 调试模式
  kMatch,        // 比赛模式

  kGbRemote,    // 云台遥控模式
  kGbAimbot,    // 云台自瞄模式
  kGbAimbotFu,  // 云台打符模式
  kGbIdentify,
  kGbFfVerify,
} StateMachineType;

inline struct GlobalWarehouse {
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
  rm::hal::Can *can1{nullptr}, *can2{nullptr};                      ///< CAN 总线接口
  rm::hal::Serial *dbus{nullptr};                                   ///< 遥控器串口接口
  rm::device::AimbotCanCommunicator *aimbot_communicator{nullptr};  ///< CAN 通信器
  rm::device::ChassisCommunicator *chassis_communicator{nullptr};   ///< CAN 通信器
  rm::device::GkSupercap *super_cap{nullptr};                       ///< 超级电容接口
  rm::hal::Serial *referee_uart{nullptr};                           ///< 裁判系统串口接口
  rm::hal::Serial *ident_uart{nullptr};
  rm::device::RxReferee *rx_referee{nullptr};                       ///< 裁判系统接口
  rm::device::VT03 *image_data{nullptr};                            ///< 裁判系统数据缓冲区

  // 设备 //
  rm::device::DeviceManager<1> device_rc;  ///< 设备管理器，维护所有设备在线状态
  rm::device::DeviceManager<2> device_gimbal;
  rm::device::DeviceManager<3> device_shoot;
  rm::device::DeviceManager<1> device_nuc;
  rm::device::DeviceManager<1> device_referee;

  // 云台
  rm::device::BMI088 *imu{nullptr};                                                 ///< IMU
  rm::device::DR16 *rc{nullptr};                                                    ///< 遥控器
  rm::device::GM6020 *yaw_motor{nullptr};                                           ///< 云台 Yaw 上电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr};  ///< 云台 Pitch 电机
  rm::device::M3508 *friction_left{nullptr};                                        ///< 左侧摩擦轮电机
  rm::device::M3508 *friction_right{nullptr};                                       ///< 右侧摩擦轮电机
  rm::device::M3508 *dial_motor{nullptr};                                           ///< 拨盘电机

  // 控制器 //
  rm::modules::MahonyAhrs ahrs{500.0f};         ///< 姿态解算器
  Gimbal2Dof gimbal_controller;                 ///< 二轴双 Yaw 云台控制器
  Shoot3Fric shoot_controller{9, 19.2f, true};  ///< 三摩擦轮发射机构控制器，8发拨盘
  EncoderCounter dail_encoder_counter;          ///< 拨盘电机位置计数器

  StateMachineType StateMachine_ = {kNoForce};  // 当前状态

  uint8_t time = 0;                   // 时间
  uint16_t init_time = 1000;          // 初始化时间
  uint16_t hurt_time = 0;             // 受伤小陀螺倒计时
  uint8_t music_choice = 0;           // 音乐选择
  uint8_t aim_mode = 0x01;            // 自瞄模式
  uint8_t time_camera = 0;            // 摄像头计数器
  uint16_t imu_count = 0;             // IMU计数器
  float chassis_move_x = 0;           // 底盘x轴目标速度
  float chassis_move_y = 0;           // 底盘y轴目标速度
  uint8_t chassis_state = 0;          // 底盘状态
  uint8_t ui_refresh_flag = 0;        // UI状态改变标志位
  uint8_t get_target_flag = 0;        // 有无目标标志位
  uint8_t suggest_fire_flag = 0;      // 建议开火标志位
  int8_t aim_speed_change = 0;        // 弹速调整标志位
  int8_t aim_speed_change_flag = 0;   // 弹速调整标志位
  bool aim_mood_change_flag = false;  // 自瞄模式切换标识位
  bool image_update_flag = false;     // 裁判系统数据更新标志位
  bool music_play_flag = false;       // 音乐播放标识位
  bool music_change_flag = false;     // 音乐改动标识位
  bool speed_change_flag = false;     // 速度调整标志位
  bool df_flag = false;               // 大符标志位
  bool df_state = false;              // 大符状态
  bool xf_flag = false;               // 小符标志位
  bool xf_state = false;              // 小符状态

  rm::device::DR16::SwitchPosition last_switch_l = rm::device::DR16::SwitchPosition::kDown;  // 左拨杆上一次状态
  rm::device::DR16::SwitchPosition last_switch_r = rm::device::DR16::SwitchPosition::kDown;  // 右拨杆上一次状态

  // 函数 //
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

  void ChassisStateUpdate();

  void Music();
} *globals;

#endif  // MAIN_HPP
