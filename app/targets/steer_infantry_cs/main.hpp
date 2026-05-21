#ifndef MAIN_HPP
#define MAIN_HPP

#include <librm.hpp>

#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "controllers/quad_steering_chassis.hpp"

#include "GimbalCommunicator.hpp"
#include "Referee.hpp"

// 状态机
typedef enum {
  kUnable = 0u,  // 断电模式
  kNoForce,      // 无力模式
  kTest,         // 调试模式
  kMatch,        // 比赛模式

  kFollow,    // 跟随模式
  kRotate,    // 旋转模式
  kReRotate,  // 反旋转模式

  kNormalSpeed,  // 正常速度模式
  kHighSpeed,    // 高速模式

  kNormal,  // 正常状态
  kDaFu,    // 大符状态
  kXiaoFu,  // 小符状态
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
  rm::hal::ThrottledCan<128, rm::modules::SchedulingPolicy::kFifo> *can1{nullptr}, *can2{nullptr};  ///< CAN 总线接口
  rm::device::GimbalCommunicator *gimbal_communicator{nullptr};                                     ///< CAN 通信器
  rm::device::GkSupercap *super_cap{nullptr};                                                       ///< 港科超级电容
  rm::device::BMI088 *imu{nullptr};                                                                 ///< IMU
  rm::hal::Serial<128> *referee_uart{nullptr};                                        ///< 裁判系统串口接口
  rm::device::RxReferee *rx_referee{nullptr};                                         ///< 裁判系统
  rm::device::Referee<rm::device::RefereeRevision::kNewV110> *referee_data{nullptr};  ///< 裁判系统数据

  // 设备 //
  rm::device::DeviceManager<1> device_gimbal;
  rm::device::DeviceManager<8> device_chassis;
  // 云台
  rm::device::GM6020 *yaw_motor{nullptr};
  // 底盘
  rm::device::GM6020 *steer_lf{nullptr};  ///< 左前轮舵电机
  rm::device::GM6020 *steer_rf{nullptr};  ///< 右前轮舵电机
  rm::device::GM6020 *steer_lb{nullptr};  ///< 左后轮舵电机
  rm::device::GM6020 *steer_rb{nullptr};  ///< 右后轮舵电机
  rm::device::M3508 *wheel_lf{nullptr};   ///< 左前轮电机
  rm::device::M3508 *wheel_rf{nullptr};   ///< 右前轮电机
  rm::device::M3508 *wheel_lb{nullptr};   ///< 左后轮电机
  rm::device::M3508 *wheel_rb{nullptr};   ///< 右后轮电机                                    ///< 拨盘电机

  // 控制器 //
  rm::modules::MahonyAhrs ahrs{500.0f};                    ///< 姿态解算器
  QuadSteeringChassis chassis_controller{0.0f, 0.45368f};  ///< 四轮转向底盘控制器
  rm::device::GkSupercap::TxData super_cap_tx{};

  uint8_t time{};                    // 时间
  u_int16_t hurt_time{};             // 受伤小陀螺倒计时
  uint8_t time_camera{};             // 摄像头计数器
  u_int16_t imu_count{};             // IMU计数器
  uint8_t aim_mode{};                // 自瞄模式
  uint8_t music_choice{};            // 音乐选择
  u_int16_t current_heat{};          // 当前热量
  u_int16_t heat_limit{};            // 热量上限
  uint8_t power_state{};             // 供能状态
  u_int16_t remain_bullet_number{};  // 剩余子弹数量
  bool music_play_flag = false;      // 控制音乐播放
  bool music_change_flag = false;    // 音乐改动标识位
  bool ui_send_choice = false;       // ui发送选择

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
