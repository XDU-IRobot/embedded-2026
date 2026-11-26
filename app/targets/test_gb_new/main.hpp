#pragma once

#include <librm.hpp>

#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "controllers/gimbal_2dof.hpp"

#include "aimbot_comm_can.hpp"
#include "pitch_gravity_2d.hpp"
#include "wave_generator.hpp"

using namespace rm;

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

enum class RemoteControlSource : u8 {
  kNone = 0,
  kDt17,
};

struct RemoteInput {
  f32 left_x{};
  f32 left_y{};
  f32 right_x{};
  f32 right_y{};
  f32 dial{};
  i16 mouse_x{};
  i16 mouse_y{};
  i16 mouse_z{};
  u16 keyboard{};
  bool mouse_left{};
  bool mouse_right{};

  [[nodiscard]] bool key(const u16 mask) const { return (keyboard & mask) != 0; }
};

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
  rm::hal::Can *can1{nullptr};                                      ///< 云台
  rm::hal::Can *can2{nullptr};                                      ///< 自瞄 CAN 总线接口
  rm::hal::Serial<18> *dbus{nullptr};                               ///< 遥控器串口接口
  rm::device::AimbotCanCommunicator *aimbot_communicator{nullptr};  ///< CAN 通信器
  rm::hal::Serial<128> *ident_uart{nullptr};

  u8 aimbot_robot_id = 3;           // 独立云台：3为红方，103为蓝方
  f32 aimbot_bullet_speed = 23.5f;  // 沿用原先未收到弹速时的默认值

  // 设备 //
  rm::device::DeviceManager<1> device_rc;  ///< 设备管理器，维护所有设备在线状态
  rm::device::DeviceManager<2> device_gimbal;
  rm::device::DeviceManager<1> device_nuc;

  // 云台
  // rm::hal::Serial<1024> *imu_uart{nullptr};
  // rm::device::HipnucImu *imu{nullptr};
  rm::device::BMI088 *imu{nullptr};                                                 ///< IMU
  rm::device::DR16 *rc{nullptr};                                                    ///< 遥控器
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *yaw_motor{nullptr};    ///< 云台 Yaw 上电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr};  ///< 云台 Pitch 电机

  // 控制器 //
  rm::modules::QuaternionEkfAhrs ahrs{500.0f};  ///< 姿态解算器
  // struct {
  //   Eigen::Vector3f angles = Eigen::Vector3f::Zero();
  //   Eigen::Vector3f &euler_angle() { return angles; }
  //   const Eigen::Vector3f &euler_angle() const { return angles; }
  // } ahrs;
  Gimbal2Dof gimbal_controller;  ///< 二轴云台控制器

  StateMachineType StateMachine_ = {kNoForce};                     // 当前状态
  RemoteControlSource remote_source = RemoteControlSource::kNone;  ///< DT17 在线时为控制源
  RemoteInput remote_input{};

  uint8_t time = 0;                   // 时间
  uint16_t init_time = 1000;          // 初始化时间
  uint8_t music_choice = 0;           // 音乐选择
  uint8_t aim_mode = 0x01;            // 自瞄模式
  uint16_t imu_count = 0;             // IMU计数器
  bool aim_mood_change_flag = false;  // 自瞄模式切换标识位
  bool music_play_flag = false;       // 音乐播放标识位
  bool music_change_flag = false;     // 音乐改动标识位
  bool df_flag = false;               // 大符标志位
  bool df_state = false;              // 大符状态
  bool xf_flag = false;               // 小符标志位
  bool xf_state = false;              // 小符状态

  rm::device::DR16::SwitchPosition last_switch_l = rm::device::DR16::SwitchPosition::kDown;  // 左拨杆上一次状态
  rm::device::DR16::SwitchPosition last_switch_r = rm::device::DR16::SwitchPosition::kDown;  // 右拨杆上一次状态

  WaveGenerator yaw_wave{};
  WaveGenerator pitch_wave{};

  // 函数 //
  void Init();

  void SubLoop500Hz();

  void SubLoop250Hz();

  void SubLoop100Hz();

  void SubLoop50Hz();

  void SubLoop10Hz();

  void GimbalPIDInit();

 private:
  void GimbalSMCInit();

  void RCStateUpdate();

  void RemoteInputUpdate();

  void AimModeUpdate();

  void Music();
} *globals;
