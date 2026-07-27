/**
 * @Author: JL HUANG
 * @Date: 2026-07-12 03:13:28
 * @LastEditors: JL HUANG
 * @LastEditTime: 2026-07-25 09:55:43
 * @FilePath: app/targets/dart_rack_new/dart_sys.hpp
 * @Description: 
 * Copyright (c) 2026 by JL HUANG, All Rights Reserved. 
 */
//
// Created by JL_HUANG on 2026/7/12.
//

#ifndef BOARDC_DART_SYS_HPP
#define BOARDC_DART_SYS_HPP

#include <librm.hpp>
#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "encoder_counter.hpp"
#include "usb.hpp"
#include "tim.h"

class PWM_Servo {
public:
  const uint16_t trigger_off_compare{2050};
  const uint16_t trigger_on_compare{1700};
  
  const uint16_t reload1_off_compare{1000};
  const uint16_t reload1_on_compare{1100};
  const uint16_t reload2_off_compare{1000};
  const uint16_t reload2_on_compare{1100};
  const uint16_t reload3_off_compare{1000};
  const uint16_t reload3_on_compare{1100};
  
  uint16_t trigger_count_{0};
  uint16_t reload1_count_{0};
  uint16_t reload2_count_{0};
  uint16_t reload3_count_{0};
  void Init() {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  }
  bool TriggerServo(uint16_t pwm_val) {
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm_val);
    if (trigger_count_ < 1000) {
      trigger_count_++;
      return false;
    }else {
      trigger_count_ = 0;
      return true;
    }
  }
  
  bool ReloadServo1(uint16_t pwm_val) {
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm_val);
    if (reload1_count_ < 1000) {
      reload1_count_++;
      return false;
    }else {
      reload1_count_ = 0;
      return true;
    }
  }
  
  bool ReloadServo2(uint16_t pwm_val) {
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm_val);
    if (reload2_count_ < 1000) {
      reload2_count_++;
      return false;
    }else {
      reload2_count_ = 0;
      return true;
    }
  }
  
  bool ReloadServo3(uint16_t pwm_val) {
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm_val);
    if (reload3_count_ < 1000) {
      reload3_count_++;
      return false;
    }else {
      reload3_count_ = 0;
      return true;
    }
  }
};

class DartSys {
public:
  // 硬件接口
  rm::hal::Can *can1_{nullptr};     ///< CAN 总线接口
  rm::hal::Can *can2_{nullptr};     ///< CAN 总线接口
  
  rm::hal::Serial *dbus_{nullptr};  ///< 遥控器串口接口
  // 设备
  rm::device::DR16 *rc_{nullptr};                    ///< 遥控器
  
  rm::device::M3508 *load_motor_l_{nullptr};         ///< 左上膛电机，正转下拉上膛
  rm::device::M3508 *load_motor_r_{nullptr};         ///< 右上膛电机，反转下拉上膛
  
  rm::device::M3508 *pitch_motor_{nullptr};          ///< 调弹簧活动电机
  
  rm::device::M3508 *yaw_motor_{nullptr};            ///< yaw轴调节电机
  rm::device::JyMe02Can *yaw_encoder_{nullptr};      ///< 编码器
  
  rm::device::GM6020 *reload_motor_{nullptr};        ///< 换弹转盘电机
  
  PWM_Servo pwm_servo_{};

  // usb设备
  USBVisionReceive_SCM_t *vision_data_{nullptr};  ///< 视觉数据

  // 裁判系统
  rm::device::Referee<rm::device::RefereeRevision::kV170> *referee_data_buffer{nullptr};
  // PID 控制器
  rm::modules::PID load_motor_l_speed_pid_{};
  rm::modules::PID load_motor_r_speed_pid_{};
  
  rm::modules::PID load_motor_angle_pid_{};  //上膛两电机同步一个位置环
  
  rm::modules::PID pitch_motor_speed_pid_{};
  rm::modules::PID pitch_motor_angle_pid_{};
  
  rm::modules::PID yaw_motor_speed_pid_{};
  rm::modules::PID yaw_motor_angle_pid_{};
  
  rm::modules::PID reload_motor_speed_pid_{};
  rm::modules::PID reload_motor_angle_pid_{};
  
  EncoderCounter load_motor_l_odometer_;  //转子约200圈
  EncoderCounter load_motor_r_odometer_;  //转子约200圈
  EncoderCounter pitch_motor_odometer_;   //转子约3000圈
  // EncoderCounter yaw_motor_force_odometer_;
  
  rm::device::BMI088 *imu_{nullptr};                    ///< BMI088 IMU
  rm::modules::MahonyAhrs ahrs_{1000.f};      ///< mahony 姿态解算器，频率 1000Hz
  
  AsyncBuzzer *buzzer_{nullptr};  ///< 蜂鸣器
  LED *led_{nullptr};             ///< RGB LED灯
  
  void init();
  void loop();
  
  bool pitch_motor_limit_enabled_{false};
protected:
  
private:
  /* 初始化状态 */
  enum class PhaseState : uint8_t { kUncomplete = 0,kReverse = 1, kDone = 2 };
  class SelfCheckStates {
    public:
    PhaseState load_motor_state_{PhaseState::kUncomplete};
    PhaseState pitch_motor_state_{PhaseState::kUncomplete};
    PhaseState yaw_motor_state_{PhaseState::kUncomplete};
    PhaseState trigger_servo_state_{PhaseState::kUncomplete};
    PhaseState reload_motor_state_{PhaseState::kUncomplete};
    PhaseState reload_servo_state_{PhaseState::kUncomplete};
  };
  SelfCheckStates self_check_states_{};
  void load_motor_check();
  void pitch_motor_check();
  void yaw_motor_check();
  void trigger_servo_check();
  void reload_motor_check();
  void reload_servo_check();
  
  class MotorResetStates {
    public:
    PhaseState load_trigger_motor_state_{PhaseState::kUncomplete};
    PhaseState pitch_yaw_motor_state_{PhaseState::kUncomplete};
    PhaseState reload_servo_motor_state_{PhaseState::kUncomplete};
  };
  MotorResetStates motor_reset_states_{};
  void load_trigger_motor_reset();
  void pitch_yaw_motor_reset();
  void reload_servo_motor_reset();
  bool dart_motor_reset();
  
  enum class ReloadState : uint8_t {
    kReloadSet = 0,
    kReloadStir = 1,
    kReloadServo = 2,
    kReloadMagnet = 3,
    kReloadDone = 4
  };
  enum class LoadState : uint8_t {
    kReset = 0,
    kLoadHighSpd = 1, 
    kLoadLowSpd = 2, 
    kLoadStall = 3,
    kLoadReversePos = 4,
    kLoadFire = 5
  };
  // enum class TriggerState : uint8_t {
  //   kTriggerOff = 0, kTriggerOffDone = 1,
  //   kTriggerOn = 2, kTriggerOnDone = 3
  // };
  // enum class AimState : uint8_t {
  //   kReset = 0, kResetDone = 1,
  //   kAim = 2, kAimDone = 3
  // };
  class FireStates {
    public:
    // uint8_t fire_count_{0};
    // ReloadState reload_state_{ReloadState::kReset};
    LoadState load_state_{LoadState::kReset};
    ReloadState reload_state_{ReloadState::kReloadSet};
    // TriggerState trigger_state_{TriggerState::kTriggerOffDone};
    // AimState aim_state_{AimState::kReset};
    // PhaseState fire_state_{PhaseState::kUncomplete};
  };
  FireStates fire_states_{};
  
  bool load_motor_pos(int16_t pos);
  bool load_motor_spd_stall(int16_t spd, uint16_t current_limit, bool if_reset);
  bool pitch_motor_pos(int16_t pos);
  bool pitch_motor_spd_stall(int16_t spd);
  
  // bool signal_fire_reload_task();
  // bool signal_fire_load_task();
  // bool signal_fire_trigger_task();
  // bool signal_fire_aim_task();
  
  /* 系统状态 */
  enum class DartState : uint8_t { 
    kDisable = 0, 
    kSelfCheck = 1, kSelfCheckDone = 2, 
    kRC_control = 3, 
    kSignalFire = 4, 
    kAuto_control = 5 
  };
  DartState dart_state_{DartState::kDisable};
  
  /* 遥控中间变量 */
  rm::device::DR16::SwitchPosition last_switch_l_{};
  rm::device::DR16::SwitchPosition now_switch_l_{};
  enum class rc_switch: uint8_t { kStopped = 0, kStarted = 1,kStarting = 2,kStoping = 3 };
  rc_switch rc_switch_l_{rc_switch::kStopped};
  float rc_yaw_motor_target_spd_{};
  float rc_pitch_motor_target_spd_{};
  float rc_load_motor_target_spd_{};
  float rc_reload_motor_target_spd_{};
  const float rc_yaw_motor_max_{8000.0f};
  const float rc_pitch_motor_max_{8000.0f};
  const float rc_load_motor_max_{3500.0f};
  const float rc_reload_motor_max_{70.0f};
  
  /* 编码记圈中间变量 */
  int32_t load_motor_angle_linear_ticks_{0}; //上膛滑块实际位置，向下拉为正
  int16_t load_motor_angle_revolutions_{0};
  int16_t load_motor_angle_delta_ticks_{0};
  
  int32_t pitch_motor_angle_linear_ticks_{0};//向下拉为正
  int16_t pitch_motor_angle_revolutions_{0};
  int16_t pitch_motor_angle_delta_ticks_{0};

  int16_t shoot_count_{0};
  float reload_motor_target_pos_{180.0f};
  /*
  const float reload_motor_reset_pos_{180.0f};
  const float reload_motor_stir_pos_1_{180.0f};
  const float reload_motor_stir_pos_2_{240.0f};
  const float reload_motor_stir_pos_3_{300.0f};
  */
  
  const int16_t load_motor_reset_revolutions_{5};
  
  const float yaw_motor_reset_degree_{88.0f};
  const float yaw_motor_left_limit_degree_{92.0f};
  const float yaw_motor_right_limit_degree_{81.0f};
  
  const int16_t pitch_motor_reset_revolutions_{500};
  
  void imu_task();
  void rc_task();
  void self_check_task();
  void dart_rc_control_task();
  void dart_signal_fire_task();
  void dart_disable_task();
  void encoder_counter_task();
  
  void rc_switch_l_state();
  void rc_switch_r_state();
  void rc_target_spd_get();
  void rc_stick_limit();
};

//extern DartSys* dart_sys;
extern DartSys dart_sys;

#endif  // BOARDC_DART_SYS_HPP
