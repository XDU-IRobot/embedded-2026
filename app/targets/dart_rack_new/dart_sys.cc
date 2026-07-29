//
// Created by JL_HUANG on 2026/7/12.
//

#include "dart_sys.hpp"
#include "can.h"
#include "usart.h"
#include "spi.h"
FreeMasterVars g_fm;
static uint8_t i = 0;
static bool pitch_done = false;
static bool pitch_finished = false;


void DartSys::init() {
  // PID初始化
  load_motor_l_speed_pid_.SetKp(20).SetKi(0.3).SetKd(0).SetMaxOut(16384).SetMaxIout(14000);
  load_motor_r_speed_pid_.SetKp(20).SetKi(0.3).SetKd(0).SetMaxOut(16384).SetMaxIout(14000);

  load_motor_angle_pid_.SetKp(150).SetKi(0).SetKd(0).SetMaxOut(7000).SetMaxIout(2000);  // 上膛两电机同步一个位置环

  pitch_motor_speed_pid_.SetKp(15).SetKi(0).SetKd(0).SetMaxOut(10000).SetMaxIout(1000);
  pitch_motor_angle_pid_.SetKp(500).SetKi(0).SetKd(0).SetMaxOut(15000).SetMaxIout(12000);

  yaw_motor_speed_pid_.SetKp(5).SetKi(0.05).SetKd(0).SetMaxOut(16384).SetMaxIout(2000);
  yaw_motor_angle_pid_.SetKp(17000).SetKi(0.05).SetKd(0).SetMaxOut(15000).SetMaxIout(5000);

  reload_motor_speed_pid_.SetKp(400).SetKi(0).SetKd(0).SetMaxOut(25000).SetMaxIout(25000);
  reload_motor_angle_pid_.SetKp(1).SetKi(0).SetKd(0).SetMaxOut(400).SetMaxIout(0).SetCircular(true).SetCircularCycle(
      360);

  // TODO:切换自动发射记得更新堵转电流，和空载不一样
  load_motor_l_odometer_.Reset();
  load_motor_l_odometer_.set_current_limit(4000);
  load_motor_r_odometer_.Reset();
  load_motor_r_odometer_.set_current_limit(4000);
  pitch_motor_odometer_.Reset();
  pitch_motor_odometer_.set_current_limit(1500);

  // 硬件接口初始化
  can1_ = new rm::hal::Can{hcan1};
  can2_ = new rm::hal::Can{hcan2};

  dbus_ = new rm::hal::Serial<128>{huart3, true, true};
  rc_ = new rm::device::DR16{*dbus_};
  rc_->Begin();

  // 电机初始化
  /* yaw pitch 6020 can2 */
  /* 上膛 编码器 can1 */
  load_motor_l_ = new rm::device::M3508{*can1_, 2};
  load_motor_r_ = new rm::device::M3508{*can1_, 3};

  pitch_motor_ = new rm::device::M3508{*can2_, 4};

  yaw_motor_ = new rm::device::M3508{*can2_, 6};

  reload_motor_ = new rm::device::GM6020{*can2_, 1};

  vision_data_ = new USBVisionReceive_SCM_t{};

  // 裁判系统
  referee_data_buffer = new rm::device::Referee<rm::device::RefereeRevision::kV170>;

  // 编码器初始化 左169.5deg 右158.5deg
  yaw_encoder_ = new rm::device::JyMe02Can{*can2_, 0x50, 0.1f};

  can1_->SetFilter(0, 0);
  can1_->Begin();
  can2_->SetFilter(0, 0);
  can2_->Begin();

  imu_ = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};

  buzzer_ = new AsyncBuzzer;
  led_ = new LED;
  buzzer_->Init();
  led_->Init();

  pwm_servo_.Init();
}

void DartSys::loop() {
  rc_task();
  self_check_task();
  dart_rc_control_task();
  dart_signal_fire_task();

  dart_disable_task();
  imu_task();              // imu更新，始终运行
  encoder_counter_task();  // 编码累计，始终运行
  rm::device::DjiMotorBase::SendCommand();
}

// volatile rm::f32 imu_pitch;
// volatile rm::f32 imu_roll;
// volatile rm::f32 imu_yaw;

void DartSys::imu_task() {
  imu_->Update();
  ahrs_.Update(rm::modules::ImuData6Dof{imu_->gyro_y(),   //
                                        imu_->gyro_x(),   //
                                        imu_->gyro_z(),   //
                                        imu_->accel_y(),  //
                                        imu_->accel_x(),  //
                                        imu_->accel_z()});

  // 获取姿态数据(欧拉角，弧度)
  // imu_pitch = ahrs_.euler_angle().pitch;
  // imu_roll = ahrs_.euler_angle().roll;
  // imu_yaw = ahrs_.euler_angle().yaw;
  // // 获取四元数
  // ahrs_.quaternion().w;
  // ahrs_.quaternion().x;
  // ahrs_.quaternion().y;
  // ahrs_.quaternion().z;
}

void DartSys::rc_task() {
  rc_switch_l_state();
  rc_target_spd_get();
  rc_switch_r_state();
}

void DartSys::self_check_task() {
  if (dart_state_ == DartState::kSelfCheck) {
    (*led_)(0xffffff00);
    // 自检，上膛电机和pitch电机堵转找零点，找到零点后反转
    // 判断堵转时间后再重置，核对堵转电流单位
    // yaw转到固定角度
    // 换弹舵机和转盘归位
    // 扳机归位
    load_motor_check();
    pitch_motor_check();
    yaw_motor_check();
    trigger_servo_check();
    reload_motor_check();
    reload_servo_check();

    if (self_check_states_.load_motor_state_ == PhaseState::kDone &&
        self_check_states_.pitch_motor_state_ == PhaseState::kDone &&
        self_check_states_.yaw_motor_state_ == PhaseState::kDone &&
        self_check_states_.trigger_servo_state_ == PhaseState::kDone &&
        self_check_states_.reload_motor_state_ == PhaseState::kDone &&
        self_check_states_.reload_servo_state_ == PhaseState::kDone) {
      // 自检完成后更新状态
      buzzer_->Beep(1, 100);
      buzzer_->Beep(2, 100);
      buzzer_->Beep(3, 100);
      buzzer_->Beep(4, 100);
      buzzer_->Beep(5, 100);
      dart_state_ = DartState::kSelfCheckDone;
    }
  }
}

void DartSys::dart_rc_control_task() {
  if (dart_state_ == DartState::kRC_control) {
    (*led_)(0xff00ff00);
    // load_motor_angle_pid_.Update(0.0f,0.0f,1.0f);
    load_motor_l_speed_pid_.Update(-rc_load_motor_target_spd_, load_motor_l_->rpm(), 1.0f);
    load_motor_r_speed_pid_.Update(rc_load_motor_target_spd_, load_motor_r_->rpm(), 1.0f);
    load_motor_l_->SetCurrent(static_cast<rm::i16>(load_motor_l_speed_pid_.out()));
    load_motor_r_->SetCurrent(static_cast<rm::i16>(load_motor_r_speed_pid_.out()));

    // pitch_motor_angle_pid_.Update(rc_pitch_motor_target_spd_,pitch_motor_->pos_degree(),1.0f);
    pitch_motor_speed_pid_.Update(-rc_pitch_motor_target_spd_, pitch_motor_->rpm(), 1.0f);
    pitch_motor_->SetCurrent(static_cast<rm::i16>(pitch_motor_speed_pid_.out()));

    // yaw_motor_angle_pid_.Update(0,yaw_encoder_->angle_deg(),1.0f);
    yaw_motor_speed_pid_.Update(-rc_yaw_motor_target_spd_, yaw_motor_->rpm(), 1.0f);
    yaw_motor_->SetCurrent(static_cast<rm::i16>(yaw_motor_speed_pid_.out()));

    // reload_motor_angle_pid_.Update(0,reload_motor_->pos_degree(),1.0f);
    reload_motor_speed_pid_.Update(-rc_reload_motor_target_spd_, reload_motor_->rpm(), 1.0f);
    reload_motor_->SetCurrent(static_cast<rm::i16>(reload_motor_speed_pid_.out()));

    motor_reset_states_.load_trigger_motor_state_ = PhaseState::kUncomplete;
    motor_reset_states_.pitch_yaw_motor_state_ = PhaseState::kUncomplete;
    motor_reset_states_.reload_servo_motor_state_ = PhaseState::kUncomplete;

    fire_states_.load_state_ = LoadState::kReset;
  }
}
float reload_motor_pos;
uint16_t ticks;
void DartSys::dart_signal_fire_task() {
  if (reload_motor_target_pos_ < 0) {
    reload_motor_target_pos_ += 360.0f;
  }
  reload_motor_pos = reload_motor_->pos_degree();

  if (dart_state_ == DartState::kSignalFire) {
    if (dart_motor_reset()) {
      // (*led_)(0xff0000ff);
      if (rc_->dial() == -660 && fire_states_.reload_state_ == ReloadState::kReloadSet) {
        // fire_states_.fire_count_ = 1;
        // fire_states_.reload_state_ = ReloadState::kReloadDart1Done;
        fire_states_.load_state_ = LoadState::kLoadHighSpd;
        // fire_states_.aim_state_ = AimState::kAim;
      }
     // if (rc_->dial() >= 600) {
        //&& fire_states_.reload_state_ == ReloadState::kReloadSet
        // if (shoot_count_ <= 3) {
        //   shoot_count_++;
        //   fire_states_.reload_state_ = ReloadState::kReloadStir;
        //   reload_motor_target_pos_ -= 60.0f;
        // }

        static bool yaw_approach_suspended = false;
        i = static_cast<uint8_t>(shoot_count_);

        static uint16_t invalid_lock_cnt = 0;
        if (g_usb_rx_count == 0 || vision_data_->IsValiLock == 0) {
          invalid_lock_cnt++;
          if (invalid_lock_cnt >= 30) {
            yaw_approach_suspended = true;
          }
        }else {
            invalid_lock_cnt = 0;
          }
          constexpr float tolerance = 5.0f;
          //constexpr float PerWidth[4] = {-70.0f, -70.0f, -70.0f, -70.0f};                 // yaw
          //constexpr int32_t PerHeight[4] = {-2250000, -2250000, -2250000, -2250000};  // pitch
          if (!yaw_approach_suspended) {
            // error为正,往右边,error为负,往左边
            // 先更新角度环,再更新速度环
           // yaw_motor_angle_pid_.Update(PerWidth[i], g_fm.yaw_vision, 1.0f);
            // g_fm.angle_out = yaw_motor_angle_pid_.out();
            // yaw_motor_speed_pid_.Update(g_fm.angle_out, yaw_motor_->rpm(), 1.0f);
            // g_fm.speed_out = yaw_motor_speed_pid_.out();
            // yaw_motor_->SetCurrent(static_cast<rm::i16>(yaw_motor_speed_pid_.out()));
          }
          g_fm.pitch_ticks = pitch_motor_odometer_.linear_ticks();
          //if (g_fm.pitch_ticks > PerHeight[i]) {
            //pitch_motor_speed_pid_.Update(-8000.0f, pitch_motor_->rpm(), 1.0f);
            //pitch_motor_->SetCurrent(static_cast<rm::i16>(pitch_motor_speed_pid_.out()));
          //}
          // else {
          //   pitch_motor_speed_pid_.Update(0.0f, pitch_motor_->rpm(), 1.0f);
          //   pitch_motor_->SetCurrent(static_cast<rm::i16>(pitch_motor_speed_pid_.out()));
          //   pitch_done = true;
          // }
        if (pitch_done) {
          if (std::abs(g_fm.rpm_pitch) < 50) {
            static int32_t trigger_cnt = 0;
            trigger_cnt++;
            if (trigger_cnt >= 500) trigger_cnt = 0;
            pitch_motor_speed_pid_.Clear();
            pitch_motor_->SetCurrent(0);
            pitch_finished = true;
         }
        }

        // if (g_fm.yaw_deg >= DartRack::kYawEcdMax || g_fm.yaw_deg <= DartRack::kYawEcdMin) {
        //   yaw_approach_suspended = true;
        // }
       // if (std::abs(g_fm.yaw_vision - PerWidth[i]) < tolerance || yaw_approach_suspended) {
          yaw_motor_speed_pid_.Update(0.0f,yaw_motor_->rpm(), 1.0f);
          yaw_motor_->SetCurrent(static_cast<rm::i16>(yaw_motor_speed_pid_.out()));
          if (std::abs(g_fm.rpm) < 50) {
            static int32_t yaw_cnt = 0;
            yaw_cnt++;
            if (yaw_cnt >= 500) yaw_cnt = 0;
            yaw_motor_angle_pid_.Clear();
            yaw_motor_speed_pid_.Clear();
            yaw_motor_->SetCurrent(0);
           // yaw_finished = true;
          }
        //}
        // if (trigger_finished && yaw_finished) {
        //   dart_rack->yaw_motor_angle_pid_.Clear();
        //   dart_rack->yaw_motor_speed_pid_.Clear();
        //   dart_rack->yaw_motor_->SetCurrent(0);
        //   dart_rack->trigger_motor_speed_pid_.Clear();
        //   dart_rack->trigger_motor_->SetCurrent(0);
        //   dart_rack->load_motor_l_speed_pid_.Clear();
        //   dart_rack->load_motor_l_->SetCurrent(0);
        //   dart_rack->load_motor_r_speed_pid_.Clear();
        //   dart_rack->load_motor_r_->SetCurrent(0);
        //   dart_rack->state_.manual_mode.aim = PhaseState::kDone;
        //
        //   // 重置所有静态变量，为下一次瞄准做准备
        //   yaw_approach_suspended = false;
        //   trigger_finished = false;
        //   yaw_finished = false;
        //   trigger_done = false;
        //   trigger_backoff_state = 0;
        // }
      //}

        if (fire_states_.load_state_ == LoadState::kLoadHighSpd) {
          // bool load_state = load_motor_pos(170);
          bool trigger_state = pwm_servo_.TriggerServo(pwm_servo_.trigger_on_compare);
          if (trigger_state) {
            (*led_)(0xff0000ff);
            fire_states_.load_state_ = LoadState::kLoadLowSpd;
          }
        }
        if (fire_states_.load_state_ == LoadState::kLoadLowSpd) {
          // 堵转判断扳机位置
          bool load_state = load_motor_spd_stall(2000, 12000, false);
          // bool load_state = true;
          if (load_state) {
            load_motor_spd_stall(2000, 10000, false);
            fire_states_.load_state_ = LoadState::kLoadStall;
          }
        }
        if (fire_states_.load_state_ == LoadState::kLoadStall) {
          bool trigger_state = pwm_servo_.TriggerServo(pwm_servo_.trigger_off_compare);
          load_motor_spd_stall(2000, 12000, false);
          if (trigger_state) {
            fire_states_.load_state_ = LoadState::kLoadReversePos;
          }
        }
        if (fire_states_.load_state_ == LoadState::kLoadReversePos) {
          bool load_state = load_motor_pos(load_motor_reset_revolutions_);
          if (load_state) {
            fire_states_.load_state_ = LoadState::kLoadFire;
          }
        }
        if (fire_states_.load_state_ == LoadState::kLoadFire && rc_->left_x() >= 600 && rc_->right_x() <= -600) {
          bool trigger_state = pwm_servo_.TriggerServo(pwm_servo_.trigger_on_compare);
          if (trigger_state) {
            fire_states_.load_state_ = LoadState::kReset;
            pwm_servo_.TriggerServo(pwm_servo_.trigger_off_compare);
            if (shoot_count_ <= 3) {
              shoot_count_++;
              fire_states_.reload_state_ = ReloadState::kReloadStir;
              reload_motor_target_pos_ -= 60.0f;
            }
          }
        }
        if (fire_states_.reload_state_ == ReloadState::kReloadStir) {
          if (abs(reload_motor_->pos_degree() - reload_motor_target_pos_) <= 1) {
            fire_states_.reload_state_ = ReloadState::kReloadServo;
          }
        }
        if (fire_states_.reload_state_ == ReloadState::kReloadServo) {
          switch (shoot_count_) {
            case 1:
              pwm_servo_.ReloadServo1(pwm_servo_.reload1_on_compare);
              fire_states_.reload_state_ = ReloadState::kReloadMagnet;
              break;
            case 2:
              pwm_servo_.ReloadServo2(pwm_servo_.reload2_on_compare);
              fire_states_.reload_state_ = ReloadState::kReloadMagnet;
              break;
            case 3:
              pwm_servo_.ReloadServo3(pwm_servo_.reload3_on_compare);
              fire_states_.reload_state_ = ReloadState::kReloadMagnet;
              break;
            default:
              break;
          }
          ticks = 0;
        }
        if (fire_states_.reload_state_ == ReloadState::kReloadMagnet) {
          switch (shoot_count_) {
            case 1:
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
              ticks++;
              if (ticks > 2000) {
                pwm_servo_.ReloadServo1(pwm_servo_.reload1_off_compare);
              }
              if (ticks > 3000) {
                reload_motor_target_pos_ -= 60.0f;
                fire_states_.reload_state_ = ReloadState::kReloadDone;
                ticks = 0;
              }

              break;
            case 2:
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
              ticks++;
              if (ticks > 2000) {
                pwm_servo_.ReloadServo2(pwm_servo_.reload2_off_compare);
              }
              if (ticks > 3000) {
                reload_motor_target_pos_ -= 60.0f;
                fire_states_.reload_state_ = ReloadState::kReloadDone;
                ticks = 0;
              }
              break;
            case 3:
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
              ticks++;
              if (ticks > 2000) {
                pwm_servo_.ReloadServo3(pwm_servo_.reload3_off_compare);
              }
              if (ticks > 3000) {
                reload_motor_target_pos_ -= 60.0f;
                fire_states_.reload_state_ = ReloadState::kReloadDone;
                ticks = 0;
              }
              break;
            default:
              break;
          }
        }
        if (fire_states_.reload_state_ == ReloadState::kReloadDone) {
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
          if (abs(reload_motor_->pos_degree() - reload_motor_target_pos_) <= 1) {
            fire_states_.reload_state_ = ReloadState::kReloadSet;
          }
        }
      }
    } else {
      shoot_count_ = 0;
      fire_states_.reload_state_ = ReloadState::kReloadSet;
    }
    // TODO:aim部分电子限位
  }


  // bool DartSys::dart_fire_reset() {
  //   if (fire_states_.reload_state_ == ReloadState::kReset) {
  //     reload_motor_angle_pid_.Update(reload_motor_reset_pos_,reload_motor_->pos_degree(),1.0f);
  //     reload_motor_speed_pid_.Update(reload_motor_angle_pid_.out(),reload_motor_->rpm(),1.0f);
  //     reload_motor_->SetCurrent(static_cast<rm::i16>(reload_motor_speed_pid_.out()));
  //     if (abs(reload_motor_->pos_degree() - reload_motor_reset_pos_) <= 1) {
  //       bool servo1_state = pwm_servo_.ReloadServo1(pwm_servo_.reload1_off_compare);
  //       bool servo2_state = pwm_servo_.ReloadServo2(pwm_servo_.reload2_off_compare);
  //       bool servo3_state = pwm_servo_.ReloadServo3(pwm_servo_.reload3_off_compare);//回收位
  //       if (servo1_state && servo2_state && servo3_state) {
  //         fire_states_.reload_state_ = ReloadState::kResetDone;
  //       }
  //     }
  //   }else if (fire_states_.reload_state_ == ReloadState::kResetDone) {
  //     reload_motor_angle_pid_.Update(reload_motor_reset_pos_,reload_motor_->pos_degree(),1.0f);
  //     reload_motor_speed_pid_.Update(reload_motor_angle_pid_.out(),reload_motor_->rpm(),1.0f);
  //     reload_motor_->SetCurrent(static_cast<rm::i16>(reload_motor_speed_pid_.out()));
  //   }
  //
  //   if (fire_states_.load_state_ == LoadState::kReset) {
  //     load_motor_angle_pid_.Update(load_motor_reset_revolutions_,load_motor_angle_revolutions_,1.0f);
  //     load_motor_l_speed_pid_.Update(load_motor_angle_pid_.out(),load_motor_l_->rpm(),1.0f);
  //     load_motor_r_speed_pid_.Update(-load_motor_angle_pid_.out(),load_motor_r_->rpm(),1.0f);
  //     load_motor_l_->SetCurrent(static_cast<rm::i16>(load_motor_l_speed_pid_.out()));
  //     load_motor_r_->SetCurrent(static_cast<rm::i16>(load_motor_r_speed_pid_.out()));
  //     if (load_motor_angle_revolutions_ == load_motor_reset_revolutions_) {
  //       // 反转到位后初始化结束
  //       fire_states_.load_state_ = LoadState::kResetDone;
  //     }
  //   }else if (fire_states_.load_state_ == LoadState::kResetDone) {
  //     load_motor_angle_pid_.Update(load_motor_reset_revolutions_,load_motor_angle_revolutions_,1.0f);
  //     load_motor_l_speed_pid_.Update(load_motor_angle_pid_.out(),load_motor_l_->rpm(),1.0f);
  //     load_motor_r_speed_pid_.Update(-load_motor_angle_pid_.out(),load_motor_r_->rpm(),1.0f);
  //     load_motor_l_->SetCurrent(static_cast<rm::i16>(load_motor_l_speed_pid_.out()));
  //     load_motor_r_->SetCurrent(static_cast<rm::i16>(load_motor_r_speed_pid_.out()));
  //   }
  //
  //   // if (fire_states_.trigger_state_ == TriggerState::kTriggerOn) {
  //   //   //正常进入该状态扳机已经关闭，此处只是以防万一
  //   //   if (pwm_servo_.TriggerServo(pwm_servo_.trigger_off_compare) ) {
  //   //     fire_states_.trigger_state_ = TriggerState::kTriggerOff;
  //   //   }
  //   // }
  //
  //   if (fire_states_.aim_state_ == AimState::kReset) {
  //     pitch_motor_angle_pid_.Update(pitch_motor_reset_revolutions_,pitch_motor_angle_revolutions_,1.0f);
  //     pitch_motor_speed_pid_.Update(pitch_motor_angle_pid_.out(),pitch_motor_->rpm(),1.0f);
  //     pitch_motor_->SetCurrent(static_cast<rm::i16>(pitch_motor_speed_pid_.out()));
  //
  //     yaw_motor_angle_pid_.Update(yaw_motor_reset_degree_,yaw_encoder_->angle_deg(),1.0f);
  //     yaw_motor_speed_pid_.Update(yaw_motor_angle_pid_.out(),yaw_motor_->rpm(),1.0f);
  //     yaw_motor_->SetCurrent(static_cast<rm::i16>(yaw_motor_speed_pid_.out()));
  //     if (pitch_motor_angle_revolutions_ == pitch_motor_reset_revolutions_
  //       && abs(yaw_encoder_->angle_deg() - yaw_motor_reset_degree_) <= 0.01
  //       ) {
  //       fire_states_.aim_state_ = AimState::kResetDone;
  //     }
  //   }else if (fire_states_.aim_state_ == AimState::kResetDone) {
  //     pitch_motor_angle_pid_.Update(pitch_motor_reset_revolutions_,pitch_motor_angle_revolutions_,1.0f);
  //     pitch_motor_speed_pid_.Update(pitch_motor_angle_pid_.out(),pitch_motor_->rpm(),1.0f);
  //     pitch_motor_->SetCurrent(static_cast<rm::i16>(pitch_motor_speed_pid_.out()));
  //
  //     yaw_motor_angle_pid_.Update(yaw_motor_reset_degree_,yaw_encoder_->angle_deg(),1.0f);
  //     yaw_motor_speed_pid_.Update(yaw_motor_angle_pid_.out(),yaw_motor_->rpm(),1.0f);
  //     yaw_motor_->SetCurrent(static_cast<rm::i16>(yaw_motor_speed_pid_.out()));
  //   }
  //   fire_states_.fire_count_ = 0;
  //
  //   if (
  //     fire_states_.reload_state_ == ReloadState::kResetDone
  //     && fire_states_.load_state_ == LoadState::kResetDone
  //     && fire_states_.trigger_state_ == TriggerState::kTriggerOffDone
  //     && fire_states_.aim_state_ == AimState::kResetDone
  //     ) {
  //     return true;
  //   }else {
  //     return false;
  //   }
  // }

  void DartSys::dart_disable_task() {
    if (dart_state_ == DartState::kDisable) {
      (*led_)(0xffff0000);

      load_motor_l_->SetCurrent(0);
      load_motor_r_->SetCurrent(0);
      pitch_motor_->SetCurrent(0);
      yaw_motor_->SetCurrent(0);
      reload_motor_->SetCurrent(0);

      self_check_states_.load_motor_state_ = PhaseState::kUncomplete;
      self_check_states_.pitch_motor_state_ = PhaseState::kUncomplete;
      self_check_states_.yaw_motor_state_ = PhaseState::kUncomplete;
      self_check_states_.trigger_servo_state_ = PhaseState::kUncomplete;
      self_check_states_.reload_motor_state_ = PhaseState::kUncomplete;
      self_check_states_.reload_servo_state_ = PhaseState::kUncomplete;

      motor_reset_states_.load_trigger_motor_state_ = PhaseState::kUncomplete;
      motor_reset_states_.pitch_yaw_motor_state_ = PhaseState::kUncomplete;
      motor_reset_states_.reload_servo_motor_state_ = PhaseState::kUncomplete;

      fire_states_.load_state_ = LoadState::kReset;

      pitch_motor_limit_enabled_ = false;

      // 累计编码值清空
      load_motor_l_odometer_.Reset();
      load_motor_r_odometer_.Reset();
      pitch_motor_odometer_.Reset();
    }
  }
  volatile uint16_t GM6020_encoder;
  volatile int32_t load_angle_linear_ticks;
  volatile int16_t load_angle_revolutions;
  volatile int16_t load_angle_delta_ticks;
  volatile int32_t pitch_motor_linear_ticks;
  volatile int16_t pitch_motor_revolutions;
  volatile int16_t pitch_motor_delta_ticks;

  // volatile int32_t load_motor_l_linear_ticks;
  // volatile int32_t load_motor_r_linear_ticks;
  // volatile int16_t load_motor_l_revolutions;
  // volatile int16_t load_motor_r_revolutions;

  volatile int16_t load_motor_l_current;
  volatile int16_t load_motor_r_current;
  volatile int16_t pitch_motor_current;

  volatile uint32_t load_motor_l_stall_time;
  volatile uint32_t load_motor_r_stall_time;
  volatile uint32_t pitch_motor_stall_time;

  volatile float yaw_angle_degree;
  volatile float yaw_angular_speed_dps;
  volatile int16_t yaw_motor_rpm;

  volatile int16_t load_motor_l_rpm;
  volatile int16_t load_motor_r_rpm;
  volatile int16_t pitch_motor_rpm;
  volatile int16_t reload_motor_rpm;

  volatile float reload_motor_angle_deg;

  void DartSys::encoder_counter_task() {
    //
    load_motor_l_odometer_.Update(load_motor_l_->encoder(), load_motor_l_->current());
    load_motor_r_odometer_.Update(load_motor_r_->encoder(), load_motor_r_->current());
    pitch_motor_odometer_.Update(pitch_motor_->encoder(), pitch_motor_->current());

    load_motor_angle_linear_ticks_ =
        (load_motor_l_odometer_.linear_ticks() - load_motor_r_odometer_.linear_ticks()) / 2;
    load_motor_angle_revolutions_ = static_cast<int16_t>(load_motor_angle_linear_ticks_ / 8192);
    load_motor_angle_delta_ticks_ = static_cast<int16_t>(load_motor_angle_linear_ticks_ % 8192);

    GM6020_encoder = reload_motor_->encoder();

    pitch_motor_angle_linear_ticks_ = pitch_motor_odometer_.linear_ticks();
    pitch_motor_angle_revolutions_ = static_cast<int16_t>(pitch_motor_odometer_.revolutions());
    pitch_motor_angle_delta_ticks_ = static_cast<int16_t>(pitch_motor_angle_linear_ticks_ % 8192);

    // 仅调试使用
    load_angle_linear_ticks = load_motor_angle_linear_ticks_;
    load_angle_revolutions = load_motor_angle_revolutions_;
    load_angle_delta_ticks = load_motor_angle_delta_ticks_;
    pitch_motor_linear_ticks = pitch_motor_angle_linear_ticks_;
    pitch_motor_revolutions = pitch_motor_angle_revolutions_;
    pitch_motor_delta_ticks = pitch_motor_angle_delta_ticks_;

    // load_motor_l_linear_ticks = load_motor_l_odometer_.linear_ticks();
    // load_motor_r_linear_ticks = load_motor_r_odometer_.linear_ticks();
    // load_motor_l_revolutions = static_cast<int16_t>(load_motor_l_odometer_.revolutions());
    // load_motor_r_revolutions = static_cast<int16_t>(load_motor_r_odometer_.revolutions());

    // 电机反馈电流，仅调试使用
    load_motor_l_current = load_motor_l_->current();
    load_motor_r_current = load_motor_r_->current();
    pitch_motor_current = pitch_motor_->current();
    // 堵转时间可视化，仅调试使用
    load_motor_l_stall_time = load_motor_l_odometer_.stall_time();
    load_motor_r_stall_time = pitch_motor_odometer_.stall_time();
    pitch_motor_stall_time = pitch_motor_odometer_.stall_time();

    yaw_angle_degree = yaw_encoder_->angle_deg();
    yaw_angular_speed_dps = yaw_encoder_->angular_speed_dps();
    yaw_motor_rpm = yaw_motor_->rpm();

    load_motor_l_rpm = load_motor_l_->rpm();
    load_motor_r_rpm = load_motor_r_->rpm();
    pitch_motor_rpm = pitch_motor_->rpm();
    reload_motor_rpm = reload_motor_->rpm();

    reload_motor_angle_deg = reload_motor_->pos_degree();

    // FreeMaster 监控变量更新
    g_fm.yaw_deg = yaw_encoder_->angle_deg();
    g_fm.rpm = static_cast<int16_t>(yaw_motor_->rpm());
    g_fm.rpm_pitch = static_cast<int32_t>(pitch_motor_->rpm());
    g_fm.yaw_vision = vision_data_->Yaw;
  }

  void aim_task() {}
  /******************************************************************************/

  void DartSys::rc_switch_l_state() {
    last_switch_l_ = now_switch_l_;
    now_switch_l_ = rc_->switch_l();
    if (last_switch_l_ == rm::device::DR16::SwitchPosition::kDown &&
        now_switch_l_ == rm::device::DR16::SwitchPosition::kMid) {
      // 系统自检，电机归零
      rc_switch_l_ = rc_switch::kStarting;
    } else if (last_switch_l_ == rm::device::DR16::SwitchPosition::kMid &&
               now_switch_l_ == rm::device::DR16::SwitchPosition::kMid) {
      // 自检完成，进入主逻辑
      rc_switch_l_ = rc_switch::kStarted;
    } else if (last_switch_l_ == rm::device::DR16::SwitchPosition::kMid &&
               now_switch_l_ == rm::device::DR16::SwitchPosition::kDown) {
      // 失能
      rc_switch_l_ = rc_switch::kStoping;
    } else if (last_switch_l_ == rm::device::DR16::SwitchPosition::kDown &&
               now_switch_l_ == rm::device::DR16::SwitchPosition::kDown) {
      // 失能
      rc_switch_l_ = rc_switch::kStopped;
    }
    switch (rc_switch_l_) {
      case rc_switch::kStarting:
        dart_state_ = DartState::kSelfCheck;
        break;
      case rc_switch::kStarted:
        if (dart_state_ == DartState::kSelfCheckDone) {
          dart_state_ = DartState::kRC_control;
        }
        break;
      case rc_switch::kStoping:
      case rc_switch::kStopped:
        dart_state_ = DartState::kDisable;
        break;
      default:
        break;
    }

    if (dart_state_ == DartState::kRC_control) {
      if (rc_->switch_l() == rm::device::DR16::SwitchPosition::kUp) {
        dart_state_ = DartState::kAuto_control;
      }
    }
    if (dart_state_ == DartState::kAuto_control) {
      if (rc_->switch_l() == rm::device::DR16::SwitchPosition::kMid) {
        dart_state_ = DartState::kRC_control;
      }
    }
  }

  void DartSys::rc_switch_r_state() {
    if (dart_state_ == DartState::kRC_control) {
      if (rc_->switch_r() == rm::device::DR16::SwitchPosition::kMid) {
        // 自检完成，进入主逻辑
        dart_state_ = DartState::kSignalFire;
      }
    }
    if (dart_state_ == DartState::kSignalFire) {
      if (rc_->switch_r() == rm::device::DR16::SwitchPosition::kDown) {
        dart_state_ = DartState::kRC_control;
      }
    }
  }

  volatile float rc_yaw;
  volatile float rc_pitch;
  volatile float rc_load;
  volatile float rc_reload;
  volatile int16_t rc_dial;
  void DartSys::rc_target_spd_get() {
    /* 遥杆值-660~660 */
    rc_yaw_motor_target_spd_ = static_cast<float>(rc_->left_x()) / 660.0f * rc_yaw_motor_max_;
    rc_pitch_motor_target_spd_ = static_cast<float>(rc_->left_y()) / 660.0f * rc_pitch_motor_max_;
    rc_reload_motor_target_spd_ = static_cast<float>(rc_->right_x()) / 660.0f * rc_reload_motor_max_;
    rc_load_motor_target_spd_ = static_cast<float>(rc_->right_y()) / 660.0f * rc_load_motor_max_;

    rc_stick_limit();  // 遥控期望值电子限位

    rc_yaw = rc_yaw_motor_target_spd_;
    rc_pitch = rc_pitch_motor_target_spd_;
    rc_load = rc_load_motor_target_spd_;
    rc_reload = rc_reload_motor_target_spd_;

    // rc_yaw = rc_->left_x();
    // rc_pitch = rc_->left_y();
    // rc_load = rc_->right_y();
    // rc_reload = rc_->right_x();

    rc_dial = rc_->dial();
  }

  void DartSys::rc_stick_limit() {
    // TODO:yaw电子限位 214.5 203.5
    if (yaw_encoder_->angle_deg() >= yaw_motor_left_limit_degree_) {
      rc_yaw_motor_target_spd_ = 1000;
    } else if (yaw_encoder_->angle_deg() <= yaw_motor_right_limit_degree_) {
      rc_yaw_motor_target_spd_ = -1000;
    }
    if (pitch_motor_angle_revolutions_ <= 50) {
      rc_pitch_motor_target_spd_ = -1000;
    }
    if (load_motor_angle_revolutions_ <= 1) {
      rc_load_motor_target_spd_ = -1000;
    }
  }

  void DartSys::load_motor_check() {
    if (self_check_states_.load_motor_state_ == PhaseState::kUncomplete) {
      // 恒定速度找零点
      bool load_state = load_motor_spd_stall(-600, 7000, true);
      if (load_state) {
        load_motor_l_odometer_.Reset();
        load_motor_r_odometer_.Reset();
        self_check_states_.load_motor_state_ = PhaseState::kReverse;
      }
    } else if (self_check_states_.load_motor_state_ == PhaseState::kReverse) {
      // 找到零点后反转5圈，防止持续堵转
      bool load_state = load_motor_pos(load_motor_reset_revolutions_);
      if (load_state) {
        self_check_states_.load_motor_state_ = PhaseState::kDone;
      }
    } else if (self_check_states_.load_motor_state_ == PhaseState::kDone) {
      // 防止出现未定义状态导致load电机疯车，即虽然load初始化完成但其他模块未完成初始化这段时间
      load_motor_pos(load_motor_reset_revolutions_);
    }
  }

  void DartSys::pitch_motor_check() {
    if (self_check_states_.pitch_motor_state_ == PhaseState::kUncomplete) {
      // bool pitch_state = pitch_motor_spd_stall(-500);
      // if (pitch_state) {
      //   pitch_motor_odometer_.Reset();
      //   self_check_states_.pitch_motor_state_ = PhaseState::kReverse;
      // }
      pitch_motor_spd_stall(-2000);
      if (pitch_motor_limit_enabled_) {
        pitch_motor_odometer_.Reset();
        self_check_states_.pitch_motor_state_ = PhaseState::kReverse;
      }
    } else if (self_check_states_.pitch_motor_state_ == PhaseState::kReverse) {
      bool pitch_state = pitch_motor_pos(pitch_motor_reset_revolutions_);
      if (pitch_state) {
        self_check_states_.pitch_motor_state_ = PhaseState::kDone;
      }
    } else if (self_check_states_.pitch_motor_state_ == PhaseState::kDone) {
      pitch_motor_pos(pitch_motor_reset_revolutions_);
    }
  }

  void DartSys::yaw_motor_check() {
    if (self_check_states_.yaw_motor_state_ == PhaseState::kUncomplete) {
      // yaw和绝对编码器联动初始化
      //  编码器范围 左169.5deg 右158.5deg
      yaw_motor_angle_pid_.Update(yaw_motor_reset_degree_, yaw_encoder_->angle_deg(), 1.0f);
      yaw_motor_speed_pid_.Update(yaw_motor_angle_pid_.out(), yaw_motor_->rpm(), 1.0f);
      yaw_motor_->SetCurrent(static_cast<rm::i16>(yaw_motor_speed_pid_.out()));
      if (abs(yaw_encoder_->angle_deg() - yaw_motor_reset_degree_) <= 0.01) {
        self_check_states_.yaw_motor_state_ = PhaseState::kDone;
      }
    } else if (self_check_states_.yaw_motor_state_ == PhaseState::kDone) {
      yaw_motor_angle_pid_.Update(yaw_motor_reset_degree_, yaw_encoder_->angle_deg(), 1.0f);
      yaw_motor_speed_pid_.Update(yaw_motor_angle_pid_.out(), yaw_motor_->rpm(), 1.0f);
      yaw_motor_->SetCurrent(static_cast<rm::i16>(yaw_motor_speed_pid_.out()));
    }
  }

  void DartSys::trigger_servo_check() {
    if (self_check_states_.trigger_servo_state_ == PhaseState::kUncomplete) {
      bool trigger_servo_state = pwm_servo_.TriggerServo(pwm_servo_.trigger_on_compare);
      if (trigger_servo_state) {
        self_check_states_.trigger_servo_state_ = PhaseState::kReverse;
      }
    } else if (self_check_states_.trigger_servo_state_ == PhaseState::kReverse) {
      bool trigger_servo_state = pwm_servo_.TriggerServo(pwm_servo_.trigger_off_compare);
      if (trigger_servo_state) {
        self_check_states_.trigger_servo_state_ = PhaseState::kDone;
      }
    }
  }

  // uint16_t reload_motor_check_counter_{0};
  void DartSys::reload_motor_check() {
    if (self_check_states_.reload_motor_state_ == PhaseState::kUncomplete) {
      reload_motor_angle_pid_.Update(reload_motor_target_pos_, reload_motor_->pos_degree(), 1.0f);
      reload_motor_speed_pid_.Update(reload_motor_angle_pid_.out(), reload_motor_->rpm(), 1.0f);
      reload_motor_->SetCurrent(static_cast<rm::i16>(reload_motor_speed_pid_.out()));

      if (abs(reload_motor_->pos_degree() - reload_motor_target_pos_) <= 1) {
        // reload_motor_check_counter_ ++;
        // if (reload_motor_check_counter_ >= 1000) {
        self_check_states_.reload_motor_state_ = PhaseState::kDone;
        fire_states_.reload_state_ = ReloadState::kReloadSet;
        // reload_motor_check_counter_ = 0;
        // }
      }
    } else if (self_check_states_.reload_motor_state_ == PhaseState::kDone) {
      reload_motor_angle_pid_.Update(reload_motor_target_pos_, reload_motor_->pos_degree(), 1.0f);
      reload_motor_speed_pid_.Update(reload_motor_angle_pid_.out(), reload_motor_->rpm(), 1.0f);
      reload_motor_->SetCurrent(static_cast<rm::i16>(reload_motor_speed_pid_.out()));
    }
  }

  void DartSys::reload_servo_check() {
    if (self_check_states_.reload_servo_state_ == PhaseState::kUncomplete) {
      bool servo1_state = pwm_servo_.ReloadServo1(pwm_servo_.reload1_on_compare);
      bool servo2_state = pwm_servo_.ReloadServo2(pwm_servo_.reload2_on_compare);
      bool servo3_state = pwm_servo_.ReloadServo3(pwm_servo_.reload3_on_compare);
      if (servo1_state && servo2_state && servo3_state) {
        self_check_states_.reload_servo_state_ = PhaseState::kReverse;
      }
    } else if (self_check_states_.reload_servo_state_ == PhaseState::kReverse) {
      bool servo1_state = pwm_servo_.ReloadServo1(pwm_servo_.reload1_off_compare);
      bool servo2_state = pwm_servo_.ReloadServo2(pwm_servo_.reload2_off_compare);
      bool servo3_state = pwm_servo_.ReloadServo3(pwm_servo_.reload3_off_compare);
      if (servo1_state && servo2_state && servo3_state) {
        self_check_states_.reload_servo_state_ = PhaseState::kDone;
      }
    }
  }

  bool DartSys::dart_motor_reset() {
    (*led_)(0xff00ffff);
    load_trigger_motor_reset();
    pitch_yaw_motor_reset();
    reload_servo_motor_reset();

    if (motor_reset_states_.load_trigger_motor_state_ == PhaseState::kDone &&
        motor_reset_states_.pitch_yaw_motor_state_ == PhaseState::kDone &&
        motor_reset_states_.reload_servo_motor_state_ == PhaseState::kDone) {
      // 自检完成后更新状态
      return true;
    }
    return false;
  }

  void DartSys::load_trigger_motor_reset() {
    if (motor_reset_states_.load_trigger_motor_state_ == PhaseState::kUncomplete) {
      load_motor_angle_pid_.Update(load_motor_reset_revolutions_, load_motor_angle_revolutions_, 1.0f);
      load_motor_l_speed_pid_.Update(load_motor_angle_pid_.out(), load_motor_l_->rpm(), 1.0f);
      load_motor_r_speed_pid_.Update(-load_motor_angle_pid_.out(), load_motor_r_->rpm(), 1.0f);
      load_motor_l_->SetCurrent(static_cast<rm::i16>(load_motor_l_speed_pid_.out()));
      load_motor_r_->SetCurrent(static_cast<rm::i16>(load_motor_r_speed_pid_.out()));

      if (load_motor_angle_revolutions_ == load_motor_reset_revolutions_ &&
          pwm_servo_.TriggerServo(pwm_servo_.trigger_off_compare)) {
        motor_reset_states_.load_trigger_motor_state_ = PhaseState::kDone;
      }
    } else if (motor_reset_states_.load_trigger_motor_state_ == PhaseState::kDone) {
      load_motor_angle_pid_.Update(load_motor_reset_revolutions_, load_motor_angle_revolutions_, 1.0f);
      load_motor_l_speed_pid_.Update(load_motor_angle_pid_.out(), load_motor_l_->rpm(), 1.0f);
      load_motor_r_speed_pid_.Update(-load_motor_angle_pid_.out(), load_motor_r_->rpm(), 1.0f);
      load_motor_l_->SetCurrent(static_cast<rm::i16>(load_motor_l_speed_pid_.out()));
      load_motor_r_->SetCurrent(static_cast<rm::i16>(load_motor_r_speed_pid_.out()));
    }
  }

  void DartSys::pitch_yaw_motor_reset() {
    if (motor_reset_states_.pitch_yaw_motor_state_ == PhaseState::kUncomplete) {
      pitch_motor_angle_pid_.Update(pitch_motor_reset_revolutions_, pitch_motor_angle_revolutions_, 1.0f);
      pitch_motor_speed_pid_.Update(pitch_motor_angle_pid_.out(), pitch_motor_->rpm(), 1.0f);
      pitch_motor_->SetCurrent(static_cast<rm::i16>(pitch_motor_speed_pid_.out()));

      yaw_motor_angle_pid_.Update(yaw_motor_reset_degree_, yaw_encoder_->angle_deg(), 1.0f);
      yaw_motor_speed_pid_.Update(yaw_motor_angle_pid_.out(), yaw_motor_->rpm(), 1.0f);
      yaw_motor_->SetCurrent(static_cast<rm::i16>(yaw_motor_speed_pid_.out()));
      if (pitch_motor_angle_revolutions_ == pitch_motor_reset_revolutions_ &&
          abs(yaw_encoder_->angle_deg() - yaw_motor_reset_degree_) <= 0.01) {
        motor_reset_states_.pitch_yaw_motor_state_ = PhaseState::kDone;
      }
    } else if (motor_reset_states_.pitch_yaw_motor_state_ == PhaseState::kDone) {
      pitch_motor_angle_pid_.Update(pitch_motor_reset_revolutions_, pitch_motor_angle_revolutions_, 1.0f);
      pitch_motor_speed_pid_.Update(pitch_motor_angle_pid_.out(), pitch_motor_->rpm(), 1.0f);
      pitch_motor_->SetCurrent(static_cast<rm::i16>(pitch_motor_speed_pid_.out()));

      yaw_motor_angle_pid_.Update(yaw_motor_reset_degree_, yaw_encoder_->angle_deg(), 1.0f);
      yaw_motor_speed_pid_.Update(yaw_motor_angle_pid_.out(), yaw_motor_->rpm(), 1.0f);
      yaw_motor_->SetCurrent(static_cast<rm::i16>(yaw_motor_speed_pid_.out()));
    }
  }

  void DartSys::reload_servo_motor_reset() {
    if (motor_reset_states_.reload_servo_motor_state_ == PhaseState::kUncomplete) {
      reload_motor_angle_pid_.Update(reload_motor_target_pos_, reload_motor_->pos_degree(), 1.0f);
      reload_motor_speed_pid_.Update(reload_motor_angle_pid_.out(), reload_motor_->rpm(), 1.0f);
      reload_motor_->SetCurrent(static_cast<rm::i16>(reload_motor_speed_pid_.out()));
      if (abs(reload_motor_->pos_degree() - reload_motor_target_pos_) <= 1) {
        bool servo1_state = pwm_servo_.ReloadServo1(pwm_servo_.reload1_off_compare);
        bool servo2_state = pwm_servo_.ReloadServo2(pwm_servo_.reload2_off_compare);
        bool servo3_state = pwm_servo_.ReloadServo3(pwm_servo_.reload3_off_compare);  // 回收位
        if (servo1_state && servo2_state && servo3_state) {
          motor_reset_states_.reload_servo_motor_state_ = PhaseState::kDone;
        }
      }
    } else if (motor_reset_states_.reload_servo_motor_state_ == PhaseState::kDone) {
      reload_motor_angle_pid_.Update(reload_motor_target_pos_, reload_motor_->pos_degree(), 1.0f);
      reload_motor_speed_pid_.Update(reload_motor_angle_pid_.out(), reload_motor_->rpm(), 1.0f);
      reload_motor_->SetCurrent(static_cast<rm::i16>(reload_motor_speed_pid_.out()));
    }
  }

  /***********************************************************************/

  bool DartSys::load_motor_pos(int16_t pos) {
    // 位置向下为正
    load_motor_angle_pid_.Update(pos, load_motor_angle_revolutions_, 1.0f);
    load_motor_l_speed_pid_.Update(load_motor_angle_pid_.out(), load_motor_l_->rpm(), 1.0f);
    load_motor_r_speed_pid_.Update(-load_motor_angle_pid_.out(), load_motor_r_->rpm(), 1.0f);
    load_motor_l_->SetCurrent(static_cast<rm::i16>(load_motor_l_speed_pid_.out()));
    load_motor_r_->SetCurrent(static_cast<rm::i16>(load_motor_r_speed_pid_.out()));

    if (load_motor_angle_revolutions_ == pos) {
      return true;
    } else {
      return false;
    }
  }

  bool DartSys::load_motor_spd_stall(int16_t spd, uint16_t current_limit, bool if_reset) {
    // 转速下拉为正
    load_motor_l_odometer_.set_current_limit(current_limit);
    load_motor_r_odometer_.set_current_limit(current_limit);

    load_motor_l_speed_pid_.Update(spd, load_motor_l_->rpm(), 1.0f);
    load_motor_r_speed_pid_.Update(-spd, load_motor_r_->rpm(), 1.0f);
    load_motor_l_->SetCurrent(static_cast<rm::i16>(load_motor_l_speed_pid_.out()));
    load_motor_r_->SetCurrent(static_cast<rm::i16>(load_motor_r_speed_pid_.out()));

    if (load_motor_l_odometer_.stall_time() >= 30 || load_motor_r_odometer_.stall_time() >= 30) {
      if (if_reset) {
        load_motor_l_odometer_.Reset();
        load_motor_r_odometer_.Reset();
      }
      return true;
    } else {
      return false;
    }
  }

  bool DartSys::pitch_motor_pos(int16_t pos) {
    pitch_motor_angle_pid_.Update(pos, pitch_motor_angle_revolutions_, 1.0f);
    pitch_motor_speed_pid_.Update(pitch_motor_angle_pid_.out(), pitch_motor_->rpm(), 1.0f);
    pitch_motor_->SetCurrent(static_cast<rm::i16>(pitch_motor_speed_pid_.out()));

    if (pitch_motor_angle_revolutions_ == pos) {
      return true;
    } else {
      return false;
    }
  }

  bool DartSys::pitch_motor_spd_stall(int16_t spd) {
    pitch_motor_speed_pid_.Update(spd, pitch_motor_->rpm(), 1.0f);
    pitch_motor_->SetCurrent(static_cast<rm::i16>(pitch_motor_speed_pid_.out()));

    if (pitch_motor_odometer_.stall_time() >= 10) {
      pitch_motor_odometer_.Reset();
      return true;
    } else {
      return false;
    }
  }
