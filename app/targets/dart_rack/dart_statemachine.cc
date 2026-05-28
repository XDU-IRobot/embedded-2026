#include "dart_statemachine.hpp"
#include "lcd_init.h"
#include "lvgl.h"
#include <cmath>
#include <cstdio>  // Added for printf
#include "sd_card.h"
#include "gpio.h"
extern bool is_lvgl_running;
extern volatile uint8_t g_add_limit_ever_hit;
extern volatile uint8_t g_trigger_limit_ever_hit;
extern volatile uint8_t g_load_l_limit_ever_hit;
extern volatile uint8_t g_load_r_limit_ever_hit;
extern volatile uint8_t g_add_limit_suppressed;
extern volatile uint8_t g_trigger_motor_limit_suppressed;
extern volatile uint8_t g_add_motor_limit_triggered;
// 裁判系统信号
extern volatile uint8_t glb_game_status;                 // 比赛阶段: 1=准备, 4=进行中
extern volatile uint8_t glb_dart_launch_opening_status;  // 舱门状态: 0=已开启, 1=关闭
// 引入 LVGL 中使用的参数存储数组
extern float Pitch[4];
extern float Yaw[4];
extern int current_category;  // 0 for Pitch, 1 for Yaw
extern int current_index;
extern float temp_val;
extern bool is_editing_val;
volatile float test_if_adjust_mode_is_running = 0.0f;  // 用于测试调参模式是否真的阻塞了其他功能

volatile float dm_smoothed_angle = 0.0f;  // 用于在 FreeMaster 中监测 dm_motor 的平滑角度
volatile uint8_t dm_status = 0;           // 用于监测达妙电机状态(1为使能)
volatile float dm_pos = 0.0f;             // 用于监测达妙电机反馈位置
volatile int32_t dm_pos_int = 0;          // dm当前位置放大1000倍后的整型值，便于FreeMaster观察细小误差
volatile float yaw_current_deg = 0.0f;
volatile int32_t dm_smoothed_angle_int = 0;  // 放大1000倍的整型角度，防止FreeMaster将浮点数按整数解析导致乱抖
volatile float glb_servo_1_target = DartRack::kServo1Init + 474.886f;  // 用于在 FreeMaster 监测舵机1目标角度
volatile float glb_servo_2_target = DartRack::kServo2Init + 190.831f;  // 用于在 FreeMaster 监测舵机2目标角度
volatile float glb_add_motor_linear = 0;
volatile uint32_t g_fire_running_time = 0;  // 全局变量，记录撒放器运行时间
static bool g_all_darts_completed = false;
volatile float trigger_motor_linear = 0.0f;
volatile float glb_aim_yaw_speed = 0.0f;
volatile int32_t glb_aim_yaw_pid_out = 0;
volatile int32_t trigger_motor_force_odometer_time = 0;
volatile int32_t trigger_motor_force_running_time = 0;
volatile int32_t glb_load_l_linear_ticks = 0;  // load左电机linear_ticks
volatile int32_t glb_load_r_linear_ticks = 0;  // load右电机linear_ticks
// aim阶段yaw调试变量
volatile float glb_aim_pixel_error = 0.0f;      // 像素误差
volatile float glb_aim_current_pixel = 0.0f;    // 当前视觉像素
volatile float glb_aim_target_pixel = 0.0f;     // 目标像素
volatile uint8_t glb_aim_yaw_done = 0;          // yaw到位标志
volatile uint8_t glb_aim_yaw_fail = 0;          // yaw逼近失败标志
volatile float glb_aim_yaw_deg = 0.0f;          // yaw编码器角度
volatile int32_t glb_aim_yaw_target_speed = 0;  // yaw目标速度

constexpr int32_t kTriggerDeadZone = 10000;  // trigger 到达死区(ticks)
volatile int32_t glb_trigger_motor_force_linear_ticks = 0;
static uint8_t add_backoff_state = 0;  // add_motor 回位状态：0=未开始, 1=反转撞限位, 2=正转脱离限位, 3=完成
static uint8_t trigger_backoff_state = 0;
volatile float yaw_deg = 100.0f;
volatile int16_t rpm = 0;
volatile int32_t rpm_trigger = 0;
volatile float angle_out = 0.0f;
volatile float speed_out = 0.0f;
volatile float yaw_vision = 0.0f;
volatile int32_t trigger_ticks = 0;
static bool trigger_done = false;
static uint8_t i = 0;
static bool trigger_pressed = false;
static bool yaw_finished = false;
static bool trigger_finished = false;
volatile int16_t current_load_l = 0;
volatile int16_t current_load_r = 0;
static bool open_lauch_state = false;
static bool close_lauch_state = false;
volatile int8_t current_fire_count = 0;
void DartStateMachineUpdate(DartState &state) {
  if (g_all_darts_completed) {
    if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kDown &&
        dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kDown) {
      g_all_darts_completed = false;
    }
    DartStateClear(state);
    DartStateUnableUpdate();
    return;
  }

  if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kDown) {
    // 无力状态
    state.unable = AbleState::kOn;
    state.manual_mode.enabled = AbleState::kOff;
    state.showtime_mode.enabled = AbleState::kOff;
    state.lvgl_mode.enabled = AbleState::kOff;
    state.add_adjust_mode.enabled = AbleState::kOff;
    state.adjust_mode.enabled = AbleState::kOff;
  } else if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kMid &&
             dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kUp) {
    // 左拨杆向中，自动打镖模式 (只初始化yaw轴)
    state.unable = AbleState::kOff;
    if (state.manual_mode.enabled != AbleState::kOn) {
      DartManualModeClear(state.manual_mode);
    }
    state.manual_mode.enabled = AbleState::kOn;
    state.showtime_mode.enabled = AbleState::kOff;
    state.lvgl_mode.enabled = AbleState::kOff;
    state.add_adjust_mode.enabled = AbleState::kOff;
    state.adjust_mode.enabled = AbleState::kOff;
  } else if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kUp &&
             dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kUp) {
    // 左右拨杆向上，自动模式(开启LVGL) (局间调节)
    state.unable = AbleState::kOff;
    state.manual_mode.enabled = AbleState::kOff;
    state.showtime_mode.enabled = AbleState::kOff;
    if (state.lvgl_mode.enabled != AbleState::kOn) {
      DartManualModeClear(state.manual_mode);
    }
    state.lvgl_mode.enabled = AbleState::kOn;
    state.add_adjust_mode.enabled = AbleState::kOff;
    state.adjust_mode.enabled = AbleState::kOff;
  } else if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kUp &&
             dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kMid) {
    // 左拨杆向上，右拨杆向中，调节除换弹之外的所有电机
    state.unable = AbleState::kOff;
    state.manual_mode.enabled = AbleState::kOff;
    state.showtime_mode.enabled = AbleState::kOff;
    state.lvgl_mode.enabled = AbleState::kOff;
    state.add_adjust_mode.enabled = AbleState::kOff;
    state.adjust_mode.enabled = AbleState::kOn;
  } else if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kMid &&
             dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kMid) {
    // 左右拨杆均在中间，换弹装置手动调节模式
    state.unable = AbleState::kOff;
    state.manual_mode.enabled = AbleState::kOff;
    state.showtime_mode.enabled = AbleState::kOff;
    state.lvgl_mode.enabled = AbleState::kOff;
    state.add_adjust_mode.enabled = AbleState::kOn;
    state.adjust_mode.enabled = AbleState::kOff;
  }

  // 状态机处理逻辑
  is_lvgl_running = false;

  if (state.unable == AbleState::kOn) {
    DartStateClear(state);
    DartStateUnableUpdate();
  } else if (state.manual_mode.enabled == AbleState::kOn) {
    DartStateManualUpdate();
  } else if (state.lvgl_mode.enabled == AbleState::kOn) {
    is_lvgl_running = true;
    dart_rack->load_motor_l_->SetCurrent(0);
    dart_rack->load_motor_r_->SetCurrent(0);
    dart_rack->trigger_motor_->SetCurrent(0);
    dart_rack->trigger_motor_force_->SetCurrent(0);
    dart_rack->add_motor_->SetCurrent(0);
    dart_rack->yaw_motor_->SetCurrent(0);
    {
      static uint32_t lvgl_tick = 0;
      lvgl_tick++;
      if (lvgl_tick % 20 == 0) {
        dart_rack->add_servo_1_->MoveTime(static_cast<uint16_t>(DartRack::kServo1Init + 435.927f), 0);
      } else if (lvgl_tick % 20 == 10) {
        dart_rack->add_servo_2_->MoveTime(static_cast<uint16_t>(DartRack::kServo2Init + 160.781f), 0);
      }
    }
  } else if (state.adjust_mode.enabled == AbleState::kOn) {
    DartStateAdjustUpdate();
  } else if (state.add_adjust_mode.enabled == AbleState::kOn) {
    DartStateAddAdjustUpdate();
  } else {
    DartStateClear(state);
  }
}

// 手动模式状态机处理
void DartStateManualUpdate() {
  switch (dart_rack->state_.manual_mode.mode) {
    case ModeState::kUnable:
      if (dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kUp) {
        dart_rack->state_.manual_mode.mode = ModeState::kInit;
      } else {
        DartStateUnableUpdate();
      }
      break;
    case ModeState::kInit:
      // 初始化逻辑 - 只有第一发执行
      if (dart_rack->dart_count_ != DartCount::kFirst) {
        // 非第一发延时500个周期再进入load状态
        // 如果是first_fire
        static uint32_t init_delay_cnt = 0;
        init_delay_cnt++;
        if (init_delay_cnt >= 500) {
          init_delay_cnt = 0;
          dart_rack->state_.manual_mode.init = PhaseState::kDone;
          dart_rack->state_.manual_mode.mode = ModeState::kload;
        }
      } else if (dart_rack->state_.manual_mode.init == PhaseState::kUncomplete) {
        DartStateInitUpdate();
      } else if (dart_rack->state_.manual_mode.init == PhaseState::kDone) {
        dart_rack->state_.manual_mode.mode = ModeState::kload;
      }
      break;
    case ModeState::kload: {
      // 舱门状态检测
      static uint8_t prev_launch_status = 1;  // 上一次舱门状态，初始为关闭
      uint8_t cur_launch_status = glb_dart_launch_opening_status;

      // 检测开启过程(从过渡到开启)
      if (prev_launch_status == 1 && cur_launch_status == 2) {
        open_lauch_state = true;
      }
      // 检测关闭过程(从过渡到关闭)
      if (prev_launch_status == 2 && cur_launch_status == 1) {
        close_lauch_state = true;
      }
      // 读完一个开启关闭之后重置状态位，current_fire_count加1
      if (open_lauch_state && close_lauch_state) {
        open_lauch_state = false;
        close_lauch_state = false;
        current_fire_count++;
      }
      prev_launch_status = cur_launch_status;

      //屏蔽第四发

      // if (dart_rack->dart_count_ == DartCount::kFourth) {
      //     return;
      // }

      if (dart_rack->dart_count_ == DartCount::kThird) {
        if (current_fire_count == 0) {
          return;
        }
        if (current_fire_count == 1 && open_lauch_state == false) {
          return;
        }
      }
      // 第三发 第二次开舱门 才不return
      if (dart_rack->state_.manual_mode.load == PhaseState::kUncomplete) {
        // 第三发镖开始时，如果还没读到第二次舱门开启，就不进入loadupdate
        if (glb_dart_launch_opening_status != 1) {
          DartStateLoadUpdate();
        }
      } else if (dart_rack->state_.manual_mode.load == PhaseState::kDone) {
        dart_rack->state_.manual_mode.mode = ModeState::kAdd;
      }
      break;
    }

    case ModeState::kAdd:
      if (dart_rack->state_.manual_mode.add == PhaseState::kUncomplete) {
        if (dart_rack->dart_count_ == DartCount::kFirst) {
          dart_rack->state_.manual_mode.add = PhaseState::kDone;
        } else if (dart_rack->dart_count_ == DartCount::kSecond) {
          DartStateAddPlaceOnly();
        } else {
          DartStateAddUpdate();
          // dart_rack->state_.manual_mode.add = PhaseState::kDone;
        }
      } else if (dart_rack->state_.manual_mode.add == PhaseState::kDone) {
        // if (dart_rack->rc_->right_x() == 660) {
        // dart_rack->state_.manual_mode.mode = ModeState::kAim;
        //}
        dart_rack->state_.manual_mode.mode = ModeState::kAim;
      }
      break;

    case ModeState::kAim:
      if (dart_rack->state_.manual_mode.aim == PhaseState::kUncomplete) {
        DartStateAimUpdate();
        // dart_rack->state_.manual_mode.aim = PhaseState::kDone;
      } else if (dart_rack->state_.manual_mode.aim == PhaseState::kDone) {
        dart_rack->state_.manual_mode.mode = ModeState::kFire;
      }
      break;
    case ModeState::kFire: {
      // 发射逻辑
      if (dart_rack->state_.manual_mode.fire == PhaseState::kUncomplete) {
        if (glb_dart_launch_opening_status == 0) {
          DartStateFireUpdate();
        }
        // if (dart_rack->rc_->left_x() == 660 && dart_rack->rc_->right_x() == -660) {
        //   DartStateFireUpdate();
        // }
      } else if (dart_rack->state_.manual_mode.fire == PhaseState::kDone) {
        uint8_t next_dart = static_cast<uint8_t>(dart_rack->dart_count_) + 1;
        if (next_dart > 3) {
          dart_rack->dart_count_ = DartCount::kFirst;
          g_all_darts_completed = true;
        } else {
          dart_rack->dart_count_ = static_cast<DartCount>(next_dart);
        }
        DartManualModeClear(dart_rack->state_.manual_mode);
      }
      break;
    }
    default:
      break;
  }
}

void DartStateUnableUpdate() {
  dart_rack->yaw_motor_->SetCurrent(0);
  dart_rack->load_motor_l_->SetCurrent(0);
  dart_rack->load_motor_r_->SetCurrent(0);
  dart_rack->trigger_motor_->SetCurrent(0);
  dart_rack->trigger_motor_force_->SetCurrent(0);
  dart_rack->add_motor_->SetCurrent(0);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);

  // 更新达妙电机位置，供FreeMaster观测
  dm_pos = dart_rack->dm_motor_->pos();
  dm_pos_int = static_cast<int32_t>(dm_pos * 1000.0f);

  // 周期性读取舵机位置，错开发送避免半双工总线冲突
  static uint16_t servo_poll_cnt = 0;
  if (++servo_poll_cnt >= 1000) {
    servo_poll_cnt = 0;
  }
  if (servo_poll_cnt == 0) {
    dart_rack->add_servo_1_->ReadPosition();
    glb_servo_1_target = static_cast<float>(dart_rack->add_servo_1_->feedback().position);
  } else if (servo_poll_cnt == 500) {
    dart_rack->add_servo_2_->ReadPosition();
    glb_servo_2_target = static_cast<float>(dart_rack->add_servo_2_->feedback().position);
  }
  dm_pos_int = static_cast<int32_t>(dm_pos * 1000.0f);

  // 只刷新一次，固定显示内容防止持续刷新
  static bool lcd_displayed = false;
  if (!lcd_displayed) {
    LCD_DISPLAY();
    lcd_displayed = true;
  }
}

void DartStateInitUpdate() {
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
  extern uint8_t g_trigger_backoff;
  extern uint8_t g_add_backoff;
  extern uint8_t g_load_l_backoff;
  extern uint8_t g_load_r_backoff;

  // 阶段一：归位 load_l, load_r, trigger, add 电机
  if (!dart_rack->state_.manual_mode.is_load_l_reset_done) {
    if (g_load_l_limit_ever_hit) {
      dart_rack->load_motor_l_speed_pid_.Clear();
      dart_rack->load_motor_l_->SetCurrent(0);
      dart_rack->load_motor_l_odometer_.Reset();
      dart_rack->state_.manual_mode.is_load_l_reset_done = true;
    } else if (!g_load_l_backoff) {
      dart_rack->load_motor_l_speed_pid_.Update(200.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
      dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
    }
  }

  if (!dart_rack->state_.manual_mode.is_load_r_reset_done) {
    if (g_load_r_limit_ever_hit) {
      dart_rack->load_motor_r_speed_pid_.Clear();
      dart_rack->load_motor_r_->SetCurrent(0);
      dart_rack->load_motor_r_odometer_.Reset();
      dart_rack->state_.manual_mode.is_load_r_reset_done = true;
    } else if (!g_load_r_backoff) {
      dart_rack->load_motor_r_speed_pid_.Update(-200.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
      dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
    }
  }

  if (!dart_rack->state_.manual_mode.is_trigger_reset_done) {
    if (g_trigger_limit_ever_hit) {
      dart_rack->trigger_motor_speed_pid_.Clear();
      dart_rack->trigger_motor_->SetCurrent(0);
      dart_rack->trigger_motor_odometer_.Reset();
      dart_rack->state_.manual_mode.is_trigger_reset_done = true;
    } else if (!g_trigger_backoff) {
      dart_rack->trigger_motor_speed_pid_.Update(2500.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
      dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
    }
  }

  if (!dart_rack->state_.manual_mode.is_add_init_done) {
    if (g_add_limit_ever_hit) {
      dart_rack->add_motor_speed_pid_.Clear();
      dart_rack->add_motor_->SetCurrent(0);
      dart_rack->add_motor_odometer_.Reset();
      dart_rack->state_.manual_mode.is_add_init_done = true;
    } else if (!g_add_backoff) {
      dart_rack->add_motor_speed_pid_.Update(-3000.0f, dart_rack->add_motor_->rpm(), 1.0f);
      dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
    }
  }

  if (dart_rack->state_.manual_mode.is_load_reset_done == false && dart_rack->state_.manual_mode.is_load_l_reset_done &&
      dart_rack->state_.manual_mode.is_load_r_reset_done) {
    dart_rack->state_.manual_mode.is_load_reset_done = true;
  }

  dart_rack->state_.manual_mode.is_trigger_force_init_done = true;

  // 初始化过程中持续发送舵机指令，保持位置
  static uint32_t servo_tick = 0;
  servo_tick++;
  if (servo_tick % 20 == 0) {
    dart_rack->add_servo_1_->MoveTime(static_cast<uint16_t>(DartRack::kServo1Init + 435.927f), 0);
  } else if (servo_tick % 20 == 10) {
    dart_rack->add_servo_2_->MoveTime(static_cast<uint16_t>(DartRack::kServo2Init + 160.781f), 0);
  }

  // Yaw轴不做转动，直接标记完成并无力
  dart_rack->yaw_motor_speed_pid_.Clear();
  dart_rack->yaw_motor_->SetCurrent(0);
  dart_rack->state_.manual_mode.is_yaw_init_done = true;

  // 全部检查完成后，初始化完成
  if (dart_rack->state_.manual_mode.is_yaw_init_done && dart_rack->state_.manual_mode.is_load_reset_done &&
      dart_rack->state_.manual_mode.is_trigger_reset_done && dart_rack->state_.manual_mode.is_trigger_force_init_done &&
      dart_rack->state_.manual_mode.is_add_init_done) {
    dart_rack->state_.manual_mode.init = PhaseState::kDone;
    dart_rack->yaw_motor_speed_pid_.Clear();
    dart_rack->yaw_motor_->SetCurrent(0);
    dart_rack->add_servo_1_->MoveTime(static_cast<uint16_t>(DartRack::kServo1Init + 435.927f), 0);
    dart_rack->add_servo_2_->MoveTime(static_cast<uint16_t>(DartRack::kServo2Init + 160.781f), 0);
  }
}

void DartStateLoadUpdate() {
  // 上膛逻辑
  current_load_l = dart_rack->load_motor_l_->current();
  current_load_r = dart_rack->load_motor_r_->current();

  // 0. 第三发第四发：add_motor 回位逻辑
  // 先反转撞限位，再正转脱离限位，完成后不再执行
  if (dart_rack->dart_count_ == DartCount::kThird || dart_rack->dart_count_ == DartCount::kFourth) {
    uint8_t add_pressed = (HAL_GPIO_ReadPin(add_motor_EXTI_GPIO_Port, add_motor_EXTI_Pin) == GPIO_PIN_RESET);
    if (add_backoff_state == 0) {
      // 开始反转撞限位
      add_backoff_state = 1;
    } else if (add_backoff_state == 1) {
      // 反转直到撞到限位
      if (!add_pressed) {
        dart_rack->add_motor_speed_pid_.Update(-3000.0f, dart_rack->add_motor_->rpm(), 1.0f);
        dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
      } else {
        // 撞到限位，重置 odometer，切换到正转脱离
        dart_rack->add_motor_odometer_.Reset();
        add_backoff_state = 2;
      }
    } else if (add_backoff_state == 2) {
      // 正转直到限位断开
      if (add_pressed) {
        dart_rack->add_motor_speed_pid_.Update(3000.0f, dart_rack->add_motor_->rpm(), 1.0f);
        dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
      } else {
        // 限位断开，停止电机，标记完成
        dart_rack->add_motor_speed_pid_.Clear();
        dart_rack->add_motor_->SetCurrent(0);
        add_backoff_state = 3;
      }
    }
  }

  // 复位逻辑（只在非第一发时执行）
  trigger_pressed = (HAL_GPIO_ReadPin(trigger_motor_EXTI_GPIO_Port, trigger_motor_EXTI_Pin) == GPIO_PIN_RESET);
  if (dart_rack->dart_count_ != DartCount::kFirst && trigger_backoff_state != 3) {
    if (trigger_backoff_state == 0) {
      // 开始反转撞限位
      trigger_backoff_state = 1;
    } else if (trigger_backoff_state == 1) {
      // 反转直到撞到限位
      if (!trigger_pressed) {
        dart_rack->trigger_motor_speed_pid_.Update(4000.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
        dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
      } else {
        // 撞到限位，重置 odometer，切换到正转脱离
        dart_rack->trigger_motor_odometer_.Reset();
        trigger_backoff_state = 2;
      }
    } else if (trigger_backoff_state == 2) {
      // 正转直到限位断开
      if (trigger_pressed) {
        dart_rack->trigger_motor_speed_pid_.Update(-3000.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
        dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
      } else {
        // 限位断开，停止电机，标记复位完成
        dart_rack->trigger_motor_speed_pid_.Update(0.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
        dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
        if (std::abs(rpm_trigger) < 50) {
          static int32_t trigger_cnt = 0;
          trigger_cnt++;
          if (trigger_cnt >= 500) {
            trigger_cnt = 0;
            trigger_backoff_state = 3;
            dart_rack->trigger_motor_speed_pid_.Clear();
            dart_rack->trigger_motor_->SetCurrent(0);
          }
        }
      }
    }
  }

  // trigger 复位完成后才执行后续逻辑
  bool trigger_backoff_done = (dart_rack->dart_count_ == DartCount::kFirst) || (trigger_backoff_state == 3);

  // 1. 撒放器锁定（反转撞限位）- 先反转堵转，堵转后reset odometer设为零点
  if (trigger_backoff_done && dart_rack->state_.manual_mode.is_trigger_lock_done == false) {
    if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 50) {
      dart_rack->trigger_motor_force_pid_.Update(-3000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
    } else {
      // 堵转结束，撒放器锁定，reset odometer 设为零点
      dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
      dart_rack->trigger_motor_force_odometer_.Reset();
      dart_rack->state_.manual_mode.is_trigger_lock_done = true;
    }
  }

  // 2. 滑台下拉 + 撒放器打开（正转）
  if (dart_rack->state_.manual_mode.is_trigger_lock_done == true &&
      dart_rack->state_.manual_mode.is_load_down_done == false) {
    // 滑台下拉过程中打开撒放器（正转）
    if (dart_rack->trigger_motor_force_odometer_.linear_ticks() < 115000 &&
        dart_rack->trigger_motor_force_odometer_.stall_time() <= 50) {
      dart_rack->trigger_motor_force_pid_.Update(3000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
    } else {
      dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
    }

    // 滑台下拉判断：linear_ticks 或堵转
    if (dart_rack->load_motor_l_odometer_.stall_time() >= 150 &&
        dart_rack->load_motor_r_odometer_.stall_time() >= 150) {
      dart_rack->load_motor_l_speed_pid_.Clear();
      dart_rack->load_motor_r_speed_pid_.Clear();
      dart_rack->load_motor_l_->SetCurrent(0);
      dart_rack->load_motor_r_->SetCurrent(0);
      dart_rack->state_.manual_mode.is_load_down_done = true;
    } else if (dart_rack->load_motor_l_odometer_.stall_time() <= 150 &&
               dart_rack->load_motor_r_odometer_.stall_time() <= 150) {
      if (dart_rack->load_motor_r_odometer_.linear_ticks() > DartRack::kTriggerEcdMax ||
          dart_rack->load_motor_l_odometer_.linear_ticks() < -DartRack::kTriggerEcdMax) {
        dart_rack->load_motor_l_speed_pid_.Update(-1000.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
        dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
        dart_rack->load_motor_r_speed_pid_.Update(1000.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
        dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
      } else {
        dart_rack->load_motor_l_speed_pid_.Update(-7000.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
        dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
        dart_rack->load_motor_r_speed_pid_.Update(7000.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
        dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
      }
    } else {
      dart_rack->load_motor_l_speed_pid_.Update(.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
      dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
      dart_rack->load_motor_r_speed_pid_.Update(.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
      dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
      dart_rack->state_.manual_mode.is_load_down_done = true;
    }
  }

  // 3. 滑台到位后，撒放器再次锁定（反转到0）
  if (dart_rack->state_.manual_mode.is_load_down_done == true &&
      dart_rack->state_.manual_mode.is_trigger_relock_done == false) {
    if (dart_rack->trigger_motor_force_odometer_.linear_ticks() > 0 &&
        dart_rack->trigger_motor_force_odometer_.stall_time() <= 50) {
      dart_rack->trigger_motor_force_pid_.Update(-3000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
    } else {
      dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
      dart_rack->state_.manual_mode.is_trigger_relock_done = true;
    }
  }

  // 4. 滑台上滑
  if (dart_rack->state_.manual_mode.is_trigger_relock_done == true &&
      dart_rack->state_.manual_mode.is_load_up_done == false) {
    const int32_t l_ticks = dart_rack->load_motor_l_odometer_.linear_ticks();
    const int32_t r_ticks = dart_rack->load_motor_r_odometer_.linear_ticks();
    constexpr int32_t kBrakeZone = 200000;
    constexpr int32_t kDeadZone = 50000;
    dart_rack->trigger_motor_force_pid_.Clear();
    dart_rack->trigger_motor_force_->SetCurrent(0);
    if (l_ticks < -kDeadZone && dart_rack->load_motor_l_odometer_.stall_time() <= 100) {
      float ratio = std::min(1.0f, static_cast<float>(-l_ticks) / static_cast<float>(kBrakeZone));
      float l_speed = 1000.0f + 6000.0f * ratio;
      dart_rack->load_motor_l_speed_pid_.Update(l_speed, dart_rack->load_motor_l_->rpm(), 1.0f);
      dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
    } else {
      dart_rack->load_motor_l_speed_pid_.Update(0.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
      dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
    }

    if (r_ticks > kDeadZone && dart_rack->load_motor_r_odometer_.stall_time() <= 100) {
      float ratio = std::min(1.0f, static_cast<float>(r_ticks) / static_cast<float>(kBrakeZone));
      float r_speed = -(1000.0f + 6000.0f * ratio);
      dart_rack->load_motor_r_speed_pid_.Update(r_speed, dart_rack->load_motor_r_->rpm(), 1.0f);
      dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
    } else {
      dart_rack->load_motor_r_speed_pid_.Update(0.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
      dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
    }

    if (l_ticks >= -kDeadZone && r_ticks <= kDeadZone) {
      // 判断速度不再持续减小后再置零
      static int16_t prev_l_rpm = 0, prev_r_rpm = 0;
      static uint8_t speed_stable_cnt = 0;
      int16_t cur_l_rpm = dart_rack->load_motor_l_->rpm();
      int16_t cur_r_rpm = dart_rack->load_motor_r_->rpm();
      bool l_speed_stable = (std::abs(cur_l_rpm) >= std::abs(prev_l_rpm) || std::abs(cur_l_rpm) < 50);
      bool r_speed_stable = (std::abs(cur_r_rpm) >= std::abs(prev_r_rpm) || std::abs(cur_r_rpm) < 50);
      if (l_speed_stable && r_speed_stable) {
        speed_stable_cnt++;
      } else {
        speed_stable_cnt = 0;
      }
      prev_l_rpm = cur_l_rpm;
      prev_r_rpm = cur_r_rpm;
      if (speed_stable_cnt >= 100) {
        dart_rack->state_.manual_mode.is_load_up_done = true;
        dart_rack->load_motor_l_speed_pid_.Update(0.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
        dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
        dart_rack->load_motor_r_speed_pid_.Update(0.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
        dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
        speed_stable_cnt = 0;
      }
    }
  }
  if (dart_rack->state_.manual_mode.is_trigger_lock_done == true &&
      dart_rack->state_.manual_mode.is_trigger_relock_done == true &&
      dart_rack->state_.manual_mode.is_load_up_done == true &&
      dart_rack->state_.manual_mode.is_load_down_done == true) {
    if (add_backoff_state == 3) {
      add_backoff_state = 0;
    }
    if (trigger_backoff_state == 3) {
      trigger_backoff_state = 0;
    }
    dart_rack->load_motor_l_->SetCurrent(0);
    dart_rack->load_motor_r_->SetCurrent(0);
    dart_rack->trigger_motor_force_->SetCurrent(0);
    dart_rack->state_.manual_mode.load = PhaseState::kDone;
  }
}

void DartStateAddUpdate() {
  static uint32_t tick = 0;
  tick++;
  dm_status = dart_rack->dm_motor_->status();
  dm_pos = dart_rack->dm_motor_->pos();
  dm_pos_int = static_cast<int32_t>(dm_pos * 1000.0f);

  if (dart_rack->dm_motor_->status() != 1) {
    if (tick % 50 == 0) {
      dart_rack->dm_motor_->SendInstruction(rm::device::DmMotorInstructions::kClearError);
      dart_rack->dm_motor_->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    }
  }
  glb_add_motor_linear = dart_rack->add_motor_odometer_.linear_ticks();

  static AddState current_state = AddState::BEFORE_CAUGHT;
  static bool add_forward_done = false;
  float target_dm_angle = 0.050f;
  static rm::modules::TrajectoryLimiter dm_limiter(8.0f, 15.0f);
  static bool dm_limiter_initialized = false;
  if (!dm_limiter_initialized && dm_status == 1) {
    dm_limiter.ResetAt(dm_pos);
    dm_limiter_initialized = true;
  }
  constexpr float S1 = DartRack::kServo1Init;
  constexpr float S2 = DartRack::kServo2Init;
  float target_servo1 = 500.0f;
  float target_servo2 = 500.0f;
  static uint32_t state_timer = 0;

  switch (current_state) {
    // 阶段1：BEFORE_CAUGHT - 分解为2步
    case AddState::BEFORE_CAUGHT:
      // Step 1：先收舵机，维持 target_dm_angle 为当前位置防止误动
      state_timer++;
      target_servo1 = (dart_rack->dart_count_ == DartCount::kThird) ? S1 + 370.0f : S1 + 370.0f;
      target_servo2 = (dart_rack->dart_count_ == DartCount::kThird) ? S2 + 130.0f : S2 + 130.0f;
      if (state_timer > 300) {
        target_dm_angle = (dart_rack->dart_count_ == DartCount::kThird) ? -0.618f : 0.442f;
      }
      // 超时保护
      if (state_timer > 500) {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
        state_timer = 0;
        current_state = AddState::CAUGHT;
      }
      break;

    // 阶段2：CAUGHT - 舵机到位后
    case AddState::CAUGHT: {
      target_dm_angle = (dart_rack->dart_count_ == DartCount::kThird) ? -0.618f : 0.442f;
      state_timer++;
      // 下发舵机夹取位置
      target_servo1 = (dart_rack->dart_count_ == DartCount::kThird) ? S1 + 538.0f : S1 + 535.0f;
      target_servo2 = (dart_rack->dart_count_ == DartCount::kThird) ? S2 + 132.0f : S2 + 129.0f;
      if (state_timer > 500) {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
        target_servo1 = (dart_rack->dart_count_ == DartCount::kThird) ? S1 + 495.0f : S1 + 495.0f;
        target_servo2 = (dart_rack->dart_count_ == DartCount::kThird) ? S2 + 96.0f : S2 + 96.0f;
      }
      // 超时保护
      if (state_timer > 800) {
        state_timer = 0;
        current_state = AddState::MOVING_FORWARD;
      }
      break;
    }

    // 阶段3：MOVING_FORWARD - 舵机动、add_motor滑台前推、dm_motor回正
    case AddState::MOVING_FORWARD: {
      // 下发舵机位置
      state_timer++;
      target_servo1 = (dart_rack->dart_count_ == DartCount::kThird) ? S1 + 383.0f : S1 + 383.0f;
      target_servo2 = (dart_rack->dart_count_ == DartCount::kThird)
                          ? S2 + 112.0f
                          : S2 + 112.0f;  // 舵机到位检测：每15周期读一次，错开读取两个舵机
      static bool gpio_triggered = false;
      static uint16_t gpio_on_cnt = 0;
      if (!gpio_triggered) {
        static uint8_t poll_cnt = 0;
        static int16_t pos1 = 0, pos2 = 0;
        static int16_t prev_pos1 = 0, prev_pos2 = 0;
        poll_cnt++;
        if (poll_cnt == 1) {
          dart_rack->add_servo_1_->ReadPosition();
        } else if (poll_cnt == 2) {
          pos1 = dart_rack->add_servo_1_->feedback().position;
          dart_rack->add_servo_2_->ReadPosition();
        } else if (poll_cnt == 3) {
          pos2 = dart_rack->add_servo_2_->feedback().position;
          bool pos_reached = (std::abs(pos1 - static_cast<int16_t>(target_servo1)) < 10 &&
                              std::abs(pos2 - static_cast<int16_t>(target_servo2)) < 10);
          bool pos_stopped = (std::abs(pos1 - prev_pos1) < 10 && std::abs(pos2 - prev_pos2) < 10);
          if (pos_reached || pos_stopped) {
            gpio_triggered = true;
          }
          prev_pos1 = pos1;
          prev_pos2 = pos2;
        } else if (poll_cnt >= 20) {
          poll_cnt = 0;
        }
      }
      // 舵机到位后执行GPIO和电机操作
      if (gpio_triggered) {
        gpio_on_cnt++;
        if (gpio_on_cnt > 500) {
          target_dm_angle = -0.055f;
          target_servo1 = S1 + 463.927;
          target_servo2 = S2 + 135.781;
        }
        if (gpio_on_cnt > 500) {
          if (!add_forward_done) {
            if (dart_rack->add_motor_odometer_.linear_ticks() < 1350000) {
              dart_rack->add_motor_speed_pid_.Update(9800.0f, dart_rack->add_motor_->rpm(), 1.0f);
              dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
            } else {
              dart_rack->add_motor_->SetCurrent(0);
              dart_rack->add_motor_speed_pid_.Clear();
              add_forward_done = true;
            }
          } else {
            dart_rack->add_motor_->SetCurrent(0);
          }
        }
        if (add_forward_done) {
          add_forward_done = false;
          state_timer = 0;
          current_state = AddState::SUSPENDED;
        }
      }
      break;
    }

    case AddState::SUSPENDED:
      target_dm_angle = -0.055f;
      target_servo1 = S1 + 796.0f;
      target_servo2 = S2 + 390.0f;
      dart_rack->add_motor_->SetCurrent(0);
      state_timer++;
      if (state_timer > 300) {
        state_timer = 0;
        current_state = AddState::PLACED;
      }
      break;

    case AddState::PLACED: {
      static bool gpio_triggered = false;
      static uint16_t gpio_on_cnt = 0;
      target_dm_angle = (dart_rack->dart_count_ == DartCount::kThird) ? -0.042f : -0.042f;
      target_servo1 = S1 + 850.0f;
      target_servo2 = S2 + 457.0f;

      // 舵机到位检测：每15周期读一次，错开读取两个舵机
      if (!gpio_triggered) {
        static uint8_t poll_cnt = 0;
        static int16_t pos1 = 0, pos2 = 0;
        static int16_t prev_pos1 = 0, prev_pos2 = 0;
        poll_cnt++;
        if (poll_cnt == 1) {
          dart_rack->add_servo_1_->ReadPosition();
        } else if (poll_cnt == 2) {
          pos1 = dart_rack->add_servo_1_->feedback().position;
          dart_rack->add_servo_2_->ReadPosition();
        } else if (poll_cnt == 3) {
          pos2 = dart_rack->add_servo_2_->feedback().position;
          bool pos_reached = (std::abs(pos1 - static_cast<int16_t>(target_servo1)) < 10 &&
                              std::abs(pos2 - static_cast<int16_t>(target_servo2)) < 10);
          bool pos_stopped = (std::abs(pos1 - prev_pos1) < 10 && std::abs(pos2 - prev_pos2) < 10);
          if (pos_reached || pos_stopped) {
            gpio_triggered = true;
          }
          prev_pos1 = pos1;
          prev_pos2 = pos2;
        } else if (poll_cnt >= 20) {
          poll_cnt = 0;
        }
      }

      // 舵机到位后执行GPIO和电机操作
      if (gpio_triggered) {
        gpio_on_cnt++;
        if (gpio_on_cnt > 50) {
          HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
          if (gpio_on_cnt > 300) {
            target_servo1 = S1 + 463.927f;
            target_servo2 = S2 + 135.781f;
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
          }
        }
        if (gpio_on_cnt > 500) {
          dart_rack->add_motor_->SetCurrent(0);
          HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
          gpio_on_cnt = 0;
          gpio_triggered = false;
          tick = 0;
          add_forward_done = false;
          dart_rack->state_.manual_mode.add = PhaseState::kDone;
          current_state = AddState::BEFORE_CAUGHT;
          return;
        }
      }
      break;
    }
  }

  if (tick % 50 != 0 || dart_rack->dm_motor_->status() == 1) {
    dm_limiter.SetTarget(target_dm_angle);
    float dm_smooth = dm_limiter.Update(0.001f);
    // 把之前写死的 0.0f 的前馈速度换成规划器内部给出的当期阶跃速度 current_velocity()
    // 更新了 Kp=15.0, Kd=0.2 减小电机抵抗外部扭矩和抖动
    dart_rack->dm_motor_->SetMitCommand(dm_smooth, dm_limiter.current_velocity(), 0.0f, 25.0f, 1.0f);

    if (tick % 20 == 0) {
      dart_rack->add_servo_1_->MoveTime(static_cast<uint16_t>(target_servo1), 0);
    } else if (tick % 20 == 10) {
      dart_rack->add_servo_2_->MoveTime(static_cast<uint16_t>(target_servo2), 0);
    }
  }
  glb_add_motor_linear = dart_rack->add_motor_odometer_.linear_ticks();
}

void DartStateAddPlaceOnly() {
  static uint32_t tick = 0;
  tick++;
  dm_status = dart_rack->dm_motor_->status();
  dm_pos = dart_rack->dm_motor_->pos();
  dm_pos_int = static_cast<int32_t>(dm_pos * 1000.0f);

  if (dart_rack->dm_motor_->status() != 1) {
    if (tick % 50 == 0) {
      dart_rack->dm_motor_->SendInstruction(rm::device::DmMotorInstructions::kClearError);
      dart_rack->dm_motor_->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    }
  }
  glb_add_motor_linear = dart_rack->add_motor_odometer_.linear_ticks();

  static AddPlaceOnlyState current_state = AddPlaceOnlyState::MOVING_FORWARD;
  static bool add_forward_done = false;
  float target_dm_angle = 0.050f;
  static rm::modules::TrajectoryLimiter dm_limiter(8.0f, 15.0f);
  static bool dm_limiter_initialized = false;
  if (!dm_limiter_initialized && dm_status == 1) {
    dm_limiter.ResetAt(dm_pos);
    dm_limiter_initialized = true;
  }
  constexpr float S1 = DartRack::kServo1Init;
  constexpr float S2 = DartRack::kServo2Init;
  float target_servo1 = 500.0f;
  float target_servo2 = 500.0f;
  static uint32_t state_timer = 0;

  switch (current_state) {
    case AddPlaceOnlyState::MOVING_FORWARD:
      target_dm_angle = -0.055f;
      target_servo1 = S1 + 463.927f;
      target_servo2 = S2 + 135.781f;
      if (!add_forward_done) {
        if (dart_rack->add_motor_odometer_.linear_ticks() < 1350000) {
          dart_rack->add_motor_speed_pid_.Update(9800.0f, dart_rack->add_motor_->rpm(), 1.0f);
          dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
        } else {
          dart_rack->add_motor_->SetCurrent(0);
          dart_rack->add_motor_speed_pid_.Clear();
          add_forward_done = true;
        }
      } else {
        dart_rack->add_motor_->SetCurrent(0);
      }
      if (add_forward_done) {
        add_forward_done = false;
        state_timer = 0;
        current_state = AddPlaceOnlyState::SUSPENDED;
      }
      break;

    case AddPlaceOnlyState::SUSPENDED:
      target_dm_angle = -0.055f;
      target_servo1 = S1 + 796.0f;
      target_servo2 = S2 + 390.0f;
      dart_rack->add_motor_->SetCurrent(0);
      state_timer++;
      if (state_timer > 300) {
        state_timer = 0;
        current_state = AddPlaceOnlyState::PLACED;
      }
      break;

    case AddPlaceOnlyState::PLACED: {
      static bool gpio_triggered = false;
      static uint16_t gpio_on_cnt = 0;
      target_dm_angle = -0.042f;
      target_servo1 = S1 + 850.0f;
      target_servo2 = S2 + 457.0f;
      dart_rack->add_motor_->SetCurrent(0);

      // 舵机到位检测：每15周期读一次，错开读取两个舵机
      if (!gpio_triggered) {
        static uint8_t poll_cnt = 0;
        static int16_t pos1 = 0, pos2 = 0;
        static int16_t prev_pos1 = 0, prev_pos2 = 0;
        poll_cnt++;
        if (poll_cnt == 1) {
          dart_rack->add_servo_1_->ReadPosition();
        } else if (poll_cnt == 2) {
          pos1 = dart_rack->add_servo_1_->feedback().position;
          dart_rack->add_servo_2_->ReadPosition();
        } else if (poll_cnt == 3) {
          pos2 = dart_rack->add_servo_2_->feedback().position;
          bool pos_reached = (std::abs(pos1 - static_cast<int16_t>(target_servo1)) < 10 &&
                              std::abs(pos2 - static_cast<int16_t>(target_servo2)) < 10);
          bool pos_stopped = (std::abs(pos1 - prev_pos1) < 5 && std::abs(pos2 - prev_pos2) < 10);
          if (pos_reached || pos_stopped) {
            gpio_triggered = true;
          }
          prev_pos1 = pos1;
          prev_pos2 = pos2;
        } else if (poll_cnt >= 10) {
          poll_cnt = 0;
        }
      }

      // 舵机到位后执行GPIO和电机操作
      if (gpio_triggered) {
        gpio_on_cnt++;
        if (gpio_on_cnt > 100) {
          HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
          if (gpio_on_cnt > 300) {
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
            target_servo1 = S1 + 463.927f;
            target_servo2 = S2 + 135.781f;
            dart_rack->trigger_motor_->SetCurrent(0);
            dart_rack->trigger_motor_speed_pid_.Clear();
          }
        }
        if (gpio_on_cnt > 500) {
          dart_rack->trigger_motor_->SetCurrent(0);
          dart_rack->add_motor_->SetCurrent(0);
          HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
          gpio_on_cnt = 0;
          gpio_triggered = false;
          tick = 0;
          add_forward_done = false;
          dart_rack->state_.manual_mode.add = PhaseState::kDone;
          current_state = AddPlaceOnlyState::MOVING_FORWARD;
          return;
        }
      }
    } break;
  }

  if (tick % 50 != 0 || dart_rack->dm_motor_->status() == 1) {
    dm_limiter.SetTarget(target_dm_angle);
    float dm_smooth = dm_limiter.Update(0.001f);
    dart_rack->dm_motor_->SetMitCommand(dm_smooth, dm_limiter.current_velocity(), 0.0f, 25.0f, 1.0f);

    if (tick % 20 == 0) {
      dart_rack->add_servo_1_->MoveTime(static_cast<uint16_t>(target_servo1), 0);
    } else if (tick % 20 == 10) {
      dart_rack->add_servo_2_->MoveTime(static_cast<uint16_t>(target_servo2), 0);
    }
  }

  glb_add_motor_linear = dart_rack->add_motor_odometer_.linear_ticks();
}

void DartStateAimUpdate() {
  yaw_deg = dart_rack->yaw_encoder_->angle_deg();
  rpm = dart_rack->yaw_motor_->rpm();
  rpm_trigger = dart_rack->trigger_motor_->rpm();
  yaw_vision = dart_rack->vision_data_->Yaw;
  static bool yaw_approach_suspended = false;
  i = static_cast<uint8_t>(dart_rack->dart_count_);
  constexpr float tolerance = 5.0f;
  constexpr float PerWidth[4] = {-20.0f,-20.0f, -20.0f, -20.0f};
  constexpr int32_t PerHeight[4] = {-2500000, -2500000, -2500000, -2500000};
  // 以下为正常推进逻辑（复位完成后或第一发时执行）

  // error为正,往右边,error为负,往左边
  // 先更新角度环,再更新速度环
  dart_rack->yaw_motor_angle_pid_.Update(PerWidth[i], yaw_vision, 1.0f);
  angle_out = dart_rack->yaw_motor_angle_pid_.out();
  dart_rack->yaw_motor_speed_pid_.Update(angle_out, dart_rack->yaw_motor_->rpm(), 1.0f);
  speed_out = dart_rack->yaw_motor_speed_pid_.out();
  dart_rack->yaw_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->yaw_motor_speed_pid_.out()));

  // trigger
  trigger_ticks = dart_rack->trigger_motor_odometer_.linear_ticks();

  if (trigger_ticks > PerHeight[i]) {
    dart_rack->trigger_motor_speed_pid_.Update(-8000.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
    dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
  } else {
    dart_rack->trigger_motor_speed_pid_.Update(0.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
    dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
    trigger_done = true;
  }
  if (trigger_done) {
    if (std::abs(rpm_trigger) < 50) {
      static int32_t trigger_cnt = 0;
      trigger_cnt++;
      if (trigger_cnt >= 500) trigger_cnt = 0;
      dart_rack->trigger_motor_speed_pid_.Clear();
      dart_rack->trigger_motor_->SetCurrent(0);
      trigger_finished = true;
    }
  }

  if (yaw_deg >= DartRack::kYawEcdMax || yaw_deg <= DartRack::kYawEcdMin) {
    yaw_approach_suspended = true;
  }
  if (std::abs(yaw_vision - PerWidth[i]) < tolerance || yaw_approach_suspended) {
    dart_rack->yaw_motor_speed_pid_.Update(0.0f, dart_rack->yaw_motor_->rpm(), 1.0f);
    dart_rack->yaw_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->yaw_motor_speed_pid_.out()));
    if (std::abs(rpm) < 50) {
      static int32_t yaw_cnt = 0;
      yaw_cnt++;
      if (yaw_cnt >= 500) yaw_cnt = 0;
      dart_rack->yaw_motor_angle_pid_.Clear();
      dart_rack->yaw_motor_speed_pid_.Clear();
      dart_rack->yaw_motor_->SetCurrent(0);
      yaw_finished = true;
    }
  }
  if (trigger_finished && yaw_finished) {
    dart_rack->yaw_motor_angle_pid_.Clear();
    dart_rack->yaw_motor_speed_pid_.Clear();
    dart_rack->yaw_motor_->SetCurrent(0);
    dart_rack->trigger_motor_speed_pid_.Clear();
    dart_rack->trigger_motor_->SetCurrent(0);
    dart_rack->state_.manual_mode.aim = PhaseState::kDone;

    // 重置所有静态变量，为下一次瞄准做准备
    yaw_approach_suspended = false;
    trigger_finished = false;
    yaw_finished = false;
    trigger_done = false;
    trigger_backoff_state = 0;
  }
}

void DartStateFireUpdate() {
  static uint32_t fire_running_time = 0;
  if (dart_rack->state_.manual_mode.fire == PhaseState::kUncomplete) {
    fire_running_time++;
    if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 30 && fire_running_time < 250) {
      dart_rack->trigger_motor_force_pid_.Update(4000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
    } else {
      dart_rack->trigger_motor_force_->SetCurrent(0);
      dart_rack->state_.manual_mode.fire = PhaseState::kDone;
      fire_running_time = 0;  // 重置计时器
    }
  } else {
    dart_rack->trigger_motor_force_->SetCurrent(0);
    dart_rack->state_.manual_mode.fire = PhaseState::kDone;
    fire_running_time = 0;  // 重置计时器
  }
}

void DartStateAdjustUpdate() {
  // 计算包含圈数的全段实际角度
  yaw_current_deg = dart_rack->yaw_encoder_->angle_deg();
  trigger_motor_linear = dart_rack->trigger_motor_odometer_.linear_ticks();
  test_if_adjust_mode_is_running = 1.0f;
  trigger_motor_force_odometer_time = dart_rack->trigger_motor_force_odometer_.stall_time();
  // Yaw轴调节
  if (dart_rack->rc_->right_x() > 330) {
    if (yaw_current_deg <= DartRack::kYawEcdMin) {
      dart_rack->yaw_motor_->SetCurrent(0);
    } else {
      dart_rack->yaw_motor_speed_pid_.Update(-2000.0f, dart_rack->yaw_motor_->rpm(), 1.0f);
      dart_rack->yaw_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->yaw_motor_speed_pid_.out()));
    }
  } else if (dart_rack->rc_->right_x() < -330) {
    if (yaw_current_deg >= DartRack::kYawEcdMax) {
      dart_rack->yaw_motor_->SetCurrent(0);
    } else {
      dart_rack->yaw_motor_speed_pid_.Update(2000.0f, dart_rack->yaw_motor_->rpm(), 1.0f);
      dart_rack->yaw_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->yaw_motor_speed_pid_.out()));
    }
  } else {
    dart_rack->yaw_motor_->SetCurrent(0);
  }
  // 扳机位置调节
  if (dart_rack->rc_->right_y() > 330) {
    if (dart_rack->trigger_motor_odometer_.stall_time() <= 100) {
      dart_rack->trigger_motor_speed_pid_.Update(-4000.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
      dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
    } else {
      dart_rack->trigger_motor_->SetCurrent(0);
    }
  } else if (dart_rack->rc_->right_y() < -330) {
    if (dart_rack->trigger_motor_odometer_.stall_time() <= 100) {
      dart_rack->trigger_motor_speed_pid_.Update(4000.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
      dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
    } else {
      dart_rack->trigger_motor_->SetCurrent(0);
    }
  } else {
    dart_rack->trigger_motor_speed_pid_.Update(.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
    dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
  }
  // 上膛调节
  glb_load_l_linear_ticks = dart_rack->load_motor_l_odometer_.linear_ticks();
  glb_load_r_linear_ticks = dart_rack->load_motor_r_odometer_.linear_ticks();
  if (dart_rack->rc_->left_y() > 330) {
    if (dart_rack->load_motor_l_odometer_.stall_time() <= 100 &&
        dart_rack->load_motor_r_odometer_.stall_time() <= 100) {
      dart_rack->load_motor_l_speed_pid_.Update(3000.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
      dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
      dart_rack->load_motor_r_speed_pid_.Update(-3000.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
      dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
    } else {
      dart_rack->load_motor_l_speed_pid_.Update(.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
      dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
      dart_rack->load_motor_r_speed_pid_.Update(.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
      dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
    }
  } else if (dart_rack->rc_->left_y() < -330) {
    if (dart_rack->load_motor_l_odometer_.stall_time() <= 100 &&
        dart_rack->load_motor_r_odometer_.stall_time() <= 100) {
      dart_rack->load_motor_l_speed_pid_.Update(-3000.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
      dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
      dart_rack->load_motor_r_speed_pid_.Update(3000.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
      dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
    } else {
      dart_rack->load_motor_l_speed_pid_.Update(.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
      dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
      dart_rack->load_motor_r_speed_pid_.Update(.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
      dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
    }
  } else {
    dart_rack->load_motor_l_speed_pid_.Update(0.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
    dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
    dart_rack->load_motor_r_speed_pid_.Update(0.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
    dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
  }
  // 扳机触发调节
  if (dart_rack->rc_->left_x() > 330) {
    glb_trigger_motor_force_linear_ticks = dart_rack->trigger_motor_force_odometer_.linear_ticks();
    if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 100) {
      if (dart_rack->trigger_motor_force_->encoder() <= 8000) {
        g_fire_running_time++;
        dart_rack->trigger_motor_force_pid_.Update(3000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
        dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
      } else {
        dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
        dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
      }
    } else {
      dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
    }
  } else if (dart_rack->rc_->left_x() < -330) {
    if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 100) {
      if (dart_rack->trigger_motor_force_->encoder() >= 5000) {
        dart_rack->trigger_motor_force_pid_.Update(-3000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
        dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
      } else {
        dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
        dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
      }
    } else {
      dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
    }
  } else {
    dart_rack->trigger_motor_force_->SetCurrent(0);
  }
}

void DartStateAddAdjustUpdate() {
  // M2006 加弹电机 (ID: 6) 摇杆 Y 轴控制逻辑
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
  if (std::abs(dart_rack->rc_->right_y()) > 50) {
    // 死区防止误触
    float add_target_speed = 2000.0f;
    if (dart_rack->rc_->right_y() > 330) {
      // 处于运行区间时给速度，超出区间时设为0
      if (dart_rack->add_motor_odometer_.linear_ticks() < 1350000) {
        dart_rack->add_motor_speed_pid_.Update(add_target_speed, dart_rack->add_motor_->rpm(), 1.0f);
        dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
      } else {
        dart_rack->add_motor_speed_pid_.Clear();
        dart_rack->add_motor_->SetCurrent(0);
      }
    } else if (dart_rack->rc_->right_y() < -330) {
      // 处于运行区间时给速度，超出区间时设为0
      dart_rack->add_motor_speed_pid_.Update(-add_target_speed, dart_rack->add_motor_->rpm(), 1.0f);
      dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
    } else {
      dart_rack->add_motor_speed_pid_.Clear();
      dart_rack->add_motor_->SetCurrent(0);
    }
  } else {
    dart_rack->add_motor_speed_pid_.Clear();
    dart_rack->add_motor_->SetCurrent(0);
  }

  static uint32_t tick = 0;
  tick++;

  // 1. 监测到达妙还未使能(例如未接收反馈、上电重启、或报错保护), 尝试重新使能并清错
  if (dart_rack->dm_motor_->status() != 1) {  // 状态1即为 kEnable
    if (tick % 50 == 0) {                     // 防止CAN总线洪泛
      dart_rack->dm_motor_->SendInstruction(rm::device::DmMotorInstructions::kClearError);
      dart_rack->dm_motor_->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    }
  }

  // 2. 进入本模式时，舵机目标只初始化一次（上电后首次调用时执行，分两帧错开发送）
  static uint8_t servo_init_state = 0;  // 0=未开始, 1=servo1已发送, 2=完成
  if (servo_init_state == 0) {
    glb_servo_1_target = DartRack::kServo1Init + 435.927f;
    glb_servo_2_target = DartRack::kServo2Init + 160.781f;
    dart_rack->add_servo_1_->MoveTime(static_cast<uint16_t>(glb_servo_1_target), 0);
    servo_init_state = 1;
  } else if (servo_init_state == 1) {
    dart_rack->add_servo_2_->MoveTime(static_cast<uint16_t>(glb_servo_2_target), 0);
    servo_init_state = 2;
  }

  // 3. 为防使能指令和MIT控制同一周期并发造成CAN邮箱覆盖发送丢失，错开判断发送
  if (tick % 50 != 0 || dart_rack->dm_motor_->status() == 1) {
    static float target_angle = 0.0f;
    static rm::modules::TrajectoryLimiter adjust_dm_limiter(8.0f, 15.0f);
    static bool is_dm_first_run = true;

    // 当第一次成功接收到使能反馈时，把电机的当前实际位置作为初始目标，防止上电时猛然回零
    if (is_dm_first_run && dart_rack->dm_motor_->status() == 1) {
      target_angle = dart_rack->dm_motor_->pos();
      adjust_dm_limiter.ResetAt(target_angle);
      is_dm_first_run = false;
    }

    // 将摇杆输入作为目标角度的累加值（即控制速度），而不是绝对值。回中时停止改变目标，保持当前位置。
    if (std::abs(dart_rack->rc_->right_x()) > 50) {  // 带死区抵抗摇杆中位漂移
      target_angle += static_cast<float>(dart_rack->rc_->right_x()) * 0.00001f;

      // 限幅保护
      if (target_angle > 1.0f) target_angle = 1.0f;
      if (target_angle < -1.0f) target_angle = -1.0f;
    }

    adjust_dm_limiter.SetTarget(target_angle);
    float smoothed_angle = adjust_dm_limiter.Update(0.001f);

    dm_smoothed_angle = smoothed_angle;  // 更新全局变量供 FreeMaster 查看
    dm_smoothed_angle_int = static_cast<int32_t>(smoothed_angle * 1000.0f);

    // 更新全局监测变量
    dm_status = dart_rack->dm_motor_->status();
    dm_pos = dart_rack->dm_motor_->pos();
    dm_pos_int = static_cast<int32_t>(dm_pos * 1000.0f);
    dart_rack->dm_motor_->SetMitCommand(smoothed_angle, adjust_dm_limiter.current_velocity(), 0.0f, 25.0f, 1.0f);
  }

  int16_t left_x_val = dart_rack->rc_->left_x();
  if (std::abs(left_x_val) > 50) {
    // 累加摇杆值。可修改0.005f调整舵机转动速度
    glb_servo_1_target += static_cast<float>(left_x_val) * 0.01f;
    if (glb_servo_1_target > 1000.0f) glb_servo_1_target = 1000.0f;
    if (glb_servo_1_target < 0.0f) glb_servo_1_target = 0.0f;
  }

  int16_t left_y_val = dart_rack->rc_->left_y();
  if (std::abs(left_y_val) > 50) {
    // 累加摇杆值。可修改0.005f调整舵机转动速度
    glb_servo_2_target += static_cast<float>(left_y_val) * 0.01f;
    // 限制范围 100 到 500
    if (glb_servo_2_target > 1000.0f) glb_servo_2_target = 1000.0f;
    if (glb_servo_2_target < 0.0f) glb_servo_2_target = 0.0f;
  }

  // 每20ms发送一次舵机指令(~50Hz)，两个舵机错开10ms避免共用串口冲突
  if (tick % 20 == 0 && tick % 50 != 0) {
    dart_rack->add_servo_1_->MoveTime(static_cast<uint16_t>(glb_servo_1_target), 0);
  } else if (tick % 20 == 10 && tick % 50 != 10) {
    dart_rack->add_servo_2_->MoveTime(static_cast<uint16_t>(glb_servo_2_target), 0);
  }

  // 更新拨弹电机的全局位置监视，方便FreeMaster查看
  // M2006 电机转子有 0-8191的绝对编码，这里直接读取原始反馈编码（注意由于36:1减速比，输出轴转一圈会经历36次0-8191）
  glb_add_motor_linear = dart_rack->add_motor_odometer_.linear_ticks();
}