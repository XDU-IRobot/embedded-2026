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
extern volatile uint8_t glb_game_status;                // 比赛阶段: 1=准备, 4=进行中
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
// aim阶段yaw调试变量
volatile float glb_aim_pixel_error = 0.0f;      // 像素误差
volatile float glb_aim_current_pixel = 0.0f;    // 当前视觉像素
volatile float glb_aim_target_pixel = 0.0f;     // 目标像素
volatile uint8_t glb_aim_yaw_done = 0;          // yaw到位标志
volatile uint8_t glb_aim_yaw_fail = 0;          // yaw逼近失败标志
volatile float glb_aim_yaw_deg = 0.0f;          // yaw编码器角度
volatile int32_t glb_aim_yaw_target_speed = 0;  // yaw目标速度

constexpr int32_t kTriggerDeadZone = 10000;  // trigger 到达死区(ticks)

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

  // 根据遥控器左拨杆位置设置状态
  // 为了检测状态切换，由于 state 每次都被重写，不方便检测边界，我们通过在其他模式中清理 manual_mode
  // 的状态来实现切回后重新初始化
  if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kDown &&
      dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kUp) {
    // 左拨杆向下，右拨杆向上，showtime模式
    state.unable = AbleState::kOff;
    if (state.showtime_mode.enabled != AbleState::kOn) {
      DartShowtimeModeClear(state.showtime_mode);
    }
    state.showtime_mode.enabled = AbleState::kOn;
    state.manual_mode.enabled = AbleState::kOff;
    state.lvgl_mode.enabled = AbleState::kOff;
    state.add_adjust_mode.enabled = AbleState::kOff;
    state.adjust_mode.enabled = AbleState::kOff;
  } else if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kDown) {
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
  } else if (state.showtime_mode.enabled == AbleState::kOn) {
    DartStateShowtimeUpdate();
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
      // 初始化逻辑
      if (dart_rack->state_.manual_mode.init == PhaseState::kUncomplete) {
        DartStateInitUpdate();
      } else if (dart_rack->state_.manual_mode.init == PhaseState::kDone) {
        // 初始化完成，进入下一个阶段
        if (dart_rack->rc_->right_x() == 660) {
          dart_rack->state_.manual_mode.mode = ModeState::kload;
        }
      }
      break;
    case ModeState::kload:
      if (dart_rack->state_.manual_mode.load == PhaseState::kUncomplete) {
        DartStateLoadUpdate();  //... 装填操作
        // dart_rack->state_.manual_mode.mode = ModeState::kAdd;
      } else if (dart_rack->state_.manual_mode.load == PhaseState::kDone) {
        dart_rack->state_.manual_mode.mode = ModeState::kAdd;
      }
      // 装填逻辑
      break;

    case ModeState::kAdd:
      if (dart_rack->state_.manual_mode.add == PhaseState::kUncomplete) {
        if (dart_rack->dart_count_ == DartCount::kFirst) {
          dart_rack->state_.manual_mode.add = PhaseState::kDone;
        } else if (dart_rack->dart_count_ == DartCount::kSecond) {
          DartStateAddPlaceOnly();
        } else {
          DartStateAddUpdate();
        }
      } else if (dart_rack->state_.manual_mode.add == PhaseState::kDone) {
        dart_rack->state_.manual_mode.mode = ModeState::kAim;
      }
      break;

    case ModeState::kAim:
      if (dart_rack->state_.manual_mode.aim == PhaseState::kUncomplete) {
        DartStateAimUpdate();
        // dart_rack->state_.manual_mode.mode = ModeState::kFire;
      } else if (dart_rack->state_.manual_mode.aim == PhaseState::kDone) {
        // 瞄准完成，进入下一个阶段
        dart_rack->state_.manual_mode.mode = ModeState::kFire;
      }
      break;
    case ModeState::kFire: {
      // 发射逻辑
      if (dart_rack->state_.manual_mode.fire == PhaseState::kUncomplete && dart_rack->rc_->left_x() == 660 &&
          dart_rack->rc_->right_x() == -660) {
        DartStateFireUpdate();
      } else if (dart_rack->state_.manual_mode.fire == PhaseState::kDone) {
        // 发射完成，始终回到第一发
        // dart_rack->dart_count_ = DartCount::kFirst;
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

// Showtime模式状态机处理
void DartStateShowtimeUpdate() {
  switch (dart_rack->state_.showtime_mode.mode) {
    case ModeState::kUnable:
      // 第一发：比赛准备阶段才进入初始化
      // 后续发：舱门开启就进入初始化
      if (dart_rack->dart_count_ == DartCount::kFirst) {
        if (glb_game_status == 1) {
          dart_rack->state_.showtime_mode.mode = ModeState::kInit;
        }
      } else {
        if (glb_dart_launch_opening_status == 0) {
          dart_rack->state_.showtime_mode.mode = ModeState::kInit;
        }
      }
      break;

    case ModeState::kInit:
      if (dart_rack->state_.showtime_mode.init == PhaseState::kUncomplete) {
        DartStateInitUpdate();
      } else if (dart_rack->state_.showtime_mode.init == PhaseState::kDone) {
        // 初始化完成，舱门打开后进入load
        if (glb_dart_launch_opening_status == 2 || glb_dart_launch_opening_status == 0) {
          dart_rack->state_.showtime_mode.mode = ModeState::kload;
        }
      }
      break;

    case ModeState::kload:
      if (dart_rack->state_.showtime_mode.load == PhaseState::kUncomplete) {
        DartStateLoadUpdate();
      } else if (dart_rack->state_.showtime_mode.load == PhaseState::kDone) {
        dart_rack->state_.showtime_mode.mode = ModeState::kAdd;
      }
      break;

    case ModeState::kAdd:
      if (dart_rack->state_.showtime_mode.add == PhaseState::kUncomplete) {
        if (dart_rack->dart_count_ == DartCount::kFirst) {
          dart_rack->state_.showtime_mode.add = PhaseState::kDone;
        } else if (dart_rack->dart_count_ == DartCount::kSecond) {
          DartStateAddPlaceOnly();
        } else {
          DartStateAddUpdate();
        }
      } else if (dart_rack->state_.showtime_mode.add == PhaseState::kDone) {
        dart_rack->state_.showtime_mode.mode = ModeState::kAim;
      }
      break;

    case ModeState::kAim:
      if (dart_rack->state_.showtime_mode.aim == PhaseState::kUncomplete) {
        DartStateAimUpdate();
      } else if (dart_rack->state_.showtime_mode.aim == PhaseState::kDone) {
        dart_rack->state_.showtime_mode.mode = ModeState::kFire;
      }
      break;

    case ModeState::kFire:
      if (dart_rack->state_.showtime_mode.fire == PhaseState::kUncomplete) {
        // 舱门已开启 → 直接发射
        if (glb_dart_launch_opening_status == 0) {
          DartStateFireUpdate();
        }
      } else if (dart_rack->state_.showtime_mode.fire == PhaseState::kDone) {
        // 发射完成，进入下一发
        uint8_t next_dart = static_cast<uint8_t>(dart_rack->dart_count_) + 1;
        if (next_dart > 3) {
          dart_rack->dart_count_ = DartCount::kFirst;
          g_all_darts_completed = true;
        } else {
          dart_rack->dart_count_ = static_cast<DartCount>(next_dart);
        }
        DartShowtimeModeClear(dart_rack->state_.showtime_mode);
        dart_rack->state_.showtime_mode.enabled = AbleState::kOn;
        dart_rack->state_.showtime_mode.mode = ModeState::kInit;
      }
      break;

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
  // 滑台下拉
  static uint32_t load_down_run_time = 0;
  load_down_run_time++;
  if (dart_rack->load_motor_l_odometer_.stall_time() <= 100 && dart_rack->load_motor_r_odometer_.stall_time() <= 100 &&
      dart_rack->state_.manual_mode.is_load_down_done == false && load_down_run_time < 3000) {
    if (dart_rack->load_motor_r_odometer_.linear_ticks() > DartRack::kTriggerEcdMax ||
        dart_rack->load_motor_l_odometer_.linear_ticks() < -DartRack::kTriggerEcdMax) {
      dart_rack->load_motor_l_speed_pid_.Update(-1000.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
      dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
      dart_rack->load_motor_r_speed_pid_.Update(1000.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
      dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
    } else {
      dart_rack->load_motor_l_speed_pid_.Update(-4000.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
      dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
      dart_rack->load_motor_r_speed_pid_.Update(4000.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
      dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
    }
  } else {
    dart_rack->load_motor_l_speed_pid_.Update(.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
    dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
    dart_rack->load_motor_r_speed_pid_.Update(.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
    dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
    dart_rack->state_.manual_mode.is_load_down_done = true;
    load_down_run_time = 0;
  }

  // 撒放器锁定
  if (dart_rack->state_.manual_mode.is_load_down_done == true) {
    if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 100 &&
        dart_rack->state_.manual_mode.is_trigger_lock_done == false) {
      dart_rack->trigger_motor_force_pid_.Update(-3000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
    } else {
      dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
      dart_rack->state_.manual_mode.is_trigger_lock_done = true;
    }
  } else {
    // 在滑台下拉阶段同时打开撒放器
    if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 50) {
      dart_rack->trigger_motor_force_pid_.Update(3000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
    } else {
      dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
    }
  }
  if (dart_rack->state_.manual_mode.is_trigger_lock_done == true &&
      dart_rack->state_.manual_mode.is_load_up_done == false) {
    const int32_t l_ticks = dart_rack->load_motor_l_odometer_.linear_ticks();
    const int32_t r_ticks = dart_rack->load_motor_r_odometer_.linear_ticks();
    constexpr int32_t kBrakeZone = 200000;
    constexpr int32_t kDeadZone = 50000;
    dart_rack->trigger_motor_force_pid_.Clear();
    dart_rack->trigger_motor_force_->SetCurrent(0);
    if (l_ticks < -kDeadZone && dart_rack->load_motor_l_odometer_.stall_time() <= 100) {
      float ratio = std::min(1.0f, static_cast<float>(-l_ticks) / static_cast<float>(kBrakeZone));
      float l_speed = 1000.0f + 3000.0f * ratio;
      dart_rack->load_motor_l_speed_pid_.Update(l_speed, dart_rack->load_motor_l_->rpm(), 1.0f);
      dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
    } else {
      dart_rack->load_motor_l_speed_pid_.Clear();
      dart_rack->load_motor_l_->SetCurrent(0);
    }

    if (r_ticks > kDeadZone && dart_rack->load_motor_r_odometer_.stall_time() <= 100) {
      float ratio = std::min(1.0f, static_cast<float>(r_ticks) / static_cast<float>(kBrakeZone));
      float r_speed = -(1000.0f + 3000.0f * ratio);
      dart_rack->load_motor_r_speed_pid_.Update(r_speed, dart_rack->load_motor_r_->rpm(), 1.0f);
      dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
    } else {
      dart_rack->load_motor_r_speed_pid_.Clear();
      dart_rack->load_motor_r_->SetCurrent(0);
    }

    if (l_ticks >= -kDeadZone && r_ticks <= kDeadZone) {
      dart_rack->state_.manual_mode.is_load_up_done = true;
      dart_rack->load_motor_l_->SetCurrent(0);
      dart_rack->load_motor_r_->SetCurrent(0);
      dart_rack->load_motor_l_speed_pid_.Clear();
      dart_rack->load_motor_r_speed_pid_.Clear();
    }
  }
  if (dart_rack->state_.manual_mode.is_trigger_lock_done == true &&
      dart_rack->state_.manual_mode.is_load_up_done == true &&
      dart_rack->state_.manual_mode.is_load_down_done == true) {
    dart_rack->state_.manual_mode.load = PhaseState::kDone;
    dart_rack->load_motor_l_->SetCurrent(0);
    dart_rack->load_motor_r_->SetCurrent(0);
    dart_rack->load_motor_l_speed_pid_.Clear();
    dart_rack->load_motor_r_speed_pid_.Clear();
  }
}

void DartStateAddUpdate() {
  // 继电器控制管脚PF1
  static uint32_t tick = 0;
  static AddState current_state = AddState::MOVING_BACK;
  static bool add_forward_done = false;
  static bool trigger_returned_done = false;
  static rm::modules::TrajectoryLimiter dm_limiter(8.0f, 15.0f);
  static bool dm_limiter_initialized = false;
  static uint32_t state_timer = 0;

  // 重新进入add阶段时重置所有static变量
  if (dart_rack->state_.manual_mode.need_add_state_reset) {
    tick = 0;
    current_state = AddState::MOVING_BACK;
    add_forward_done = false;
    trigger_returned_done = false;
    dm_limiter_initialized = false;
    state_timer = 0;
    dart_rack->state_.manual_mode.need_add_state_reset = false;
  }

  tick++;
  dm_status = dart_rack->dm_motor_->status();
  dm_pos = dart_rack->dm_motor_->pos();
  dm_pos_int = static_cast<int32_t>(dm_pos * 1000.0f);

  // 1. 监测到达妙还未使能(例如未接收反馈、上电重启、或报错保护), 尝试重新使能并清错
  if (dart_rack->dm_motor_->status() != 1) {  // 状态1即为 kEnable
    if (tick % 50 == 0) {                     // 防止CAN总线洪泛
      dart_rack->dm_motor_->SendInstruction(rm::device::DmMotorInstructions::kClearError);
      dart_rack->dm_motor_->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    }
  }
  glb_add_motor_linear = dart_rack->add_motor_odometer_.linear_ticks();
  float target_dm_angle = -0.050f;
  if (!dm_limiter_initialized && dm_status == 1) {
    dm_limiter.ResetAt(dm_pos);
    dm_limiter_initialized = true;
  }
  constexpr float S1 = DartRack::kServo1Init;
  constexpr float S2 = DartRack::kServo2Init;
  float target_servo1 = 500.0f;
  float target_servo2 = 500.0f;

  switch (current_state) {
    case AddState::MOVING_BACK:
      target_dm_angle = -0.050f;
      target_servo1 = S1 + 435.927f;
      target_servo2 = S2 + 160.781f;
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
      if (dart_rack->dart_count_ == DartCount::kThird || dart_rack->dart_count_ == DartCount::kFourth) {
        if (!g_add_limit_suppressed) {
          if (dart_rack->add_motor_odometer_.linear_ticks() > 0) {
            float back_speed = (dart_rack->add_motor_odometer_.linear_ticks() < 50000) ? -3000.0f : -6000.0f;
            dart_rack->add_motor_speed_pid_.Update(back_speed, dart_rack->add_motor_->rpm(), 1.0f);
            dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
          } else {
            dart_rack->add_motor_->SetCurrent(0);
            dart_rack->add_motor_speed_pid_.Clear();
            state_timer = 0;
            current_state = AddState::SUSPENDED_init;
          }
          break;
        }
      }

    case AddState::SUSPENDED_init:
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
      target_dm_angle = (dart_rack->dart_count_ == DartCount::kThird) ? -0.546f : 0.430f;
      target_servo1 = S1 + 435.927f;
      target_servo2 = S2 + 160.781f;
      dart_rack->add_motor_->SetCurrent(0);
      state_timer++;
      if (state_timer > 300) {
        state_timer = 0;
        current_state = AddState::CAUGHT;
      }
      break;

    case AddState::CAUGHT:
      target_dm_angle = (dart_rack->dart_count_ == DartCount::kThird) ? -0.546f : 0.430f;
      target_servo1 = S1 + 529.0f;
      target_servo2 = S2 + 159.0f;
      dart_rack->add_motor_->SetCurrent(0);
      state_timer++;
      if (state_timer > 300) {
        state_timer = 0;
        current_state = AddState::MOVING_FORWARD;
      }
      break;

    case AddState::MOVING_FORWARD:
      target_servo1 = S1 + 391.337f;
      target_servo2 = S2 + 50.111f;
      if (!add_forward_done) {
        if (dart_rack->add_motor_odometer_.linear_ticks() < 2000000) {
          dart_rack->add_motor_speed_pid_.Update(6000.0f, dart_rack->add_motor_->rpm(), 1.0f);
          target_dm_angle = -0.050f;
          dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
        } else {
          dart_rack->add_motor_->SetCurrent(0);
          dart_rack->add_motor_speed_pid_.Clear();
          target_dm_angle = -0.050f;
          add_forward_done = true;
        }
      } else {
        dart_rack->add_motor_->SetCurrent(0);
      }
      if (!trigger_returned_done) {
        if (g_trigger_motor_limit_suppressed) {
          dart_rack->trigger_motor_speed_pid_.Clear();
          dart_rack->trigger_motor_->SetCurrent(0);
          trigger_returned_done = true;
        } else {
          float trigger_speed =
              (dart_rack->trigger_motor_odometer_.linear_ticks() < -kTriggerDeadZone) ? 4000.0f : 2000.0f;
          dart_rack->trigger_motor_speed_pid_.Update(trigger_speed, dart_rack->trigger_motor_->rpm(), 1.0f);
          dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
        }
      } else {
        dart_rack->trigger_motor_->SetCurrent(0);
      }
      if (add_forward_done && trigger_returned_done) {
        add_forward_done = false;
        trigger_returned_done = false;
        state_timer = 0;
        current_state = AddState::SUSPENDED;
      }
      break;

    case AddState::SUSPENDED:
      target_dm_angle = -0.050f;
      target_servo1 = S1 + 741.337f;
      target_servo2 = S2 + 386.671f;
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
      target_dm_angle = (dart_rack->dart_count_ == DartCount::kThird) ? -0.038f : -0.000f;
      target_servo1 = S1 + 874.0f;
      target_servo2 = S2 + 500.0f;
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
          bool pos_reached = (std::abs(pos1 - static_cast<int16_t>(target_servo1)) < 20 &&
                              std::abs(pos2 - static_cast<int16_t>(target_servo2)) < 20);
          bool pos_stopped = (std::abs(pos1 - prev_pos1) < 20 && std::abs(pos2 - prev_pos2) < 20);
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
        if (gpio_on_cnt > 100) {
          HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
           if (gpio_on_cnt > 150) {
            target_servo1 = S1 + 435.927f;
            target_servo2 = S2 + 160.781f;
            if (dart_rack->trigger_motor_odometer_.linear_ticks() > -50000 &&
                dart_rack->trigger_motor_odometer_.stall_time() <= 100) {
              dart_rack->trigger_motor_speed_pid_.Update(-2000.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
              dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
            } else {
              dart_rack->trigger_motor_->SetCurrent(0);
              dart_rack->trigger_motor_speed_pid_.Clear();
            }
          }
        }
        if (gpio_on_cnt > 500) {
          HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
          gpio_on_cnt = 0;
          gpio_triggered = false;
          tick = 0;
          add_forward_done = false;
          trigger_returned_done = false;
          dart_rack->state_.manual_mode.add = PhaseState::kDone;
          current_state = AddState::MOVING_BACK;
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

  static AddState current_state = AddState::MOVING_FORWARD;
  static bool add_forward_done = false;
  static bool trigger_returned_done = false;
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
    case AddState::MOVING_FORWARD:
      target_dm_angle = -0.050f;
      target_servo1 = S1 + 391.337f;
      target_servo2 = S2 + 50.111f;
      if (!add_forward_done) {
        if (dart_rack->add_motor_odometer_.linear_ticks() < 2000000) {
          dart_rack->add_motor_speed_pid_.Update(6000.0f, dart_rack->add_motor_->rpm(), 1.0f);
          dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
        } else {
          dart_rack->add_motor_->SetCurrent(0);
          dart_rack->add_motor_speed_pid_.Clear();
          add_forward_done = true;
        }
      } else {
        dart_rack->add_motor_->SetCurrent(0);
      }
      if (!trigger_returned_done) {
        if (g_trigger_motor_limit_suppressed) {
          dart_rack->trigger_motor_speed_pid_.Clear();
          dart_rack->trigger_motor_->SetCurrent(0);
          trigger_returned_done = true;
        } else {
          float trigger_speed =
              (dart_rack->trigger_motor_odometer_.linear_ticks() < -kTriggerDeadZone) ? 4000.0f : 2000.0f;
          dart_rack->trigger_motor_speed_pid_.Update(trigger_speed, dart_rack->trigger_motor_->rpm(), 1.0f);
          dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
        }
      } else {
        dart_rack->trigger_motor_->SetCurrent(0);
      }
      if (add_forward_done && trigger_returned_done) {
        add_forward_done = false;
        trigger_returned_done = false;
        state_timer = 0;
        current_state = AddState::SUSPENDED;
      }
      break;

    case AddState::SUSPENDED:
      target_dm_angle = -0.050f;
      target_servo1 = S1 + 741.337f;
      target_servo2 = S2 + 386.671f;
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
      target_dm_angle = (dart_rack->dart_count_ == DartCount::kThird) ? -0.050f : -0.020f;
      target_servo1 = S1 + 874.0f;
      target_servo2 = S2 + 500.0f;
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
          bool pos_reached = (std::abs(pos1 - static_cast<int16_t>(target_servo1)) < 20 &&
                              std::abs(pos2 - static_cast<int16_t>(target_servo2)) < 20);
          bool pos_stopped = (std::abs(pos1 - prev_pos1) < 20 && std::abs(pos2 - prev_pos2) < 20);
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
        if (gpio_on_cnt > 100) {
          HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
          if (gpio_on_cnt > 150) {
            target_servo1 = S1 + 435.927f;
            target_servo2 = S2 + 160.781f;
            if (dart_rack->trigger_motor_odometer_.linear_ticks() > -50000 &&
                dart_rack->trigger_motor_odometer_.stall_time() <= 100) {
              dart_rack->trigger_motor_speed_pid_.Update(-2000.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
              dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
            } else {
              dart_rack->trigger_motor_->SetCurrent(0);
              dart_rack->trigger_motor_speed_pid_.Clear();
            }
          }
        }
        if (gpio_on_cnt > 500) {
          HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
          gpio_on_cnt = 0;
          gpio_triggered = false;
          tick = 0;
          add_forward_done = false;
          trigger_returned_done = false;
          dart_rack->state_.manual_mode.add = PhaseState::kDone;
          current_state = AddState::MOVING_FORWARD;
          return;
        }
      }
      break;
    }
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
  // ============ 可调参数接口 ============
  constexpr float kYawPixelTolerance = 1.0f;       // 像素允许误差
  constexpr float kYawSpeedLimit = 4000.0f;        // 大误差时的最大速度
  constexpr float kYawSlowSpeed = 1500.0f;         // 小误差时的速度（误差<15像素）
  constexpr int32_t kTriggerBaseTarget = -2000000;  // trigger 基准目标(ticks)
  constexpr float kTriggerRunSpeed = -4000.0f;     // trigger 运行速度
  constexpr int kApproachFailThreshold = 40;       // 连续未减小次数阈值，可调整

  // ---- 每发镖偏置 (按 dart_count 索引) ----
  // Yaw[idx] 即为目标像素值（绝对像素坐标）

  static bool is_first_run = true;
  static bool trigger_reached = false;
  static float target_pixel = 0.0f;

  // 新增：逼近判断相关静态变量
  static float prev_abs_error = -1.0f;
  static int consecutive_no_decrease = 0;
  static bool yaw_approach_failed = false;

  trigger_motor_linear = dart_rack->trigger_motor_odometer_.linear_ticks();
  uint8_t idx = static_cast<uint8_t>(dart_rack->dart_count_);

  if (dart_rack->vision_data_ == nullptr) {
    dart_rack->state_.manual_mode.aim = PhaseState::kDone;
    return;
  }

  if (is_first_run) {
    is_first_run = false;
    trigger_reached = false;
    target_pixel = Yaw[idx];
    // 重置所有逼近相关状态
    prev_abs_error = -1.0f;
    consecutive_no_decrease = 0;
    yaw_approach_failed = false;
  }

  float current_pixel = dart_rack->vision_data_->Yaw;
  float pixel_error = target_pixel - current_pixel;
  float abs_error = std::abs(pixel_error);
  bool yaw_done = abs_error <= kYawPixelTolerance;

  // 调试变量更新
  glb_aim_current_pixel = current_pixel;
  glb_aim_target_pixel = target_pixel;
  glb_aim_pixel_error = pixel_error;
  glb_aim_yaw_done = yaw_done ? 1 : 0;
  glb_aim_yaw_fail = yaw_approach_failed ? 1 : 0;

  // ---- Yaw 轴逼近失败判断（仅当未到达且未失败时）----
  if (!yaw_done && !yaw_approach_failed) {
    if (prev_abs_error >= 0.0f) {  // 不是第一次获取误差
      if (abs_error >= prev_abs_error) {
        // 误差没有减小（增大或相等）
        consecutive_no_decrease++;
        if (consecutive_no_decrease >= kApproachFailThreshold) {
          yaw_approach_failed = true;
          // 立即停止 yaw 电机
          dart_rack->yaw_motor_speed_pid_.Clear();
          dart_rack->yaw_motor_->SetCurrent(0);
        }
      } else {
        // 误差在减小，重置计数器
        consecutive_no_decrease = 0;
      }
    }
    prev_abs_error = abs_error;
  }

  // ---- Yaw 轴控制（仅在未到达且未逼近失败时执行）----
  if (!yaw_done && !yaw_approach_failed) {
    // 根据误差绝对值选择速度大小
    float speed_magnitude = (abs_error >= 15.0f) ? kYawSpeedLimit : kYawSlowSpeed;
    float target_speed = (pixel_error > 0) ? speed_magnitude : -speed_magnitude;

    // 限位保护：检查编码器角度是否接近机械限位
    float yaw_deg = dart_rack->yaw_encoder_->angle_deg();
    glb_aim_yaw_deg = yaw_deg;
    glb_aim_yaw_target_speed = static_cast<int32_t>(target_speed);
    if ((target_speed > 0 && yaw_deg >= DartRack::kYawEcdMax - 0.5f) ||
        (target_speed < 0 && yaw_deg <= DartRack::kYawEcdMin + 0.5f)) {
      dart_rack->yaw_motor_speed_pid_.Clear();
      dart_rack->yaw_motor_->SetCurrent(0);
    } else {
      dart_rack->yaw_motor_speed_pid_.Update(target_speed, dart_rack->yaw_motor_->rpm(), 1.0f);
      glb_aim_yaw_pid_out = static_cast<int32_t>(dart_rack->yaw_motor_speed_pid_.out());
      dart_rack->yaw_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->yaw_motor_speed_pid_.out()));
    }
  } else {
    // 已到达目标或已逼近失败：确保 yaw 电机停止
    dart_rack->yaw_motor_speed_pid_.Clear();
    dart_rack->yaw_motor_->SetCurrent(0);
  }

  // ---- Trigger 轴控制（保持不变）----
  int32_t trigger_target = kTriggerBaseTarget + static_cast<int32_t>(Pitch[idx]);
  int32_t trigger_current_ticks = dart_rack->trigger_motor_odometer_.linear_ticks();
  int32_t trigger_remain = trigger_target - trigger_current_ticks;

  if (!trigger_reached) {
    if (std::abs(trigger_remain) > kTriggerDeadZone) {
      dart_rack->trigger_motor_speed_pid_.Update(kTriggerRunSpeed, dart_rack->trigger_motor_->rpm(), 1.0f);
      dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
    } else {
      dart_rack->trigger_motor_speed_pid_.Clear();
      dart_rack->trigger_motor_->SetCurrent(0);
      trigger_reached = true;
    }
  }

  // ---- 瞄准完成条件（yaw 成功到达 或 逼近失败）且 trigger 已到位 ----
  if ((yaw_done || yaw_approach_failed) && trigger_reached) {
    dart_rack->yaw_motor_speed_pid_.Clear();
    dart_rack->yaw_motor_->SetCurrent(0);
    dart_rack->trigger_motor_speed_pid_.Clear();
    dart_rack->trigger_motor_->SetCurrent(0);
    dart_rack->state_.manual_mode.aim = PhaseState::kDone;
    is_first_run = true;
    trigger_reached = false;
    // 重置逼近相关状态，为下一次瞄准做准备
    yaw_approach_failed = false;
    prev_abs_error = -1.0f;
    consecutive_no_decrease = 0;
  }
}

void DartStateFireUpdate() {
  static uint32_t fire_running_time = 0;
  static bool fire_first_entry = true;
  if (fire_first_entry) {
    dart_rack->trigger_motor_force_odometer_.Reset();
    fire_first_entry = false;
  }
  trigger_motor_force_odometer_time = dart_rack->trigger_motor_force_odometer_.stall_time();
  trigger_motor_force_running_time = fire_running_time;
  if (dart_rack->state_.manual_mode.fire == PhaseState::kUncomplete) {
    if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 100 && fire_running_time < 200) {
      dart_rack->trigger_motor_force_pid_.Update(4000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
      fire_running_time++;
    } else {
      dart_rack->trigger_motor_force_->SetCurrent(0);
      dart_rack->state_.manual_mode.fire = PhaseState::kDone;
      fire_running_time = 0;
      fire_first_entry = true;
    }
  } else {
    dart_rack->trigger_motor_force_->SetCurrent(0);
    dart_rack->state_.manual_mode.fire = PhaseState::kDone;
    fire_running_time = 0;
    fire_first_entry = true;
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
  if (std::abs(dart_rack->rc_->right_y()) > 50) {
    // 死区防止误触
    float add_target_speed = 2000.0f;
    if (dart_rack->rc_->right_y() > 330) {
      // 处于运行区间时给速度，超出区间时设为0
      if (dart_rack->add_motor_odometer_.linear_ticks() < 2000000) {
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