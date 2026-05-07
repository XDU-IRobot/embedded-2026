#include "dart_statemachine.hpp"
#include "lcd_init.h"
#include "lvgl.h"
#include <cmath>
#include <cstdio>  // Added for printf
#include "sd_card.h"
#include "librm/device/actuator/dm_motor.hpp"
extern bool is_lvgl_running;
extern volatile uint8_t g_add_limit_ever_hit;
extern volatile uint8_t g_trigger_limit_ever_hit;
extern volatile uint8_t g_add_limit_suppressed;
extern volatile uint8_t g_add_motor_limit_triggered;
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
  if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kDown) {
    // 无力状态
    state.unable = AbleState::kOn;
    state.manual_mode.enabled = AbleState::kOff;
    state.lvgl_mode.enabled = AbleState::kOff;
    state.add_adjust_mode.enabled = AbleState::kOff;
    state.adjust_mode.enabled = AbleState::kOff;
  } else if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kMid &&
             dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kUp) {
    // 左拨杆向中，自动打镖模式 (只初始化yaw轴)
    state.unable = AbleState::kOff;
    state.manual_mode.enabled = AbleState::kOn;
    state.lvgl_mode.enabled = AbleState::kOff;
    state.add_adjust_mode.enabled = AbleState::kOff;
    state.adjust_mode.enabled = AbleState::kOff;
  } else if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kUp &&
             dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kUp) {
    // 左右拨杆向上，自动模式(开启LVGL) (局间调节)
    state.unable = AbleState::kOff;
    state.manual_mode.enabled = AbleState::kOff;
    if (state.lvgl_mode.enabled != AbleState::kOn) {
      DartManualModeClear(state.manual_mode);
      LvglHomingStart();
    }
    state.lvgl_mode.enabled = AbleState::kOn;
    state.add_adjust_mode.enabled = AbleState::kOff;
    state.adjust_mode.enabled = AbleState::kOff;
  } else if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kUp &&
             dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kMid) {
    // 左拨杆向上，右拨杆向中，调节除换弹之外的所有电机
    state.unable = AbleState::kOff;
    state.manual_mode.enabled = AbleState::kOff;
    state.lvgl_mode.enabled = AbleState::kOff;
    state.add_adjust_mode.enabled = AbleState::kOff;
    state.adjust_mode.enabled = AbleState::kOn;
  } else if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kMid &&
             dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kMid) {
    // 左右拨杆均在中间，换弹装置手动调节模式
    state.unable = AbleState::kOff;
    state.manual_mode.enabled = AbleState::kOff;
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
    if (!LvglHomingUpdate()) {
      is_lvgl_running = false;
    } else {
      is_lvgl_running = true;
      dart_rack->load_motor_l_->SetCurrent(0);
      dart_rack->load_motor_r_->SetCurrent(0);
      dart_rack->trigger_motor_->SetCurrent(0);
      dart_rack->trigger_motor_force_->SetCurrent(0);
      dart_rack->add_motor_->SetCurrent(0);
      dart_rack->yaw_motor_->SetCurrent(0);
      dart_rack->add_servo_->SetServoAngle(static_cast<uint16_t>(DartRack::kServo1Init + 474.886f), 1, 0);
      dart_rack->add_servo_->SetServoAngle(static_cast<uint16_t>(DartRack::kServo2Init + 190.831f), 2, 0);
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
      } else if (dart_rack->state_.manual_mode.aim == PhaseState::kDone) {
        // 瞄准完成，进入下一个阶段
        dart_rack->state_.manual_mode.mode = ModeState::kFire;
      }
      break;
    case ModeState::kFire:
      // 发射逻辑
      if (dart_rack->state_.manual_mode.fire == PhaseState::kUncomplete && dart_rack->rc_->left_x() == 660 &&
          dart_rack->rc_->right_x() == -660) {
        DartStateFireUpdate();
      } else if (dart_rack->state_.manual_mode.fire == PhaseState::kDone) {
        // 发射完成，返回待机状态
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

  // 只刷新一次，固定显示内容防止持续刷新
  static bool lcd_displayed = false;
  if (!lcd_displayed) {
    LCD_DISPLAY();
    lcd_displayed = true;
  }
}

void DartStateInitUpdate() {
  dart_rack->add_servo_->SetServoAngle(static_cast<uint16_t>(DartRack::kServo1Init + 474.886f), 1, 0);
  dart_rack->add_servo_->SetServoAngle(static_cast<uint16_t>(DartRack::kServo2Init + 190.831f), 2, 0);

  // Yaw轴根据是第几发镖初始化
  if (!dart_rack->state_.manual_mode.is_yaw_init_done) {
    static float last_yaw_error = 0.0f;
    static float last_yaw_angle = 0.0f;
    static uint32_t stall_count = 0;
    static bool is_first_run = true;

    float yaw_target = Yaw[static_cast<uint8_t>(dart_rack->dart_count_)];
    float yaw_current = dart_rack->yaw_encoder_->angle_deg();
    float yaw_error = yaw_target - yaw_current;

    if (is_first_run) {
      last_yaw_error = yaw_error;
      last_yaw_angle = yaw_current;
      stall_count = 0;
      is_first_run = false;
    }

    if (std::abs(yaw_error) < 1.0f && std::abs(yaw_current - last_yaw_angle) < 0.01f) {
      stall_count++;
    } else {
      stall_count = 0;
    }

    if (std::abs(yaw_error) <= 0.05f || (last_yaw_error * yaw_error < 0.0f) || stall_count > 100) {
      dart_rack->yaw_motor_speed_pid_.Clear();
      dart_rack->yaw_motor_->SetCurrent(0);
      dart_rack->state_.manual_mode.is_yaw_init_done = true;
      is_first_run = true;
    } else {
      float target_speed = yaw_error * 1000.0f;
      if (target_speed > 1000.0f)
        target_speed = 1000.0f;
      else if (target_speed < -1000.0f)
        target_speed = -1000.0f;

      dart_rack->yaw_motor_speed_pid_.Update(target_speed, dart_rack->yaw_motor_->rpm(), 1.0f);
      dart_rack->yaw_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->yaw_motor_speed_pid_.out()));
    }
    last_yaw_error = yaw_error;
    last_yaw_angle = yaw_current;
  }

  // load/trigger/add 已在 LVGL 阶段完成归位，直接标记完成
  dart_rack->state_.manual_mode.is_load_reset_done = true;
  dart_rack->state_.manual_mode.is_trigger_reset_done = true;
  dart_rack->state_.manual_mode.is_add_init_done = true;
  dart_rack->state_.manual_mode.is_trigger_force_init_done = true;

  // 全部检查完成后，初始化完成
  if (dart_rack->state_.manual_mode.is_yaw_init_done && dart_rack->state_.manual_mode.is_load_reset_done &&
      dart_rack->state_.manual_mode.is_trigger_reset_done && dart_rack->state_.manual_mode.is_trigger_force_init_done &&
      dart_rack->state_.manual_mode.is_add_init_done) {
    dart_rack->state_.manual_mode.init = PhaseState::kDone;
    dart_rack->yaw_motor_speed_pid_.Clear();
    dart_rack->yaw_motor_->SetCurrent(0);
  }
}

void DartStateLoadUpdate() {
  // 上膛逻辑
  // 滑台下拉
  if (dart_rack->load_motor_l_odometer_.stall_time() <= 100 && dart_rack->load_motor_r_odometer_.stall_time() <= 100 &&
      dart_rack->state_.manual_mode.is_load_down_done == false) {
    if (dart_rack->load_motor_r_odometer_.linear_ticks() > DartRack::kTriggerEcdMax ||
        dart_rack->load_motor_l_odometer_.linear_ticks() < -DartRack::kTriggerEcdMax) {
      dart_rack->load_motor_l_speed_pid_.Update(-1500.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
      dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
      dart_rack->load_motor_r_speed_pid_.Update(1500.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
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
  }

  // 撒放器锁定
  if (dart_rack->state_.manual_mode.is_load_down_done == true) {
    if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 100 &&
        dart_rack->state_.manual_mode.is_trigger_lock_done == false) {
      dart_rack->trigger_motor_force_pid_.Update(-1000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
    } else {
      dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
      dart_rack->state_.manual_mode.is_trigger_lock_done = true;
    }
  } else {
    // 在滑台下拉阶段同时打开撒放器
    if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 50) {
      dart_rack->trigger_motor_force_pid_.Update(1000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
    } else {
      dart_rack->trigger_motor_force_pid_.Update(0.f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
    }
  }
  if (dart_rack->state_.manual_mode.is_trigger_lock_done == true &&
      dart_rack->state_.manual_mode.is_load_up_done == false) {
    const int32_t l_ticks = dart_rack->load_motor_l_odometer_.linear_ticks();
    const int32_t r_ticks = dart_rack->load_motor_r_odometer_.linear_ticks();
    constexpr int32_t kBrakeZone = 200000;
    constexpr int32_t kDeadZone = 50000;

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
  }
}

void DartStateAddUpdate() {
  // 继电器控制管脚PF1
  static uint32_t tick = 0;
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

  static AddState current_state = AddState::MOVING_BACK;
  float target_dm_angle = -0.020f;
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
    case AddState::MOVING_BACK:
      target_dm_angle = -0.020f;
      target_servo1 = S1 + 474.886f;
      target_servo2 = S2 + 190.831f;
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
      if (dart_rack->dart_count_ == DartCount::kThird || dart_rack->dart_count_ == DartCount::kFourth) {
        if (!g_add_limit_suppressed) {
          if (dart_rack->add_motor_odometer_.linear_ticks() > 300000) {
            float back_speed = (dart_rack->add_motor_odometer_.linear_ticks() < 500000) ? -1500.0f : -5000.0f;
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
      target_dm_angle = (dart_rack->dart_count_ == DartCount::kThird) ? -0.568f : 0.464f;
      target_servo1 = S1 + 474.886f;
      target_servo2 = S2 + 190.831f;
      dart_rack->add_motor_->SetCurrent(0);
      state_timer++;
      if (state_timer > 500) {
        state_timer = 0;
        current_state = AddState::CAUGHT;
      }
      break;

    case AddState::CAUGHT:
      target_dm_angle = (dart_rack->dart_count_ == DartCount::kThird) ? -0.568f : 0.464f;
      target_servo1 = S1 + 522.407f;
      target_servo2 = S2 + 154.871f;
      dart_rack->add_motor_->SetCurrent(0);
      state_timer++;
      if (state_timer > 500) {
        state_timer = 0;
        current_state = AddState::MOVING_FORWARD;
      }
      break;

    case AddState::MOVING_FORWARD:
      target_servo1 = S1 + 287.301f;
      target_servo2 = S2 + 65.63f;
      if (dart_rack->add_motor_odometer_.linear_ticks() < 2200000) {
        dart_rack->add_motor_speed_pid_.Update(5000.0f, dart_rack->add_motor_->rpm(), 1.0f);
        target_dm_angle = -0.020f;
        dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
      } else {
        dart_rack->add_motor_->SetCurrent(0);
        dart_rack->add_motor_speed_pid_.Clear();
        target_dm_angle = -0.020f;
        state_timer = 0;
        current_state = AddState::SUSPENDED;
      }
      break;

    case AddState::SUSPENDED:
      target_dm_angle = -0.020f;
      target_servo1 = S1 + 698.596f;
      target_servo2 = S2 + 373.522f;
      dart_rack->add_motor_->SetCurrent(0);
      state_timer++;
      if (state_timer > 500) {
        state_timer = 0;
        current_state = AddState::PLACED;
      }
      break;

    case AddState::PLACED:
      target_dm_angle = -0.020f;
      target_servo1 = S1 + 811.746f;
      target_servo2 = S2 + 453.321f;
      dart_rack->add_motor_->SetCurrent(0);
      state_timer++;
      if (state_timer > 1000 && state_timer <= 1005) {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
      }
      if (state_timer > 1000) {
        target_servo1 = S1 + 474.886f;
        target_servo2 = S2 + 190.831f;
      }
      if (state_timer > 1500) {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
        state_timer = 0;
        tick = 0;
        dart_rack->state_.manual_mode.add = PhaseState::kDone;
        current_state = AddState::MOVING_BACK;
        return;
      }
      break;
  }

  if (tick % 50 != 0 || dart_rack->dm_motor_->status() == 1) {
    dm_limiter.SetTarget(target_dm_angle);
    float dm_smooth = dm_limiter.Update(0.001f);
    // 把之前写死的 0.0f 的前馈速度换成规划器内部给出的当期阶跃速度 current_velocity()
    // 更新了 Kp=15.0, Kd=0.2 减小电机抵抗外部扭矩和抖动
    dart_rack->dm_motor_->SetMitCommand(dm_smooth, dm_limiter.current_velocity(), 0.0f, 25.0f, 1.0f);

    dart_rack->add_servo_->SetServoAngle(static_cast<uint16_t>(target_servo1), 1, 10);
    dart_rack->add_servo_->SetServoAngle(static_cast<uint16_t>(target_servo2), 2, 10);
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
  float target_dm_angle = 0.020f;
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
      target_dm_angle = -0.020f;
      target_servo1 = S1 + 287.301f;
      target_servo2 = S2 + 65.63f;
      if (dart_rack->add_motor_odometer_.linear_ticks() < 2200000) {
        dart_rack->add_motor_speed_pid_.Update(5000.0f, dart_rack->add_motor_->rpm(), 1.0f);
        dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
      } else {
        dart_rack->add_motor_->SetCurrent(0);
        dart_rack->add_motor_speed_pid_.Clear();
        state_timer = 0;
        current_state = AddState::SUSPENDED;
      }
      break;

    case AddState::SUSPENDED:
      target_dm_angle = -0.020f;
      target_servo1 = S1 + 698.596f;
      target_servo2 = S2 + 373.522f;
      dart_rack->add_motor_->SetCurrent(0);
      state_timer++;
      if (state_timer > 500) {
        state_timer = 0;
        current_state = AddState::PLACED;
      }
      break;

    case AddState::PLACED:
      target_dm_angle = -0.020f;
      target_servo1 = S1 + 811.746f;
      target_servo2 = S2 + 453.321f;
      dart_rack->add_motor_->SetCurrent(0);
      state_timer++;
      if (state_timer > 1000 && state_timer <= 1005) {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
      }
      if (state_timer > 1000) {
        target_servo1 = S1 + 474.886f;
        target_servo2 = S2 + 190.831f;
      }
      if (state_timer > 1500) {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
        state_timer = 0;
        tick = 0;
        dart_rack->state_.manual_mode.add = PhaseState::kDone;
        current_state = AddState::MOVING_FORWARD;
        return;
      }
      break;
  }

  if (tick % 50 != 0 || dart_rack->dm_motor_->status() == 1) {
    dm_limiter.SetTarget(target_dm_angle);
    float dm_smooth = dm_limiter.Update(0.001f);
    dart_rack->dm_motor_->SetMitCommand(dm_smooth, dm_limiter.current_velocity(), 0.0f, 25.0f, 1.0f);

    dart_rack->add_servo_->SetServoAngle(static_cast<uint16_t>(target_servo1), 1, 10);
    dart_rack->add_servo_->SetServoAngle(static_cast<uint16_t>(target_servo2), 2, 10);
  }

  glb_add_motor_linear = dart_rack->add_motor_odometer_.linear_ticks();
}

void DartStateAimUpdate() {
  dart_rack->state_.manual_mode.aim = PhaseState::kDone;  // 瞄准待实现
  // 速度小于0 trigger_motor_正转
  // pitch 和 yaw轴都采用sd卡里的数据
}

void DartStateFireUpdate() {
  static uint32_t fire_running_time = 0;
  // 释放扳机即可
  if (dart_rack->state_.manual_mode.fire == PhaseState::kUncomplete) {
    // 恢复使用测量的 running_time 防止堵转保护电机
    if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 100 && fire_running_time < 500) {
      dart_rack->trigger_motor_force_pid_.Update(1000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
      fire_running_time++;
    } else {
      dart_rack->trigger_motor_force_->SetCurrent(0);
      dart_rack->state_.manual_mode.fire = PhaseState::kDone;
      fire_running_time = 0;
    }
  } else {
    dart_rack->trigger_motor_force_->SetCurrent(0);
    dart_rack->state_.manual_mode.fire = PhaseState::kDone;
    fire_running_time = 0;
  }
}

void DartStateAdjustUpdate() {
  // 计算包含圈数的全段实际角度
  yaw_current_deg = dart_rack->yaw_encoder_->angle_deg();
  test_if_adjust_mode_is_running = 1.0f;
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
      dart_rack->trigger_motor_speed_pid_.Update(-8000.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
      dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
    } else {
      dart_rack->trigger_motor_->SetCurrent(0);
    }
  } else if (dart_rack->rc_->right_y() < -330) {
    if (dart_rack->trigger_motor_odometer_.stall_time() <= 100) {
      dart_rack->trigger_motor_speed_pid_.Update(8000.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
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
        dart_rack->trigger_motor_force_pid_.Update(1000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
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
        dart_rack->trigger_motor_force_pid_.Update(-1000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
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
      if (dart_rack->add_motor_odometer_.linear_ticks() < 2200000) {
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

  // 2. 为防使能指令和MIT控制同一周期并发造成CAN邮箱覆盖发送丢失，错开判断发送
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

    dart_rack->add_servo_->SetServoAngle(static_cast<uint16_t>(glb_servo_1_target), 1, 0);
  }

  int16_t left_y_val = dart_rack->rc_->left_y();
  if (std::abs(left_y_val) > 50) {
    // 累加摇杆值。可修改0.005f调整舵机转动速度
    glb_servo_2_target += static_cast<float>(left_y_val) * 0.01f;
    // 限制范围 100 到 500
    if (glb_servo_2_target > 1000.0f) glb_servo_2_target = 1000.0f;
    if (glb_servo_2_target < 0.0f) glb_servo_2_target = 0.0f;

    dart_rack->add_servo_->SetServoAngle(static_cast<uint16_t>(glb_servo_2_target), 2, 0);
  }

  // 更新拨弹电机的全局位置监视，方便FreeMaster查看
  // M2006 电机转子有 0-8191的绝对编码，这里直接读取原始反馈编码（注意由于36:1减速比，输出轴转一圈会经历36次0-8191）
  glb_add_motor_linear = dart_rack->add_motor_odometer_.linear_ticks();
}