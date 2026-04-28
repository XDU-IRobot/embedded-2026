#include "dart_statemachine.hpp"
#include "lcd_init.h"
#include <cmath>
#include <cstdio>  // Added for printf
#include "sd_card.h"
#include "librm/device/actuator/dm_motor.hpp"
extern bool is_lvgl_running;  // 引入定义在 main.cc 中的全局标志

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
volatile float glb_servo_1_target = 500.0f;  // 用于在 FreeMaster 监测舵机1目标角度
volatile float glb_servo_2_target = 500.0f;  // 用于在 FreeMaster 监测舵机2目标角度
volatile int32_t glb_add_motor_pos = 0;      // 用于在 FreeMaster 监测拨弹电机当前位置
volatile float glb_add_motor_linear = 0;

void DartStateMachineUpdate(DartState &state) {
  // 根据遥控器左拨杆位置设置状态
  // 为了检测状态切换，由于 state 每次都被重写，不方便检测边界，我们通过在其他模式中清理 manual_mode
  // 的状态来实现切回后重新初始化
  if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kDown) {
    state.unable = AbleState::kOn;
    state.manual_mode.enabled = AbleState::kOff;
    state.lvgl_mode.enabled = AbleState::kOff;
    state.adjust_mode.enabled = AbleState::kOff;
  } else if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kMid) {
    // 左拨杆向中，手动模式
    state.unable = AbleState::kOff;
    state.manual_mode.enabled = AbleState::kOn;
    state.lvgl_mode.enabled = AbleState::kOff;
    state.adjust_mode.enabled = AbleState::kOff;
  } else if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kUp &&
             dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kUp) {
    // 左右拨杆向上，自动模式(开启LVGL)
    state.unable = AbleState::kOff;
    state.manual_mode.enabled = AbleState::kOff;
    state.lvgl_mode.enabled = AbleState::kOn;
    state.adjust_mode.enabled = AbleState::kOff;
  } else if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kUp &&
             dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kMid) {
    // 左拨杆向上，右拨杆向中，调节模式
    state.unable = AbleState::kOff;
    state.manual_mode.enabled = AbleState::kOff;
    state.lvgl_mode.enabled = AbleState::kOff;
    state.adjust_mode.enabled = AbleState::kOn;
  }

  // 状态机处理逻辑
  if (state.lvgl_mode.enabled == AbleState::kOn) {
    // 仅仅在自动模式时开启 LVGL 调节
    is_lvgl_running = true;
  } else {
    is_lvgl_running = false;  // 其他情况关闭LVGL阻塞
  }

  if (state.unable == AbleState::kOn) {
    DartStateClear(state);
    DartStateUnableUpdate();
  } else if (state.manual_mode.enabled == AbleState::kOn) {
    DartStateManualUpdate();
  } else if (state.lvgl_mode.enabled == AbleState::kOn) {
    // 当处于自动模式时，清除手动模式的标志位，这样当你切回手动模式(且右拨杆上去之后)，它就会重新执行 Yaw 轴初始化
    DartManualModeClear(state.manual_mode);

    // 其他电机保持休眠或锁定
    dart_rack->load_motor_l_->SetCurrent(0);
    dart_rack->load_motor_r_->SetCurrent(0);
    dart_rack->trigger_motor_->SetCurrent(0);
    dart_rack->trigger_motor_force_->SetCurrent(0);
    dart_rack->add_motor_->SetCurrent(0);
    dart_rack->yaw_motor_->SetCurrent(0);  // 既然不想在调参时跟随转动，这里也一并锁死关闭
  } else if (state.adjust_mode.enabled == AbleState::kOn) {
    DartStateAdjustUpdate();
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
        // DartStateInitUpdate();
        dart_rack->state_.manual_mode.mode = ModeState::kload;
      } else if (dart_rack->state_.manual_mode.init == PhaseState::kDone) {
        // 初始化完成，进入下一个阶段
        dart_rack->state_.manual_mode.mode = ModeState::kload;
      }
      break;
    case ModeState::kload:
      if (dart_rack->state_.manual_mode.load == PhaseState::kUncomplete) {
        dart_rack->state_.manual_mode.mode = ModeState::kAdd;  // 一定要记得把这个注释掉啊!!!
        // DartStateLoadUpdate();  //... 装填操作
      } else if (dart_rack->state_.manual_mode.load == PhaseState::kDone) {
        dart_rack->state_.manual_mode.mode = ModeState::kAdd;
      }
      // 装填逻辑
      break;

    case ModeState::kAdd:
      if (dart_rack->state_.manual_mode.add == PhaseState::kUncomplete) {
        DartStateAddUpdate();
      } else if (dart_rack->state_.manual_mode.add == PhaseState::kDone) {
        // 加弹完成，进入下一个阶段
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
        dart_rack->state_.manual_mode.mode = ModeState::kUnable;
        uint8_t next_dart = static_cast<uint8_t>(dart_rack->dart_count_) + 1;
        if (next_dart > 3) {
          next_dart = 0;  // 四发过后回到底部重新开始
        }
        dart_rack->dart_count_ = static_cast<DartCount>(next_dart);
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
  // Yaw轴根据是第几发镖初始化
  if (!dart_rack->state_.manual_mode.is_yaw_init_done) {
    static float last_yaw_error = 0.0f;
    static float last_yaw_angle = 0.0f;
    static uint32_t stall_count = 0;
    static bool is_first_run = true;

    // 使用从 SD 卡读取并缓存在内存中的 Yaw 数组作为目标角度，而不是原来的 固定宏配置
    float yaw_target = Yaw[static_cast<uint8_t>(dart_rack->dart_count_)];
    float yaw_current = dart_rack->yaw_encoder_->angle_deg();
    float yaw_error = yaw_target - yaw_current;

    if (is_first_run) {
      last_yaw_error = yaw_error;
      last_yaw_angle = yaw_current;
      stall_count = 0;
      is_first_run = false;
    }

    // 检测静摩擦导致的停滞：若误差已在较小范围内（如 1.0 度以内），且角度变化极小 (<0.01度)
    if (std::abs(yaw_error) < 1.0f && std::abs(yaw_current - last_yaw_angle) < 0.01f) {
      stall_count++;
    } else {
      stall_count = 0;
    }

    // 完成条件：进入死区(0.05度) 或 冲过头越过零点(符号相异) 或 依靠防反冲静摩擦停滞检测(100个周期完全停止)
    if (std::abs(yaw_error) <= 0.05f || (last_yaw_error * yaw_error < 0.0f) || stall_count > 100) {
      dart_rack->yaw_motor_speed_pid_.Clear();
      dart_rack->yaw_motor_->SetCurrent(0);
      dart_rack->state_.manual_mode.is_yaw_init_done = true;
      is_first_run = true;  // 状态复位供下一发测试使用
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

  // 如果是第一发镖，首先全部转到限位并清除计圈器
  // 上膛电机初始化
  if (dart_rack->dart_count_ == DartCount::kFirst && dart_rack->state_.manual_mode.is_load_reset_done == false) {
    // 直接清除计圈器
    dart_rack->load_motor_l_speed_pid_.Update(.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
    dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
    dart_rack->load_motor_r_speed_pid_.Update(.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
    dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
    dart_rack->state_.manual_mode.is_load_reset_done = true;
    dart_rack->load_motor_l_odometer_.Reset();
    dart_rack->load_motor_r_odometer_.Reset();

    dart_rack->state_.manual_mode.is_trigger_reset_done = true;
    dart_rack->trigger_motor_odometer_.Reset();
  } else {
    dart_rack->load_motor_l_speed_pid_.Update(.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
    dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
    dart_rack->load_motor_r_speed_pid_.Update(.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
    dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
    dart_rack->state_.manual_mode.is_load_reset_done = true;
    dart_rack->state_.manual_mode.is_trigger_reset_done = true;
  }

  dart_rack->state_.manual_mode.is_add_init_done = true;

  // 初始化阶段不再旋转撒放器，直接设为完成
  dart_rack->state_.manual_mode.is_trigger_force_init_done = true;
  dart_rack->trigger_motor_force_->SetCurrent(0);

  // 全部检查完成后，初始化完成
  if (dart_rack->state_.manual_mode.is_yaw_init_done == true &&
      dart_rack->state_.manual_mode.is_load_reset_done == true &&
      dart_rack->state_.manual_mode.is_trigger_reset_done == true &&
      dart_rack->state_.manual_mode.is_trigger_force_init_done == true &&
      dart_rack->state_.manual_mode.is_add_init_done == true) {
    dart_rack->state_.manual_mode.init = PhaseState::kDone;
    dart_rack->yaw_motor_speed_pid_.Clear();
    dart_rack->yaw_motor_->SetCurrent(0);
    dart_rack->load_motor_l_speed_pid_.Update(0.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
    dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
    dart_rack->load_motor_r_speed_pid_.Update(0.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
    dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
    dart_rack->trigger_motor_speed_pid_.Update(.0f, dart_rack->trigger_motor_->rpm(), 1.0f);
    dart_rack->trigger_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->trigger_motor_speed_pid_.out()));
    dart_rack->trigger_motor_force_pid_.Update(.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
    dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
    dart_rack->add_motor_speed_pid_.Update(.0f, dart_rack->add_motor_->rpm(), 1.0f);
    dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
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
  if (dart_rack->state_.manual_mode.is_trigger_lock_done == true) {
    // 滑台还原
    if (dart_rack->load_motor_l_odometer_.linear_ticks() <= 0 ||
        dart_rack->load_motor_r_odometer_.linear_ticks() >= 0 ||
        dart_rack->state_.manual_mode.is_load_reset_done == false) {
      if (dart_rack->load_motor_l_odometer_.stall_time() <= 100 &&
          dart_rack->load_motor_r_odometer_.stall_time() <= 100) {
        dart_rack->load_motor_l_speed_pid_.Update(4000.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
        dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
        dart_rack->load_motor_r_speed_pid_.Update(-4000.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
        dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
      } else {
        dart_rack->load_motor_l_speed_pid_.Update(.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
        dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
        dart_rack->load_motor_r_speed_pid_.Update(.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
        dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
      }
    } else if (abs(dart_rack->load_motor_l_odometer_.linear_ticks() +
                   dart_rack->load_motor_r_odometer_.linear_ticks()) >= 100000) {
      if (dart_rack->load_motor_l_odometer_.linear_ticks() >= 0) {
        dart_rack->load_motor_l_speed_pid_.Update(-3000.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
        dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
      } else if (dart_rack->load_motor_r_odometer_.linear_ticks() <= 0) {
        dart_rack->load_motor_r_speed_pid_.Update(3000.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
        dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
      } else {
        dart_rack->load_motor_l_speed_pid_.Update(.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
        dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
        dart_rack->load_motor_r_speed_pid_.Update(.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
        dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
      }
    } else {
      dart_rack->state_.manual_mode.is_load_up_done = true;
      dart_rack->load_motor_l_speed_pid_.Update(.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
      dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
      dart_rack->load_motor_r_speed_pid_.Update(.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
      dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
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
 //  // 继电器控制管脚PF1
 //  static uint32_t tick = 0;
 //  tick++;
 //  dm_status = dart_rack->dm_motor_->status();
 //  dm_pos = dart_rack->dm_motor_->pos();
 //  dm_pos_int = static_cast<int32_t>(dm_pos * 1000.0f);
 //
 //  // 1. 监测到达妙还未使能(例如未接收反馈、上电重启、或报错保护), 尝试重新使能并清错
 //  if (dart_rack->dm_motor_->status() != 1) {  // 状态1即为 kEnable
 //    if (tick % 50 == 0) {                     // 防止CAN总线洪泛
 //      dart_rack->dm_motor_->SendInstruction(rm::device::DmMotorInstructions::kClearError);
 //      dart_rack->dm_motor_->SendInstruction(rm::device::DmMotorInstructions::kEnable);
 //    }
 //  }
 //  glb_add_motor_pos = dart_rack->add_motor_->encoder();
 //  glb_add_motor_linear = dart_rack->add_motor_odometer_.linear_ticks();
 //
 //  static AddState current_state = AddState::SUSPENDED;
 //  // 定义目标值变量，每帧根据状态更新
 //  float target_dm_angle = 0.0f;
 //  float target_servo1 = 0.0f;
 //  float target_servo2 = 0.0f;
 //  float target_add_current = 0.0f; // 拨弹轮目标电流
 //
 //  // 状态机流转
 //  switch (current_state) {
 //    case AddState::SUSPENDED:
 //      // [状态0] 悬空安全模式
 //      target_dm_angle = -0.59f;
 //      target_servo1 = 413.19f;
 //      target_servo2 = 200.83f;
 //      dart_rack->add_motor_->SetCurrent(0);
 //
 //      // 转移条件: 摇杆右推 -> 进入抓弹
 //      if (dart_rack->rc_->right_x() > 330) {
 //        current_state = AddState::CAUGHT;
 //      }
 //      break;
 //
 //    case AddState::CAUGHT:
 //      // [状态1] 抓弹模式
 //      target_dm_angle = (dart_rack->dart_count_ == DartCount::kFirst) ? -0.59f : 0.59f;
 //      target_servo1 = 505.041f;
 //      target_servo2 = 121.391f;
 //      dart_rack->add_motor_->SetCurrent(0);
 //
 //      // 转移条件: 摇杆左拉 -> 进入前移
 //      if (dart_rack->rc_->right_x() < -330) {
 //        current_state = AddState::MOVING_FORWARD;
 //      }
 //      break;
 //
 //    case AddState::MOVING_FORWARD:
 //      // [状态2] 前移模式 (执行动作中)
 //      target_dm_angle = (dart_rack->dart_count_ == DartCount::kFirst) ? -0.59f : 0.59f;
 //      target_servo1 = 505.041f;
 //      target_servo2 = 121.391f;
 //
 //      // 执行拨弹动作
 //      dart_rack->add_motor_speed_pid_.Update(2000.0f, dart_rack->add_motor_->rpm(), 1.0f);
 //      target_add_current = static_cast<float>(dart_rack->add_motor_speed_pid_.out());
 //
 //      // 转移条件: 编码器到位 -> 进入放弹
 //      if (dart_rack->add_motor_odometer_.linear_ticks() >= 2134190) {
 //        dart_rack->add_motor_->SetCurrent(0);
 //        dart_rack->add_motor_speed_pid_.Clear();
 //        current_state = AddState::PLACED;
 //      }
 //      break;
 //
 //    case AddState::PLACED:
 //      // [状态3] 放弹模式
 //      target_dm_angle = 0.0f;
 //      target_servo1 = 836.731f;
 //      target_servo2 = 359.53f;
 //      dart_rack->add_motor_->SetCurrent(0);
 //      // 触发继电器
 //      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
 //
 //      // 转移条件: 摇杆左拉 -> 进入回退
 //      if (dart_rack->rc_->right_x() < -330) {
 //        current_state = AddState::MOVING_BACK;
 //      }
 //      break;
 //
 //    case AddState::MOVING_BACK:
 //      // [状态4] 回退模式
 //      target_dm_angle = 0.0f;
 //      target_servo1 = 505.041f;
 //      target_servo2 = 121.391f;
 //      dart_rack->add_motor_->SetCurrent(0);
 //
 //      // 这里逻辑稍微特殊：根据你的原代码，回退是反向拨弹
 //      // 如果你的回退是纯机械复位不需要电机反转，可以去掉下面的 Update
 //      // 如果确实需要反转拨弹轮来辅助回退：
 //      dart_rack->add_motor_speed_pid_.Update(-2000.0f, dart_rack->add_motor_->rpm(), 1.0f);
 //      target_add_current = static_cast<float>(dart_rack->add_motor_speed_pid_.out());
 //
 //      // 转移条件: 编码器回位 -> 回到悬空
 //      if (dart_rack->add_motor_odometer_.linear_ticks() <= 150000) {
 //        // 复位拨弹PID
 //        dart_rack->add_motor_speed_pid_.Clear();
 //        current_state = AddState::SUSPENDED; // 循环回到初始状态
 //      }
 //      break;
 //  }
 //
 //  // --- 4. 统一发送指令 (每帧都发，保证实时性) ---
 //
 //  // 只有在非使能发送周期，或者电机已使能时才发送控制指令
 //  if (tick % 50 != 0 || dart_rack->dm_motor_->status() == 1) {
 //
 //    // 1. 发送 DM 电机 MIT 指令
 //    dart_rack->dm_motor_->SetMitCommand(target_dm_angle, 0.0f, 0.0f, 15.0f, 0.5f);
 //
 //    // 2. 发送 拨弹电机 电流指令
 //    // 注意：如果是 MOVING_BACK 状态且不需要反转拨弹，这里可能需要特殊处理
 //    if (current_state == AddState::MOVING_BACK) {
 //         // 根据你的原代码逻辑，回退时也在 SetCurrent
 //         dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(target_add_current));
 //    } else {
 //         dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(target_add_current));
 //    }
 //
 //    // 3. 发送 舵机 角度指令 (带限幅)
 //    // 使用 std::clamp 防止越界 (需包含 <algorithm>)
 //    target_servo1 = std::clamp(target_servo1, 410.0f, 1000.0f);
 //    target_servo2 = std::clamp(target_servo2, 410.0f, 1000.0f); // 修正了你原代码中 100/500 的逻辑笔误
 //
 //    dart_rack->add_servo_->SetServoAngle(static_cast<uint16_t>(target_servo1), 1, 0);
 //    dart_rack->add_servo_->SetServoAngle(static_cast<uint16_t>(target_servo2), 2, 0);
 //  }
 //
 //  // --- 5. 更新全局监视变量 ---
 //  glb_add_motor_pos = dart_rack->add_motor_->encoder();
 //  glb_add_motor_linear = dart_rack->add_motor_odometer_.linear_ticks();
 // }

   // 继电器控制管脚PF1
  if (dart_rack->rc_->left_x() > 330 && dart_rack->rc_->right_x() < -330) {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
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
    static float smoothed_angle = 0.0f;
    static bool is_dm_first_run = true;

    // 当第一次成功接收到使能反馈时，把电机的当前实际位置作为初始目标，防止上电时猛然回零
    if (is_dm_first_run && dart_rack->dm_motor_->status() == 1) {
      smoothed_angle = dart_rack->dm_motor_->pos();
      is_dm_first_run = false;
    }

    // 将摇杆输入作为目标角度的累加值（即控制速度），而不是绝对值。回中时停止改变目标，保持当前位置。
    if (std::abs(dart_rack->rc_->right_x()) > 50) {  // 带死区抵抗摇杆中位漂移
      // 0.00005f 是速度系数，决定拨满时电机转动有多快，觉得转太快/慢自己微调
      smoothed_angle += static_cast<float>(dart_rack->rc_->right_x()) * 0.00001f;

      // 限幅保护，防止一直推摇杆导致目标角度被积分到无限大引起越界报错 (P_MAX默认填了3.0)
      if (smoothed_angle > 1.0f) smoothed_angle = 1.0f;
      if (smoothed_angle < -1.0f) smoothed_angle = -1.0f;
    }

    dm_smoothed_angle = smoothed_angle;  // 更新全局变量供 FreeMaster 查看
    dm_smoothed_angle_int = static_cast<int32_t>(smoothed_angle * 1000.0f);

    // 更新全局监测变量
    dm_status = dart_rack->dm_motor_->status();
    dm_pos = dart_rack->dm_motor_->pos();
    dm_pos_int = static_cast<int32_t>(dm_pos * 1000.0f);

    // 防过流保护，先给个柔和刚度防报错：kp 15.0f, kd 0.5f，确认能转了你再往大去调
    dart_rack->dm_motor_->SetMitCommand(smoothed_angle, 0.0f, 0.0f, 15.0f, 0.5f);
  }

  // M2006 加弹电机 (ID: 6) 摇杆 Y 轴控制逻辑
  static float smoothed_add_speed = 0.0f;
  if (std::abs(dart_rack->rc_->right_y()) > 50) {  // 死区防止误触
    // 摇杆行程为 -660 到 660，将其线性映射到 -3000 到 3000 RPM (可根据需要调整最大速度)
    float add_target_speed = static_cast<float>(dart_rack->rc_->right_y()) * (4000.0f / 660.0f);

    // 使用低通滤波平滑目标速度，柔和手感
    smoothed_add_speed += (add_target_speed - smoothed_add_speed) * 0.1f;  // 平滑系数

    dart_rack->add_motor_speed_pid_.Update(smoothed_add_speed, dart_rack->add_motor_->rpm(), 1.0f);
    dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
  } else {
    // 回中时不设PID，直接设为0电流，完全顺滑
    smoothed_add_speed = 0.0f;
    dart_rack->add_motor_speed_pid_.Clear();
    dart_rack->add_motor_->SetCurrent(0);

    int16_t left_x_val = dart_rack->rc_->left_x();
    if (std::abs(left_x_val) > 50) {
      // 累加摇杆值。可修改0.005f调整舵机转动速度
      glb_servo_1_target += static_cast<float>(left_x_val) * 0.01f;
      // 限制范围 410 到 640
      if (glb_servo_1_target > 1000.0f) glb_servo_1_target = 1000.0f;
      if (glb_servo_1_target < 410.0f) glb_servo_1_target = 410.0f;

      dart_rack->add_servo_->SetServoAngle(static_cast<uint16_t>(glb_servo_1_target), 1, 0);
    }

    int16_t left_y_val = dart_rack->rc_->left_y();
    if (std::abs(left_y_val) > 50) {
      // 累加摇杆值。可修改0.005f调整舵机转动速度
      glb_servo_2_target += static_cast<float>(left_y_val) * 0.01f;
      // 限制范围 100 到 500
      if (glb_servo_2_target > 500.0f) glb_servo_2_target = 500.0f;
      if (glb_servo_2_target < 100.0f) glb_servo_2_target = 100.0f;

      dart_rack->add_servo_->SetServoAngle(static_cast<uint16_t>(glb_servo_2_target), 2, 0);
    }
  }

  // 更新拨弹电机的全局位置监视，方便FreeMaster查看
  // M2006 电机转子有 0-8191 的绝对编码，这里直接读取原始反馈编码（注意由于36:1减速比，输出轴转一圈会经历36次0-8191）
  glb_add_motor_pos = dart_rack->add_motor_->encoder();
  glb_add_motor_linear= dart_rack->add_motor_odometer_.linear_ticks();
}

void DartStateAimUpdate() {
  dart_rack->state_.manual_mode.aim = PhaseState::kDone;  // 瞄准待实现
}

void DartStateFireUpdate() {
  static uint32_t fire_running_time = 0;
  // 释放扳机即可
  if (dart_rack->state_.manual_mode.fire == PhaseState::kUncomplete) {
    // 恢复使用测量的 running_time 防止堵转保护电机
    if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 100 && fire_running_time < 200) {
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
