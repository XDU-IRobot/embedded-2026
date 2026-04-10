#include "dart_statemachine.hpp"
#include "lcd_init.h"
#include <cmath>
#include <cstdio> // Added for printf
#include "sd_card.h"
extern bool is_lvgl_running; // 引入定义在 main.cc 中的全局标志

// 引入 LVGL 中使用的参数存储数组
extern float Pitch[4];
extern float Yaw[4];

extern int current_category; // 0 for Pitch, 1 for Yaw
extern int current_index;
extern float temp_val;
extern bool is_editing_val;

void DartStateMachineUpdate(DartState &state) {
  // 根据遥控器左拨杆位置设置状态
  // 为了检测状态切换，由于 state 每次都被重写，不方便检测边界，我们通过在其他模式中清理 manual_mode 的状态来实现切回后重新初始化
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
    is_lvgl_running = false; // 其他情况关闭LVGL阻塞
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
      dart_rack->yaw_motor_->SetCurrent(0); // 既然不想在调参时跟随转动，这里也一并锁死关闭
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
        DartStateInitUpdate();
      } else if (dart_rack->state_.manual_mode.init == PhaseState::kDone) {
        // 初始化完成，进入下一个阶段
        //dart_rack->state_.manual_mode.mode = ModeState::kload;
      }
      break;
    case ModeState::kload:
      if (dart_rack->state_.manual_mode.load == PhaseState::kUncomplete) {
        DartStateLoadUpdate();  //... 装填操作
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
             next_dart = 0; // 四发过后回到底部重新开始
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
            is_first_run = true; // 状态复位供下一发测试使用
        } else {
            float target_speed = yaw_error * 1000.0f;
            if (target_speed > 1000.0f) target_speed = 1000.0f;
            else if (target_speed < -1000.0f) target_speed = -1000.0f;

            dart_rack->yaw_motor_speed_pid_.Update(target_speed, dart_rack->yaw_motor_->rpm(), 1.0f);
            dart_rack->yaw_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->yaw_motor_speed_pid_.out()));
        }
        last_yaw_error = yaw_error;
        last_yaw_angle = yaw_current;
    }

    // 如果是第一发镖，首先全部转到限位并清除计圈器
    // 上膛电机初始化
    if (dart_rack->dart_count_ == DartCount::kFirst && dart_rack->state_.manual_mode.is_load_reset_done == false) {
        //直接清除计圈器
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
        dart_rack->state_.manual_mode.is_trigger_force_init_done == true && dart_rack->state_.manual_mode.
        is_add_init_done == true) {
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
    if (dart_rack->load_motor_l_odometer_.stall_time() <= 100 &&
        dart_rack->load_motor_r_odometer_.stall_time() <= 100 && dart_rack->state_.manual_mode.is_load_down_done ==
        false) {
        if (dart_rack->load_motor_r_odometer_.linear_ticks()>DartRack::kTriggerEcdMax||dart_rack->load_motor_l_odometer_.linear_ticks()<-DartRack::kTriggerEcdMax) {

            dart_rack->load_motor_l_speed_pid_.Update(-1500.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
            dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
            dart_rack->load_motor_r_speed_pid_.Update(1500.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
            dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
        }
        else {

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
            dart_rack->trigger_motor_force_->SetCurrent(
                static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
        } else {
            dart_rack->trigger_motor_force_pid_.Update(0.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
            dart_rack->trigger_motor_force_->SetCurrent(
                static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
            dart_rack->state_.manual_mode.is_trigger_lock_done = true;
        }
    } else {
        // 在滑台下拉阶段同时打开撒放器
        if (dart_rack->trigger_motor_force_odometer_.stall_time() <= 50) {
            dart_rack->trigger_motor_force_pid_.Update(1000.0f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
            dart_rack->trigger_motor_force_->SetCurrent(
                static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
        } else {
            dart_rack->trigger_motor_force_pid_.Update(0.f, dart_rack->trigger_motor_force_->rpm(), 1.0f);
            dart_rack->trigger_motor_force_->SetCurrent(
                static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
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
  // 加弹逻辑
  if (dart_rack->dart_count_ == DartCount::kFirst) {
    dart_rack->state_.manual_mode.add = PhaseState::kDone;  // 第一发不用换弹
  }

  if (dart_rack->state_.manual_mode.add == PhaseState::kUncomplete) {
    const auto add_index = static_cast<uint8_t>(dart_rack->dart_count_) - 1;
    const auto target_ticks = DartRack::kAddEcd[add_index];
    if (dart_rack->state_.manual_mode.is_add_down_done == false) {
      if (dart_rack->state_.manual_mode.is_add_down_done == false &&
          dart_rack->add_motor_odometer_.linear_ticks() > target_ticks) {
        dart_rack->add_motor_speed_pid_.Update(-300.0f, dart_rack->add_motor_->rpm(), 1.0f);
        dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
      } else if (dart_rack->state_.manual_mode.is_add_down_done == false &&
                 dart_rack->add_motor_odometer_.linear_ticks() <= target_ticks) {
        dart_rack->state_.manual_mode.is_add_down_done = true;
        dart_rack->add_motor_speed_pid_.Update(.0f, dart_rack->add_motor_->rpm(), 1.0f);
        dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
      } else {
        dart_rack->add_motor_speed_pid_.Update(.0f, dart_rack->add_motor_->rpm(), 1.0f);
        dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
      }
    }
    if (dart_rack->state_.manual_mode.is_add_down_done == true &&
        dart_rack->state_.manual_mode.is_add_plate_done == false) {
      if (dart_rack->ticks <= 1000) {
        dart_rack->add_plate_servo_->SetServoAngle(DartRack::kAddPlateUnlockEcd[add_index], add_index, 0);
        dart_rack->ticks++;
        dart_rack->add_motor_speed_pid_.Update(.0f, dart_rack->add_motor_->rpm(), 1.0f);
        dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
      } else {
        dart_rack->state_.manual_mode.is_add_plate_done = true;
        dart_rack->ticks = 0;
        dart_rack->add_motor_speed_pid_.Update(.0f, dart_rack->add_motor_->rpm(), 1.0f);
        dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
      }
    }
    if (dart_rack->state_.manual_mode.is_add_plate_done == true &&
        dart_rack->state_.manual_mode.is_add_up_done == false) {
      if (dart_rack->add_motor_odometer_.linear_ticks() < 0 && dart_rack->add_motor_odometer_.stall_time() <= 100) {
        dart_rack->add_motor_speed_pid_.Update(300.0f, dart_rack->add_motor_->rpm(), 1.0f);
        dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
      } else {
        dart_rack->state_.manual_mode.is_add_up_done = true;
        dart_rack->add_motor_speed_pid_.Update(.0f, dart_rack->add_motor_->rpm(), 1.0f);
        dart_rack->add_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->add_motor_speed_pid_.out()));
        // dart_rack->add_plate_servo_->SetServoAngle(
        //     DartRack::[add_index], add_index, 0);
      }
    }
    if (dart_rack->state_.manual_mode.is_add_plate_done == true &&
        dart_rack->state_.manual_mode.is_add_up_done == true &&
        dart_rack->state_.manual_mode.is_add_down_done == true) {
      dart_rack->state_.manual_mode.add = PhaseState::kDone;
    }
  }
  if (dart_rack->state_.manual_mode.add == PhaseState::kDone) {
    dart_rack->add_motor_->SetCurrent(0);
  }
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
  float yaw_current_deg = dart_rack->yaw_encoder_->angle_deg();

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
    dart_rack->add_plate_servo_->SetServoAngle(593, 0, 0);
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
    dart_rack->add_plate_servo_->SetServoAngle(204, 1, 0);

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
    dart_rack->add_plate_servo_->SetServoAngle(214, 2, 0);

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