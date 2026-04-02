#include "dart_statemachine.hpp"
#include <cmath>
#include <cstdio> // Added for printf

int32_t debug_trigger_force_encoder = 0;
uint32_t debug_trigger_force_stall_time = 0;
uint32_t debug_adjust_motor_running_time = 0;

// 全局观测变量，用于FreeMASTER或调试
bool debug_trigger_force_init_done = false;
bool debug_load_down_done = false;
bool debug_trigger_lock_done = false;
bool debug_load_up_done = false;
uint32_t debug_trigger_open_time = 0;
uint32_t debug_trigger_lock_time = 0;
uint32_t debug_load_l_stall_time = 0;
uint32_t debug_load_r_stall_time = 0;

float debug_trigger_force_rpm = 0.0f;
int32_t debug_load_l_encoder = 0;
float debug_load_l_rpm = 0.0f;
int32_t debug_load_r_encoder = 0;
float debug_load_r_rpm = 0.0f;

float debug_yaw_rpm = 0.0f;
int32_t debug_yaw_encoder = 0;
float debug_yaw_angle = 0.0f;

void DartStateMachineUpdate(DartState &state) {
  // 更新用于 FreeMASTER 观测的调试变量
  debug_trigger_force_encoder = dart_rack->trigger_motor_force_->encoder();
  debug_trigger_force_rpm = dart_rack->trigger_motor_force_->rpm();
  debug_trigger_force_stall_time = dart_rack->trigger_motor_force_odometer_.stall_time();

  debug_load_l_encoder = dart_rack->load_motor_l_->encoder();
  debug_load_l_rpm = dart_rack->load_motor_l_->rpm();
  debug_load_r_encoder = dart_rack->load_motor_r_->encoder();
  debug_load_r_rpm = dart_rack->load_motor_r_->rpm();

  debug_yaw_rpm = dart_rack->yaw_motor_->rpm();
  debug_yaw_encoder = dart_rack->yaw_motor_->encoder();
  debug_yaw_angle = dart_rack->yaw_encoder_->angle_deg() +
                    static_cast<float>(static_cast<int16_t>(dart_rack->yaw_encoder_->rotations())) * 360.0f;

  // 根据遥控器左拨杆位置设置状态
  if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kDown) {
    // 左拨杆向下，无力状态
    state.unable = AbleState::kOn;
    state.manual_mode.enabled = AbleState::kOff;
    state.auto_mode.enabled = AbleState::kOff;
    state.adjust_mode.enabled = AbleState::kOff;
  } else if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kMid) {
    // 左拨杆向中，手动模式
    state.unable = AbleState::kOff;
    state.manual_mode.enabled = AbleState::kOn;
    state.auto_mode.enabled = AbleState::kOff;
    state.adjust_mode.enabled = AbleState::kOff;
  } else if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kUp &&
             dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kUp) {
    // 左右拨杆向上，自动模式
    state.unable = AbleState::kOff;
    state.manual_mode.enabled = AbleState::kOff;
    state.auto_mode.enabled = AbleState::kOn;
    state.adjust_mode.enabled = AbleState::kOff;
  } else if (dart_rack->rc_->switch_l() == rm::device::DR16::SwitchPosition::kUp &&
             dart_rack->rc_->switch_r() == rm::device::DR16::SwitchPosition::kMid) {
    state.unable = AbleState::kOff;
    state.manual_mode.enabled = AbleState::kOff;
    state.auto_mode.enabled = AbleState::kOff;
    state.adjust_mode.enabled = AbleState::kOn;
  }
  // 状态机处理逻辑
  if (state.unable == AbleState::kOn) {
    DartStateClear(state);
    DartStateUnableUpdate();
    return;
  } else if (state.manual_mode.enabled == AbleState::kOn) {
    DartStateManualUpdate();
  } else if (state.auto_mode.enabled == AbleState::kOn) {
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
        dart_rack->state_.manual_mode.mode = ModeState::kload;
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
        dart_rack->dart_count_ = static_cast<DartCount>(static_cast<uint8_t>(dart_rack->dart_count_) + 1);
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
}

void DartStateInitUpdate() {
  // 增加静态变量用于检测由于摩擦和死区导致的来回震荡现象
  static float last_yaw_error = 0.0f;
  static int osc_count = 0;
  static bool is_first_run = true;

  // Yaw轴根据是第几发镖初始化
  if (!dart_rack->state_.manual_mode.is_yaw_init_done) {
    float yaw_target = DartRack::kYawEcd[static_cast<uint8_t>(dart_rack->dart_count_)];
    // 直接在状态机内计算包含圈数补偿的绝对全局角度
    float yaw_current = dart_rack->yaw_encoder_->angle_deg() +
                        static_cast<float>(static_cast<int16_t>(dart_rack->yaw_encoder_->rotations())) * 360.0f;
    float yaw_error = yaw_target - yaw_current;

    // 优化的偏差处理防止突然掉头疯转（越过180度或-180度时的处理）
    while (yaw_error > 180.0f) yaw_error -= 360.0f;
    while (yaw_error < -180.0f) yaw_error += 360.0f;

    // 限制最大规划偏差在正负15度以内
    if (yaw_error > 15.0f) yaw_error = 15.0f;
    else if (yaw_error < -15.0f) yaw_error = -15.0f;

    // 初始化第一次偏差
    if (is_first_run) {
      last_yaw_error = yaw_error;
      is_first_run = false;
    }

    // 检测误差的正负号反转 (意味着已经冲过目标点一次)
    if (last_yaw_error * yaw_error < 0.0f) {
      osc_count++;
    }
    last_yaw_error = yaw_error;

    // 误差大于1.0度 并且 冲出/震荡的此时少于2次，继续进行调节（保留起步大电流克服摩擦）
    if (std::abs(yaw_error) > 1.0f && osc_count < 2) {
      float target_speed = yaw_error * 500.0f;

      // 正常的正比例限幅，最大 2000.0f
      if (target_speed > 2000.0f) target_speed = 2000.0f;
      else if (target_speed < -2000.0f) target_speed = -2000.0f;

      // 克服静摩擦力的最低起步速度 500.0f
      if (target_speed > 0.0f && target_speed < 500.0f) target_speed = 500.0f;
      else if (target_speed < 0.0f && target_speed > -500.0f) target_speed = -500.0f;

      // 修复：大疆 C610 电调在受到上述那种极限方波暴力震荡时，内部速度计算会溢出导致反馈 -18000 的极值
      // 为了保护 PID 不炸掉，我们添加一层异常值滤波：过滤掉毫无物理可能性的转速
      float safe_rpm = dart_rack->yaw_motor_->rpm();
      if (std::abs(safe_rpm) > 12000.0f) {
         safe_rpm = debug_yaw_rpm; // 如果读到如 -18000 这样的跳变值，直接沿用上次的合理值
      }
      debug_yaw_rpm = safe_rpm; // 更新全局观测

      dart_rack->yaw_motor_speed_pid_.Update(target_speed, safe_rpm, 1.0f);
      dart_rack->yaw_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->yaw_motor_speed_pid_.out()));
    } else {
      // 到达目标范围，或检测到产生来回震荡迹象后，强制妥协断电，直接视为完成
      dart_rack->yaw_motor_speed_pid_.Update(0.0f, dart_rack->yaw_motor_->rpm(), 1.0f);
      dart_rack->yaw_motor_->SetCurrent(0);
      dart_rack->state_.manual_mode.is_yaw_init_done = true;
      osc_count = 0; // 重置计数器供下一发使用
      is_first_run = true;
    }
  } else {
    // 已经满足过一次条件就将其锁在停止状态
    dart_rack->yaw_motor_speed_pid_.Update(0.0f, dart_rack->yaw_motor_->rpm(), 1.0f);
    dart_rack->yaw_motor_->SetCurrent(0);
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

  // 全部检查完成后，初始化完成 (移除了 is_trigger_force_init_done，将其放到 load 中执行)
  if (dart_rack->state_.manual_mode.is_yaw_init_done == true &&
      dart_rack->state_.manual_mode.is_load_reset_done == true &&
      dart_rack->state_.manual_mode.is_trigger_reset_done == true &&
      dart_rack->state_.manual_mode.is_add_init_done == true) {
    dart_rack->state_.manual_mode.init = PhaseState::kDone;
    dart_rack->yaw_motor_speed_pid_.Update(.0f, dart_rack->yaw_motor_->rpm(), 1.0f);
    dart_rack->yaw_motor_->SetCurrent(static_cast<rm::i16>(dart_rack->yaw_motor_speed_pid_.out()));
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
  bool target_trigger_active = false;
  float target_trigger_speed = 0.0f;

  bool target_load_active = false;
  float target_load_speed_l = 0.0f;
  float target_load_speed_r = 0.0f;

  static uint32_t trigger_open_time = 0;
  static uint32_t trigger_lock_time = 0;

  // 更新全局观测变量
  debug_trigger_force_init_done = dart_rack->state_.manual_mode.is_trigger_force_init_done;
  debug_load_down_done = dart_rack->state_.manual_mode.is_load_down_done;
  debug_trigger_lock_done = dart_rack->state_.manual_mode.is_trigger_lock_done;
  debug_load_up_done = dart_rack->state_.manual_mode.is_load_up_done;
  debug_trigger_open_time = trigger_open_time;
  debug_trigger_lock_time = trigger_lock_time;
  debug_load_l_stall_time = dart_rack->load_motor_l_odometer_.stall_time();
  debug_load_r_stall_time = dart_rack->load_motor_r_odometer_.stall_time();

  // Phase 1: 打开撒放器 + 滑台下拉，同时进行
  if (dart_rack->state_.manual_mode.is_trigger_force_init_done == false) {
    // 恢复原有的撒放器堵转时间检测，防止强制通电烧毁电机
    if (trigger_open_time < 200 && dart_rack->trigger_motor_force_odometer_.stall_time() <= 50) {
      target_trigger_active = true;
      target_trigger_speed = 1000.0f;
      trigger_open_time++;
    } else {
      dart_rack->state_.manual_mode.is_trigger_force_init_done = true;
      target_trigger_speed = 0.0f;
    }
  }

  if (dart_rack->state_.manual_mode.is_load_down_done == false) {
    // 添加对滑台是否移动离开死区的检测，防止通电起步阶段速度为0瞬间被误判为堵转
    bool l_moved = std::abs(dart_rack->load_motor_l_odometer_.linear_ticks()) > 1000;
    bool r_moved = std::abs(dart_rack->load_motor_r_odometer_.linear_ticks()) > 1000;

    // 恢复为两个电机均必须堵转才算完成 (!l_moved || !r_moved) || ( stall_time<=100 || stall_time<=100 )
    if ((!l_moved || !r_moved) ||
        (dart_rack->load_motor_l_odometer_.stall_time() <= 100 && dart_rack->load_motor_r_odometer_.stall_time() <= 100)) {
      target_load_active = true;
      if (dart_rack->load_motor_r_odometer_.linear_ticks() > DartRack::kTriggerEcdMax ||
          dart_rack->load_motor_l_odometer_.linear_ticks() < -DartRack::kTriggerEcdMax) {
        target_load_speed_l = -1500.0f;
        target_load_speed_r = 1500.0f;
      } else {
        target_load_speed_l = -3000.0f;
        target_load_speed_r = 3000.0f;
      }
    } else {
      dart_rack->state_.manual_mode.is_load_down_done = true;
      target_trigger_speed = 0.0f;
    }
  }

  // Phase 2 & 3: 两者均完成后，依次锁定撒放器并上拉还原
  if (dart_rack->state_.manual_mode.is_load_down_done == true &&
      dart_rack->state_.manual_mode.is_trigger_force_init_done == true) {

    // Phase 2: 锁定撒放器 (在滑台完全下拉即 load电机堵转后 开始)
    if (dart_rack->state_.manual_mode.is_trigger_lock_done == false) {
      // 核心修复嵌套逻辑：在撒放器锁定的这段时间里，必须强制保持load电机的下拉力和堵转状态！
      // 否则一旦进入这个阶段target_load_active变为false从而断电，弹簧会瞬间将滑台暴力抽回导致撒放器无法咬合弦！
      target_load_active = true;
      if (dart_rack->load_motor_r_odometer_.linear_ticks() > DartRack::kTriggerEcdMax ||
          dart_rack->load_motor_l_odometer_.linear_ticks() < -DartRack::kTriggerEcdMax) {
        target_load_speed_l = -1500.0f;
        target_load_speed_r = 1500.0f;
      } else {
        target_load_speed_l = -3000.0f;
        target_load_speed_r = 3000.0f;
      }

      // 恢复扳机闭合的堵转保护
      if (trigger_lock_time < 200 && dart_rack->trigger_motor_force_odometer_.stall_time() <= 50) {
        target_trigger_active = true;
        target_trigger_speed = -1000.0f;
        trigger_lock_time++;
      } else {
        dart_rack->state_.manual_mode.is_trigger_lock_done = true;
        target_trigger_speed = 0.0f;
      }
    }
    // Phase 3: 滑台还原
    else if (dart_rack->state_.manual_mode.is_load_up_done == false && dart_rack->state_.manual_mode.is_trigger_lock_done == true) {
      bool l_done = dart_rack->load_motor_l_odometer_.linear_ticks() >= 0;
      bool r_done = dart_rack->load_motor_r_odometer_.linear_ticks() <= 0;

      if (!l_done || !r_done || dart_rack->state_.manual_mode.is_load_reset_done == false) {
        if (dart_rack->load_motor_l_odometer_.stall_time() <= 100 &&
            dart_rack->load_motor_r_odometer_.stall_time() <= 100) {
          target_load_active = true;
          target_load_speed_l = l_done ? 0.0f : 3000.0f;
          target_load_speed_r = r_done ? 0.0f : -3000.0f;
        } else {
           dart_rack->state_.manual_mode.is_load_up_done = true;
        }
      } else {
        dart_rack->state_.manual_mode.is_load_up_done = true;
      }
    }
  }

  // 统一输出电机指令，避免函数内发生覆盖
  if (target_trigger_active) {
    dart_rack->trigger_motor_force_pid_.Update(target_trigger_speed, dart_rack->trigger_motor_force_->rpm(), 1.0f);
    dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
  } else {
    // 彻底摒弃复杂容易被物理弹簧扯开的电子弹簧，改用恒定维持强死力
    if (dart_rack->state_.manual_mode.is_trigger_force_init_done == true &&
        dart_rack->state_.manual_mode.is_trigger_lock_done == false) {
      // 处于打开保持阶段，给予一个强劲的负保持电流 (-3000) 确保机件不会软缩回去导致看起来像提早关闭
      dart_rack->trigger_motor_force_pid_.Update(.0, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
    } else if (dart_rack->state_.manual_mode.is_trigger_lock_done == true) {
      // 处于锁定维持阶段，给予正向维持电流
      dart_rack->trigger_motor_force_pid_.Update(.0, dart_rack->trigger_motor_force_->rpm(), 1.0f);
      dart_rack->trigger_motor_force_->SetCurrent(static_cast<rm::i16>(-dart_rack->trigger_motor_force_pid_.out()));
    } else {
      dart_rack->trigger_motor_force_->SetCurrent(0);
    }
  }

  if (target_load_active) {
    dart_rack->load_motor_l_speed_pid_.Update(target_load_speed_l, dart_rack->load_motor_l_->rpm(), 1.0f);
    dart_rack->load_motor_r_speed_pid_.Update(target_load_speed_r, dart_rack->load_motor_r_->rpm(), 1.0f);
    dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
    dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
  } else {
    // 还原后彻底将速度PID清0并直接断电为0电流，防止因保留上次刹车产生的巨大负电流造成的迅速反抽下滑
    dart_rack->load_motor_l_speed_pid_.Update(0.0f, dart_rack->load_motor_l_->rpm(), 1.0f);
    dart_rack->load_motor_r_speed_pid_.Update(0.0f, dart_rack->load_motor_r_->rpm(), 1.0f);
    dart_rack->load_motor_l_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_l_speed_pid_.out()));
    dart_rack->load_motor_r_->SetCurrent(static_cast<rm::i16>(dart_rack->load_motor_r_speed_pid_.out()));
  }

  if (dart_rack->state_.manual_mode.is_trigger_lock_done == true &&
      dart_rack->state_.manual_mode.is_load_up_done == true &&
      dart_rack->state_.manual_mode.is_load_down_done == true &&
      dart_rack->state_.manual_mode.is_trigger_force_init_done == true) {
    dart_rack->state_.manual_mode.load = PhaseState::kDone;
    trigger_open_time = 0;
    trigger_lock_time = 0;
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
  float yaw_current_deg = dart_rack->yaw_encoder_->angle_deg() +
                          static_cast<float>(static_cast<int16_t>(dart_rack->yaw_encoder_->rotations())) * 360.0f;

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
    debug_adjust_motor_running_time++;
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
    debug_adjust_motor_running_time++;
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
    debug_adjust_motor_running_time = 0;
    dart_rack->trigger_motor_force_->SetCurrent(0);
  }
}