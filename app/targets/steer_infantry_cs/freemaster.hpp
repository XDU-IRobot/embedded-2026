#ifndef STEER_INFANTRY_CS_FREEMASTER_HPP
#define STEER_INFANTRY_CS_FREEMASTER_HPP

#include "Chassis.hpp"

/**
 * @brief FreeMASTER 100 Hz 监视快照。
 *
 * 保持为全局 volatile 对象，使 FreeMASTER 可以从单一符号
 * `freemaster_data` 读取完整快照。成员仅用于观测，不应由
 * FreeMASTER 写入，否则会破坏本板的实际控制状态。
 */
struct FreeMasterData {
  u32 sample_count{};

  // 云台板下发的 CAN 数据。
  f32 remote_speed_x{};
  f32 remote_speed_y{};
  u8 chassis_mode{};
  u8 ui_show_flag{};
  u8 get_target_flag{};
  u8 suggest_fire_flag{};
  i8 aim_speed_change{};
  u16 robot_hp[5]{};

  // 底部 Yaw 编码器及与底盘前向的差值。
  u16 chassis_yaw_encoder{};
  f32 chassis_yaw_angle_rad{};
  f32 chassis_yaw_delta_rad{};

  // 官方裁判系统数据。
  u8 robot_id{};
  u8 robot_level{};
  u16 current_hp{};
  u16 maximum_hp{};
  u16 chassis_power_limit{};
  u16 buffer_energy{};
  u16 shooter_17mm_heat{};
  u16 shooter_heat_limit{};
  f32 projectile_initial_speed{};
  u8 power_management_gimbal_output{};
  u8 power_management_chassis_output{};
  u8 power_management_shooter_output{};
  u16 projectile_allowance_17mm{};
  u16 remaining_gold_coin{};

  // 底盘运行状态与超级电容反馈。
  u8 chassis_move_state{};
  u8 buff_state{};
  u8 speed_mode{};
  u8 supercap_energy{};
  u8 supercap_error_code{};
  u16 supercap_referee_power_limit{};
  u16 supercap_referee_energy_buffer{};

  // 四个舵电机：顺序为左前、右前、左后、右后。
  u16 steer_encoder[4]{};
  f32 steer_position_rad[4]{};
  i16 steer_rpm[4]{};
  i16 steer_feedback_current[4]{};
  f32 steer_command[4]{};

  // 四个驱动电机：顺序为左前、右前、左后、右后。
  i16 wheel_rpm[4]{};
  i16 wheel_feedback_current[4]{};
  f32 wheel_command[4]{};

  // 四舵轮控制器内部的最终目标量。
  f32 chassis_target_vx{};
  f32 chassis_target_vy{};
  f32 chassis_target_w{};
};

inline volatile FreeMasterData freemaster_data{};

/**
 * @brief 将当前底盘运行数据复制到 FreeMASTER 快照。
 *
 * 由 GlobalWarehouse::SubLoop100Hz() 调用；函数不参与控制决策，
 * 只读取当前状态并更新 freemaster_data。
 */
inline void freemaster() {
  if (globals == nullptr || chassis == nullptr || globals->gimbal_communicator == nullptr ||
      globals->referee_data == nullptr || globals->super_cap == nullptr || globals->yaw_motor == nullptr ||
      globals->steer_lf == nullptr || globals->steer_rf == nullptr || globals->steer_lb == nullptr ||
      globals->steer_rb == nullptr || globals->wheel_lf == nullptr || globals->wheel_rf == nullptr ||
      globals->wheel_lb == nullptr || globals->wheel_rb == nullptr) {
    return;
  }

  constexpr f32 kTwoPi = 2.0f * static_cast<f32>(M_PI);
  constexpr f32 kFrontDownYawAngle = 4.5166f;
  constexpr u16 kSteerInitEncoder[4]{1114, 3111, 2912, 2274};

  const auto &referee = globals->referee_data->data();
  auto &controller = globals->chassis_controller;
  rm::device::GM6020 *const steer_motors[4]{globals->steer_lf, globals->steer_rf, globals->steer_lb, globals->steer_rb};
  rm::device::M3508 *const wheel_motors[4]{globals->wheel_lf, globals->wheel_rf, globals->wheel_lb, globals->wheel_rb};

  freemaster_data.sample_count = freemaster_data.sample_count + 1U;

  freemaster_data.remote_speed_x = globals->gimbal_communicator->remote_speed_x();
  freemaster_data.remote_speed_y = globals->gimbal_communicator->remote_speed_y();
  freemaster_data.chassis_mode = globals->gimbal_communicator->chassis_mode();
  freemaster_data.ui_show_flag = globals->gimbal_communicator->UI_show_flag();
  freemaster_data.get_target_flag = globals->gimbal_communicator->get_target_flag();
  freemaster_data.suggest_fire_flag = globals->gimbal_communicator->suggest_fire_flag();
  freemaster_data.aim_speed_change = globals->gimbal_communicator->aim_speed_change();
  for (usize i = 0; i < 5; ++i) {
    freemaster_data.robot_hp[i] = globals->gimbal_communicator->robot_hp()[i];
  }

  freemaster_data.chassis_yaw_encoder = globals->yaw_motor->encoder();
  freemaster_data.chassis_yaw_angle_rad =
      rm::modules::Map(static_cast<f32>(freemaster_data.chassis_yaw_encoder), 0.0f, 8192.0f, 0.0f, kTwoPi);
  freemaster_data.chassis_yaw_delta_rad = rm::modules::Wrap(kFrontDownYawAngle - freemaster_data.chassis_yaw_angle_rad,
                                                            -static_cast<f32>(M_PI), static_cast<f32>(M_PI));

  freemaster_data.robot_id = referee.robot_status.robot_id;
  freemaster_data.robot_level = referee.robot_status.robot_level;
  freemaster_data.current_hp = referee.robot_status.current_HP;
  freemaster_data.maximum_hp = referee.robot_status.maximum_HP;
  freemaster_data.chassis_power_limit = referee.robot_status.chassis_power_limit;
  freemaster_data.buffer_energy = referee.power_heat_data.buffer_energy;
  freemaster_data.shooter_17mm_heat = referee.power_heat_data.shooter_17mm_1_barrel_heat;
  freemaster_data.shooter_heat_limit = referee.robot_status.shooter_barrel_heat_limit;
  freemaster_data.projectile_initial_speed = referee.shoot_data.initial_speed;
  freemaster_data.power_management_gimbal_output = referee.robot_status.power_management_gimbal_output;
  freemaster_data.power_management_chassis_output = referee.robot_status.power_management_chassis_output;
  freemaster_data.power_management_shooter_output = referee.robot_status.power_management_shooter_output;
  freemaster_data.projectile_allowance_17mm = referee.projectile_allowance.projectile_allowance_17mm;
  freemaster_data.remaining_gold_coin = referee.projectile_allowance.remaining_gold_coin;

  freemaster_data.chassis_move_state = static_cast<u8>(chassis->ChassisMove_);
  freemaster_data.buff_state = static_cast<u8>(chassis->buff_state_);
  freemaster_data.speed_mode = static_cast<u8>(chassis->speed_mode_);
  freemaster_data.supercap_energy = globals->super_cap->GetCapEnergy();
  freemaster_data.supercap_error_code = globals->super_cap->GetErrorCode();
  freemaster_data.supercap_referee_power_limit = globals->super_cap_tx.feedback_referee_power_limit;
  freemaster_data.supercap_referee_energy_buffer = globals->super_cap_tx.feedback_referee_energy_buffer;

  const f32 steer_command[4]{controller.output().lf_steer, controller.output().rf_steer, controller.output().lb_steer,
                             controller.output().rb_steer};
  const f32 wheel_command[4]{controller.output().lf_wheel, controller.output().rf_wheel, controller.output().lb_wheel,
                             controller.output().rb_wheel};
  for (usize i = 0; i < 4; ++i) {
    freemaster_data.steer_encoder[i] = steer_motors[i]->encoder();
    freemaster_data.steer_position_rad[i] = rm::modules::Map(
        static_cast<f32>(static_cast<i32>(freemaster_data.steer_encoder[i]) - static_cast<i32>(kSteerInitEncoder[i])),
        0.0f, 8191.0f, 0.0f, kTwoPi);
    freemaster_data.steer_rpm[i] = steer_motors[i]->rpm();
    freemaster_data.steer_feedback_current[i] = steer_motors[i]->current();
    freemaster_data.steer_command[i] = steer_command[i];
    freemaster_data.wheel_rpm[i] = wheel_motors[i]->rpm();
    freemaster_data.wheel_feedback_current[i] = wheel_motors[i]->current();
    freemaster_data.wheel_command[i] = wheel_command[i];
  }

  freemaster_data.chassis_target_vx = controller.target().vx;
  freemaster_data.chassis_target_vy = controller.target().vy;
  freemaster_data.chassis_target_w = controller.target().w;
}

#endif  // STEER_INFANTRY_CS_FREEMASTER_HPP
