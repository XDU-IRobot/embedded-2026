#pragma once

#include <cstdint>

#include "Gimbal.hpp"

/**
 * @brief FreeMASTER snapshot of one PID controller.
 *
 * The fields are copied at 100 Hz so FreeMASTER only needs to read simple
 * scalar globals instead of following C++ objects and pointers.
 */
struct FreemasterPidData {
  float set{};       ///< PID 目标值；位置环为 rad，速度环为 rad/s
  float feedback{};  ///< PID 反馈值；位置环为 rad，速度环为 rad/s
  float error{};     ///< 当前误差 set - feedback
  float p_out{};     ///< 比例项输出
  float i_out{};     ///< 积分项输出
  float d_out{};     ///< 微分项输出
  float out{};       ///< P、I、D 求和并限幅后的 PID 输出
};

/**
 * @brief Target-local telemetry exported to FreeMASTER.
 *
 * Add this global symbol as `freemaster_data` in FreeMASTER. All fields are
 * read-only telemetry snapshots from the control code.
 */
struct FreemasterData {
  std::uint32_t update_count{};  ///< 100 Hz 快照累计更新次数，可用于检查刷新是否正常

  struct {
    std::uint8_t robot_state{};   ///< 整机状态，对应 StateMachineType
    std::uint8_t gimbal_mode{};   ///< 云台运动模式，对应 Gimbal::GimbalMove_
    std::uint8_t rc_online{};     ///< DT17 在线标志：0 离线，1 在线
    std::uint8_t vt03_online{};   ///< VT03 在线标志：0 离线，1 在线
    std::uint8_t remote_source{}; ///< 当前控制源：0 无，1 DT17，2 VT03
    std::uint8_t aimbot_online{}; ///< 自瞄通信器在线标志：0 离线，1 在线
    std::uint8_t gimbal_online{}; ///< 云台电机在线标志：0 异常，1 全部在线
    std::uint8_t shoot_online{};  ///< 发射机构电机在线标志：0 异常，1 全部在线
    float fire_delay_avg_ms{};    ///< 平均开火延迟，单位 ms
  } runtime;                      ///< 运行状态与设备在线状态

  struct {
    float yaw{};                   ///< 最终 Yaw 位置目标，单位 rad
    float pitch{};                 ///< 最终 Pitch 位置目标，单位 rad
    float yaw_speed{};             ///< Yaw 速度参考，单位 rad/s
    float pitch_speed{};           ///< Pitch 速度参考，单位 rad/s
    float yaw_accel{};             ///< Yaw 加速度参考，单位 rad/s^2
    float pitch_accel{};           ///< Pitch 加速度参考，单位 rad/s^2
    float yaw_speed_feedforward{}; ///< 实际送入控制器的 Yaw 速度前馈，单位 rad/s
  } gimbal_target;                 ///< 云台控制目标与轨迹参考量

  struct {
    float yaw{};                       ///< AHRS Yaw 姿态角，单位 rad
    float pitch{};                     ///< AHRS Pitch 姿态角，单位 rad
    float roll{};                      ///< AHRS Roll 姿态角，单位 rad
    float gyro_x{};                    ///< IMU X 轴角速度，单位 rad/s
    float gyro_y{};                    ///< IMU Y 轴角速度，单位 rad/s
    float gyro_z{};                    ///< IMU Z 轴角速度，单位 rad/s
    float controller_yaw_position{};   ///< Yaw 位置环实际使用的反馈，单位 rad
    float controller_yaw_speed{};      ///< Yaw 速度环实际使用的反馈，单位 rad/s
    float controller_pitch_position{}; ///< Pitch 位置环实际使用的反馈，单位 rad
    float controller_pitch_speed{};    ///< Pitch 速度环实际使用的反馈，单位 rad/s
    float yaw_motor_rpm{};             ///< GM6020 Yaw 电机反馈转速，单位 rpm
    float yaw_motor_current{};         ///< GM6020 Yaw 电机回传电流原始值
    float pitch_motor_position{};      ///< DM Pitch 电机位置，单位 rad
    float pitch_motor_speed{};         ///< DM Pitch 电机速度，单位 rad/s
    float pitch_motor_torque{};        ///< DM Pitch 电机回传转矩，单位 N*m
  } gimbal_feedback;                   ///< 姿态、控制器反馈和电机反馈

  struct {
    std::int16_t left_x{};       ///< 遥控器左摇杆横向原始值，范围约 -660~660
    std::int16_t left_y{};       ///< 遥控器左摇杆纵向原始值，范围约 -660~660
    std::int16_t right_x{};      ///< 遥控器右摇杆横向原始值，范围约 -660~660
    std::int16_t right_y{};      ///< 遥控器右摇杆纵向原始值，范围约 -660~660
    std::int16_t dial{};         ///< 遥控器拨轮原始值，范围约 -660~660
    std::int16_t mouse_x{};      ///< 鼠标 X 轴输入原始值
    std::int16_t mouse_y{};      ///< 鼠标 Y 轴输入原始值
    std::int16_t mouse_z{};      ///< 鼠标 Z 轴输入原始值
    std::uint16_t keyboard{};    ///< 键盘按键位图，各位定义对应 DR16::Key
    std::uint8_t switch_l{};     ///< 左拨杆：0 未知，1 上，2 下，3 中
    std::uint8_t switch_r{};     ///< 右拨杆：0 未知，1 上，2 下，3 中
    std::uint8_t mouse_left{};   ///< 鼠标左键：0 松开，1 按下
    std::uint8_t mouse_right{};  ///< 鼠标右键：0 松开，1 按下
  } remote;                      ///< DR16 遥控器接收数据

  struct {
    std::uint8_t state{};        ///< 自瞄状态位：bit0 有目标，bit1 建议开火
    std::uint8_t target{};       ///< NUC 下发的目标编号/类型原始值
    std::uint8_t nuc_start{};    ///< NUC 启动标志原始值
    float yaw{};                 ///< NUC 下发的 Yaw 目标角，单位 rad
    float pitch{};               ///< NUC 下发的 Pitch 目标角，单位 rad
    float yaw_velocity{};        ///< NUC 下发的 Yaw 目标速度，单位 rad/s
    float pitch_velocity{};      ///< NUC 下发的 Pitch 目标速度，单位 rad/s
    float yaw_acceleration{};    ///< NUC 下发的 Yaw 目标加速度，单位 rad/s^2
    float pitch_acceleration{};  ///< NUC 下发的 Pitch 目标加速度，单位 rad/s^2
  } aimbot_rx;                   ///< 云台自瞄 CAN 通信器接收数据

  struct {
    std::uint16_t heat_real{};   ///< 底盘回传的当前枪口热量
    std::uint16_t heat_limit{};  ///< 底盘回传的枪口热量上限
    float ammo_speed{};          ///< 底盘回传的裁判系统弹速，单位 m/s
    std::uint8_t robot_id{};     ///< 底盘回传的机器人阵营/编号选择位
    std::uint8_t gimbal_power{}; ///< 裁判系统云台供电状态：0 关闭，1 开启
    std::uint8_t chassis_power{}; ///< 裁判系统底盘供电状态：0 关闭，1 开启
    std::uint8_t ammo_power{};   ///< 裁判系统发射机构供电状态：0 关闭，1 开启
  } chassis_rx;                  ///< 底盘 CAN 通信器接收数据

  struct {
    FreemasterPidData yaw_position;   ///< Yaw 外环（位置环）PID 数据
    FreemasterPidData yaw_speed;      ///< Yaw 内环（速度环）PID 数据
    FreemasterPidData pitch_position; ///< Pitch 外环（位置环）PID 数据
    FreemasterPidData pitch_speed;    ///< Pitch 内环（速度环）PID 数据
    float yaw_controller_out{};       ///< Yaw 串级 PID 最终输出，GM6020 指令量纲
    float pitch_controller_out{};     ///< Pitch 串级 PID 最终输出，单位 N*m
    float yaw_feedforward_torque{};   ///< 动力学模型计算的 Yaw 前馈转矩，单位 N*m
    float yaw_motor_command{};        ///< 限幅后的 GM6020 Yaw 最终电压指令
    float pitch_motor_command{};      ///< 限幅后的 DM Pitch 最终转矩指令，单位 N*m
  } gimbal_output;                    ///< 云台四个 PID、前馈及最终执行指令
};

inline volatile FreemasterData freemaster_data{};  ///< FreeMASTER 读取的 100 Hz 全局数据快照

/**
 * @brief Refresh all FreeMASTER telemetry.
 *
 * Called by GlobalWarehouse::SubLoop100Hz() after device-state updates.
 */
inline void freemaster() {
  if (globals == nullptr || gimbal == nullptr || globals->rc == nullptr || globals->imu == nullptr ||
      globals->yaw_motor == nullptr || globals->pitch_motor == nullptr || globals->aimbot_communicator == nullptr ||
      globals->chassis_communicator == nullptr) {
    return;
  }

  const auto copy_pid = [](volatile FreemasterPidData &destination, const rm::modules::PID &source) {
    destination.set = source.set();
    destination.feedback = source.ref()[0];
    destination.error = source.error()[0];
    destination.p_out = source.p_out();
    destination.i_out = source.i_out();
    destination.d_out = source.d_out()[0];
    destination.out = source.out();
  };

  freemaster_data.update_count = freemaster_data.update_count + 1U;

  freemaster_data.runtime.robot_state = static_cast<std::uint8_t>(globals->StateMachine_);
  freemaster_data.runtime.gimbal_mode = static_cast<std::uint8_t>(gimbal->GimbalMove_);
  freemaster_data.runtime.rc_online = static_cast<std::uint8_t>(globals->device_rc.all_device_ok());
  freemaster_data.runtime.vt03_online = static_cast<std::uint8_t>(globals->device_referee.all_device_ok());
  freemaster_data.runtime.remote_source = static_cast<std::uint8_t>(globals->remote_source);
  freemaster_data.runtime.aimbot_online = static_cast<std::uint8_t>(globals->device_nuc.all_device_ok());
  freemaster_data.runtime.gimbal_online = static_cast<std::uint8_t>(globals->device_gimbal.all_device_ok());
  freemaster_data.runtime.shoot_online = static_cast<std::uint8_t>(globals->device_shoot.all_device_ok());
  freemaster_data.runtime.fire_delay_avg_ms = AdelayTime;

  freemaster_data.gimbal_target.yaw = gimbal->yaw_target();
  freemaster_data.gimbal_target.pitch = gimbal->pitch_target();
  freemaster_data.gimbal_target.yaw_speed = gimbal->yaw_speed_reference();
  freemaster_data.gimbal_target.pitch_speed = gimbal->pitch_speed_reference();
  freemaster_data.gimbal_target.yaw_accel = gimbal->yaw_accel_reference();
  freemaster_data.gimbal_target.pitch_accel = gimbal->pitch_accel_reference();
  freemaster_data.gimbal_target.yaw_speed_feedforward = globals->gimbal_controller.target().yaw_speed_ff;

  freemaster_data.gimbal_feedback.yaw = globals->ahrs.euler_angle().yaw;
  freemaster_data.gimbal_feedback.pitch = globals->ahrs.euler_angle().pitch;
  freemaster_data.gimbal_feedback.roll = globals->ahrs.euler_angle().roll;
  freemaster_data.gimbal_feedback.gyro_x = globals->imu->gyro_x();
  freemaster_data.gimbal_feedback.gyro_y = globals->imu->gyro_y();
  freemaster_data.gimbal_feedback.gyro_z = globals->imu->gyro_z();
  freemaster_data.gimbal_feedback.controller_yaw_position = globals->gimbal_controller.state().yaw_position;
  freemaster_data.gimbal_feedback.controller_yaw_speed = globals->gimbal_controller.state().yaw_speed;
  freemaster_data.gimbal_feedback.controller_pitch_position = globals->gimbal_controller.state().pitch_position;
  freemaster_data.gimbal_feedback.controller_pitch_speed = globals->gimbal_controller.state().pitch_speed;
  freemaster_data.gimbal_feedback.yaw_motor_rpm = static_cast<float>(globals->yaw_motor->rpm());
  freemaster_data.gimbal_feedback.yaw_motor_current = static_cast<float>(globals->yaw_motor->current());
  freemaster_data.gimbal_feedback.pitch_motor_position = globals->pitch_motor->pos();
  freemaster_data.gimbal_feedback.pitch_motor_speed = globals->pitch_motor->vel();
  freemaster_data.gimbal_feedback.pitch_motor_torque = globals->pitch_motor->tau();

  freemaster_data.remote.left_x = globals->rc->left_x();
  freemaster_data.remote.left_y = globals->rc->left_y();
  freemaster_data.remote.right_x = globals->rc->right_x();
  freemaster_data.remote.right_y = globals->rc->right_y();
  freemaster_data.remote.dial = globals->rc->dial();
  freemaster_data.remote.mouse_x = globals->rc->mouse_x();
  freemaster_data.remote.mouse_y = globals->rc->mouse_y();
  freemaster_data.remote.mouse_z = globals->rc->mouse_z();
  std::uint16_t keyboard = 0U;
  for (std::uint16_t bit = 0U; bit < 16U; ++bit) {
    const auto key = static_cast<rm::device::DR16::Key>(1U << bit);
    if (globals->rc->key(key)) {
      keyboard |= static_cast<std::uint16_t>(1U << bit);
    }
  }
  freemaster_data.remote.keyboard = keyboard;
  freemaster_data.remote.switch_l = static_cast<std::uint8_t>(globals->rc->switch_l());
  freemaster_data.remote.switch_r = static_cast<std::uint8_t>(globals->rc->switch_r());
  freemaster_data.remote.mouse_left = static_cast<std::uint8_t>(globals->rc->mouse_button_left());
  freemaster_data.remote.mouse_right = static_cast<std::uint8_t>(globals->rc->mouse_button_right());

  freemaster_data.aimbot_rx.state = globals->aimbot_communicator->aimbot_state();
  freemaster_data.aimbot_rx.target = globals->aimbot_communicator->aimbot_target();
  freemaster_data.aimbot_rx.nuc_start = globals->aimbot_communicator->nuc_start_flag();
  freemaster_data.aimbot_rx.yaw = globals->aimbot_communicator->yaw();
  freemaster_data.aimbot_rx.pitch = globals->aimbot_communicator->pitch();
  freemaster_data.aimbot_rx.yaw_velocity = globals->aimbot_communicator->yaw_vel();
  freemaster_data.aimbot_rx.pitch_velocity = globals->aimbot_communicator->pitch_vel();
  freemaster_data.aimbot_rx.yaw_acceleration = globals->aimbot_communicator->yaw_acc();
  freemaster_data.aimbot_rx.pitch_acceleration = globals->aimbot_communicator->pitch_acc();

  freemaster_data.chassis_rx.heat_real = globals->chassis_communicator->heat_real();
  freemaster_data.chassis_rx.heat_limit = globals->chassis_communicator->heat_limit();
  freemaster_data.chassis_rx.ammo_speed = globals->chassis_communicator->ammo_speed();
  freemaster_data.chassis_rx.robot_id = globals->chassis_communicator->robot_id();
  freemaster_data.chassis_rx.gimbal_power = globals->chassis_communicator->gimbal_power_state();
  freemaster_data.chassis_rx.chassis_power = globals->chassis_communicator->chassis_power_state();
  freemaster_data.chassis_rx.ammo_power = globals->chassis_communicator->ammo_power_state();

  auto &gimbal_pid = globals->gimbal_controller.pid();
  copy_pid(freemaster_data.gimbal_output.yaw_position, gimbal_pid.yaw_position);
  copy_pid(freemaster_data.gimbal_output.yaw_speed, gimbal_pid.yaw_speed);
  copy_pid(freemaster_data.gimbal_output.pitch_position, gimbal_pid.pitch_position);
  copy_pid(freemaster_data.gimbal_output.pitch_speed, gimbal_pid.pitch_speed);
  freemaster_data.gimbal_output.yaw_controller_out = globals->gimbal_controller.output().yaw;
  freemaster_data.gimbal_output.pitch_controller_out = globals->gimbal_controller.output().pitch;
  freemaster_data.gimbal_output.yaw_feedforward_torque = gimbal->yaw_feedforward_torque();
  freemaster_data.gimbal_output.yaw_motor_command = gimbal->yaw_command();
  freemaster_data.gimbal_output.pitch_motor_command = gimbal->pitch_command();
}
