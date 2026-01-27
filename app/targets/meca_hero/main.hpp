#ifndef BOARDC_MAIN_HPP
#define BOARDC_MAIN_HPP

#include <librm.hpp>
#include "can.h"
#include "usart.h"
#include "spi.h"
#include "timer_task.hpp"
/*-------------------------------------------------
 *变量
 */
inline struct GlobalWarehouse {
  // 硬件接口 //
  rm::hal::Can *can1{nullptr}, *can2{nullptr};  ///< CAN 总线接口
  rm::hal::Serial *dbus{nullptr};               ///< 遥控器串口接口

  // 设备 //
  rm::device::DR16 *rc{nullptr};  ///< 遥控器
  // rm::device::GM6020 *yaw_motor{nullptr};                                              ///< 云台 Yaw 电机
  // rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *magazine_motor{nullptr};  ///< 云台 Pitch 电机
  rm::device::BMI088 *imu{nullptr};  ///< BMI088 IMU

  // 创建电机对象
  rm::device::M3508 *chassis_motor_1{nullptr};
  rm::device::M3508 *chassis_motor_2{nullptr};
  rm::device::M3508 *chassis_motor_3{nullptr};
  rm::device::M3508 *chassis_motor_4{nullptr};

  rm::device::M3508 *shooter_motor_1{nullptr};
  rm::device::M3508 *shooter_motor_2{nullptr};
  rm::device::M3508 *shooter_motor_3{nullptr};
  rm::device::M3508 *shooter_motor_4{nullptr};
  rm::device::M3508 *shooter_motor_5{nullptr};
  rm::device::M3508 *shooter_motor_6{nullptr};

  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *magazine_motor{nullptr};

  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *gimbal_motor_yaw{nullptr};
  rm::device::M3508 *gimbal_motor_pitch{nullptr};
  // 创建PID控制器
  rm::modules::PID *pid_chassis_1{nullptr};
  rm::modules::PID *pid_chassis_2{nullptr};
  rm::modules::PID *pid_chassis_3{nullptr};
  rm::modules::PID *pid_chassis_4{nullptr};

  rm::modules::PID *pid_shooter_1{nullptr};
  rm::modules::PID *pid_shooter_2{nullptr};
  rm::modules::PID *pid_shooter_3{nullptr};
  rm::modules::PID *pid_shooter_4{nullptr};
  rm::modules::PID *pid_shooter_5{nullptr};
  rm::modules::PID *pid_shooter_6{nullptr};

  rm::modules::PID *pid_magz_position{nullptr};
  rm::modules::PID *pid_magz_velocity{nullptr};

  rm::modules::PID *pid_yaw_position{nullptr};
  rm::modules::PID *pid_yaw_velocity{nullptr};
  rm::modules::PID *pid_pitch_position{nullptr};
  rm::modules::PID *pid_pitch_velocity{nullptr};

  // 底盘随动
  rm::modules::PID *pid_chassis_follow{nullptr};

  // 控制器 //
  rm::modules::MahonyAhrs ahrs{1000.0f};  ///< mahony 姿态解算器，频率 1000Hz

  void Init() {
    can1 = new rm::hal::Can{hcan1};
    can2 = new rm::hal::Can{hcan2};
    dbus = new rm::hal::Serial{huart3, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
    // 遥控
    rc = new rm::device::DR16{*dbus};  // 设置了遥控器以及用了串口
    // IMU
    imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
    /*------*/
    // 电机
    chassis_motor_1 = new rm::device::M3508{*can2, 1, false};
    chassis_motor_2 = new rm::device::M3508{*can2, 2, false};
    chassis_motor_3 = new rm::device::M3508{*can2, 3, false};
    chassis_motor_4 = new rm::device::M3508{*can2, 4, false};
    // 摩擦轮电机
    shooter_motor_1 = new rm::device::M3508{*can1, 1, false};
    shooter_motor_2 = new rm::device::M3508{*can1, 2, false};
    shooter_motor_3 = new rm::device::M3508{*can1, 3, false};
    shooter_motor_4 = new rm::device::M3508{*can1, 4, false};
    shooter_motor_5 = new rm::device::M3508{*can1, 5, false};
    shooter_motor_6 = new rm::device::M3508{*can1, 6, false};
    // 拨盘电机
    magazine_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
        *can1, {0x12, 0x03, 3.141593f, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}}};
    // 云台电机
    gimbal_motor_yaw = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
        *can2, {0x12, 0x02, 3.141593f, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}}, true};
    gimbal_motor_pitch = new rm::device::M3508{*can2, 5, true};
    /*--------*/
    // PID控制器
    pid_chassis_1 = new rm::modules::PID{20, 2, 4, 18000, 2};
    pid_chassis_2 = new rm::modules::PID{20, 2, 4, 18000, 2};
    pid_chassis_3 = new rm::modules::PID{20, 2, 4, 18000, 2};
    pid_chassis_4 = new rm::modules::PID{20, 2, 4, 18000, 2};

    pid_shooter_1 = new rm::modules::PID{25, 2, 4, 10000, 2};  // 20
    pid_shooter_2 = new rm::modules::PID{25, 2, 4, 10000, 2};  // 20
    pid_shooter_3 = new rm::modules::PID{25, 2, 4, 10000, 2};
    pid_shooter_4 = new rm::modules::PID{25, 2, 4, 10000, 2};
    pid_shooter_5 = new rm::modules::PID{25, 2, 4, 10000, 2};
    pid_shooter_6 = new rm::modules::PID{25, 2, 4, 10000, 2};

    pid_magz_position = new rm::modules::PID{19, 0.001, 0.4, 6, 0};
    // pid_magz_velocity = new rm::modules::PID{0.17, 0, 0.0002, 6.4, 0};

    pid_yaw_position = new rm::modules::PID{80, 0.01, 8, 6.5, 0};
    pid_yaw_velocity = new rm::modules::PID{1, 0, 0.001, 6, 0};
    pid_pitch_position = new rm::modules::PID{11, 0, 0, 1, 0};
    pid_pitch_velocity = new rm::modules::PID{5000, 15, 0, 8000, 2000};

    // 底盘随动
    pid_chassis_follow = new rm::modules::PID{11000, 0, 100, 10000, 0};

    can1->SetFilter(0, 0);
    can1->Begin();
    can2->SetFilter(0, 0);
    can2->Begin();
    rc->Begin();  // 启动遥控器接收，这行或许比较适合放到AppMain里面？
  }
} *globals;
;

// 底盘速度
inline rm::i16 Vx, Vy, Vw;
// 云台角度
inline float target_pos_yaw, target_pos_pitch;
// 云台当前角度
inline float eulerangle_yaw, eulerangle_pitch, eulerangle_roll;
// imu陀螺仪
inline float Gy, Gz, Gx;
// 拨盘增加角度
inline float target_magz = 0;
inline float target_velocity;
// 左摇杆状态
inline rm::device::DR16::SwitchPosition l_switch_position_now = rm::device::DR16::SwitchPosition::kUnknown;
inline rm::device::DR16::SwitchPosition l_switch_position_last = rm::device::DR16::SwitchPosition::kUnknown;
// 右摇杆状态
inline rm::device::DR16::SwitchPosition r_switch_position_now = rm::device::DR16::SwitchPosition::kUnknown;
inline rm::device::DR16::SwitchPosition r_switch_position_last = rm::device::DR16::SwitchPosition::kUnknown;
// 拨盘反馈值
inline float pos;
inline float vel;
// 扳机计数
inline int counter = 0;
// 摩擦轮速度
inline rm::i16 V_shooter_1 = -600;
inline rm::i16 V_shooter_2 = -550;
// 摩擦轮速度监测
inline rm::i16 shooter_1;
inline rm::i16 shooter_2;
inline rm::i16 shooter_3;
inline rm::i16 shooter_4;
inline rm::i16 shooter_5;
inline rm::i16 shooter_6;
// 底盘速度监测
inline rm::i16 chassis_1;
inline rm::i16 chassis_2;
inline rm::i16 chassis_3;
inline rm::i16 chassis_4;
// PIDerror
inline float error;
// pitch_out
inline float pitch_out;
inline float yaw_out;
// 电机状态
inline uint8_t yaw_state;
// 拨盘补偿标志
inline bool magz_compensation_flag{false};
inline int magz_compensation_count{0};
inline float magz_compensation = 0;
/*----------------------------------------------
 *执行函数
 */
// 拨盘电机逻辑
void MagazineControl();
// 摩擦轮电机逻辑
void ShooterControl();
// 底盘逻辑
void ChassisControl();
// 云台逻辑
void GimbalControl();
#endif  // BOARDC_MAIN_HPP