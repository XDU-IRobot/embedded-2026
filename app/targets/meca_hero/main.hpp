#ifndef BOARDC_MAIN_HPP
#define BOARDC_MAIN_HPP

#include <librm.hpp>
#include "can.h"
#include "usart.h"
#include "spi.h"
#include "timer_task.hpp"
#include "tim.h"
/*-------------------------------------------------
 *变量
 */
inline struct GlobalWarehouse {
  // 硬件接口 //
  rm::hal::Can *can1{nullptr}, *can2{nullptr}; ///< CAN 总线接口
  rm::hal::Serial *dbus{nullptr}; ///< 遥控器串口接口

  // 设备 //
  rm::device::DR16 *rc{nullptr}; ///< 遥控器
  // rm::device::GM6020 *yaw_motor{nullptr};                                              ///< 云台 Yaw 电机
  // rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *magazine_motor{nullptr};  ///< 云台 Pitch 电机
  rm::device::BMI088 *imu{nullptr}; ///< BMI088 IMU

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

  // 控制器 //
  rm::modules::MahonyAhrs ahrs{1000.f}; ///< mahony 姿态解算器，频率 1000Hz

  void Init() {
    can1 = new rm::hal::Can{hcan1};
    can2 = new rm::hal::Can{hcan2};
    dbus = new rm::hal::Serial{huart3, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
    // 遥控
    rc = new rm::device::DR16{*dbus}; // 设置了遥控器以及用了串口
    // 电机
    chassis_motor_1 = new rm::device::M3508{*can2, 1, false};
    chassis_motor_2 = new rm::device::M3508{*can2, 2, false};
    chassis_motor_3 = new rm::device::M3508{*can2, 3, false};
    chassis_motor_4 = new rm::device::M3508{*can2, 4, false};

    shooter_motor_1 = new rm::device::M3508{*can1, 1, false};
    shooter_motor_2 = new rm::device::M3508{*can1, 2, false};
    shooter_motor_3 = new rm::device::M3508{*can1, 3, false};
    shooter_motor_4 = new rm::device::M3508{*can1, 4, false};
    shooter_motor_5 = new rm::device::M3508{*can1, 5, false};
    shooter_motor_6 = new rm::device::M3508{*can1, 6, false};

    magazine_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
        *can1, {0x12, 0x03, 3.141593f, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}}};

    // PID控制器
    pid_chassis_1 = new rm::modules::PID{20, 2, 4, 18000, 2};
    pid_chassis_2 = new rm::modules::PID{20, 2, 4, 18000, 2};
    pid_chassis_3 = new rm::modules::PID{25, 2, 4, 18000, 2};
    pid_chassis_4 = new rm::modules::PID{25, 2, 4, 18000, 2};

    pid_shooter_1 = new rm::modules::PID{25, 2, 4, 18000, 2}; //20
    pid_shooter_2 = new rm::modules::PID{25, 2, 4, 18000, 2}; //20
    pid_shooter_3 = new rm::modules::PID{25, 2, 4, 18000, 2};
    pid_shooter_4 = new rm::modules::PID{25, 2, 4, 18000, 2};
    pid_shooter_5 = new rm::modules::PID{25, 2, 4, 18000, 2};
    pid_shooter_6 = new rm::modules::PID{25, 2, 4, 18000, 2};

    pid_magz_position = new rm::modules::PID{2.25, 0.001, 0.149, 6, 0};
    // pid_magz_velocity = new rm::modules::PID{0.17, 0, 0.0002, 6.4, 0};

    can1->SetFilter(0, 0);
    can1->Begin();
    can2->SetFilter(0, 0);
    can2->Begin();
    rc->Begin(); // 启动遥控器接收，这行或许比较适合放到AppMain里面？
  }
} *globals;;

// 底盘速度
inline rm::i16 Vx, Vy, Vw;
//拨盘增加角度
inline float target_magz;
inline float target_velocity;
//左摇杆状态
inline rm::device::DR16::SwitchPosition l_switch_position_now = rm::device::DR16::SwitchPosition::kUnknown;
inline rm::device::DR16::SwitchPosition l_switch_position_last = rm::device::DR16::SwitchPosition::kUnknown;
//拨盘反馈值
inline float pos;
inline float vel;
//扳机计数
inline int counter = 0;
//摩擦轮速度
inline rm::i16 V_shooter_1=-6000;
inline rm::i16 V_shooter_2=-5500;

/*----------------------------------------------
 *执行函数
 */
//拨盘电机逻辑
void MagazineControl();
//摩擦轮电机逻辑
void ShooterControl();
//底盘逻辑
void ChassisControl();
#endif  // BOARDC_MAIN_HPP
