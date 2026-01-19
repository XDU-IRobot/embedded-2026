//
// Created by CHIPS KILLER on 2026/1/13.
//

#ifndef BOARDC_MAIN_HPP
#define BOARDC_MAIN_HPP

#include <librm.hpp>
#include "can.h"
#include "usart.h"
#include "spi.h"
#include "timer_task.hpp"
#include "tim.h"

// //底盘速度
//   rm::i16 Vx,Vy,Vw;
//
// struct GlobalWarehouse {
//
//   // 硬件接口 //
//   rm::hal::Can *can1{nullptr}, *can2{nullptr};  ///< CAN 总线接口
//   rm::hal::Serial *dbus{nullptr};               ///< 遥控器串口接口
//
//   // 设备 //
//   rm::device::DR16 *rc{nullptr};           ///< 遥控器
//   rm::device::GM6020 *yaw_motor{nullptr};  ///< 云台 Yaw 电机
//   rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *magazine_motor{nullptr};  ///< 云台 Pitch 电机
//   rm::device::BMI088 *imu{nullptr};///< BMI088 IMU
//
//   //创建电机对象
//   rm::device::M3508 *wheel_motor_1{nullptr};
//   rm::device::M3508 *wheel_motor_2{nullptr};
//   rm::device::M3508 *wheel_motor_3{nullptr};
//   rm::device::M3508 *wheel_motor_4{nullptr};
//
//   //创建PID控制器
//   rm::modules::PID *pid_1{nullptr};
//   rm::modules::PID *pid_2{nullptr};
//   rm::modules::PID *pid_3{nullptr};
//   rm::modules::PID *pid_4{nullptr};
//
//   // 控制器 //
//   rm::modules::MahonyAhrs ahrs{1000.f};  ///< mahony 姿态解算器，频率 1000Hz
//
//   void Init() {
//
//     can1 = new rm::hal::Can{hcan1};
//     can2 = new rm::hal::Can{hcan2};
//     dbus = new rm::hal::Serial{huart3, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
//     //遥控
//     rc = new rm::device::DR16{*dbus};//设置了遥控器以及用了串口
//     //电机
//     wheel_motor_1=new rm::device::M3508{*can1,1,false};
//     wheel_motor_2=new rm::device::M3508{*can1,2,false};
//     wheel_motor_3=new rm::device::M3508{*can1,3,false};
//     wheel_motor_4=new rm::device::M3508{*can1,4,false};
//     // yaw_motor = new rm::device::GM6020{*can1, 1};
//     // magazine_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{*can2, {}};
//     // imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
//     //PID控制器
//     pid_1=new rm::modules::PID{10,2,2,30000,2};
//     pid_2=new rm::modules::PID{10,2,2,30000,2};
//     pid_3=new rm::modules::PID{10,2,2,30000,2};
//     pid_4=new rm::modules::PID{10,2,2,30000,2};
//
//
//
//     can1->SetFilter(0, 0);
//     can1->Begin();
//     can2->SetFilter(0, 0);
//     can2->Begin();
//     rc->Begin();  // 启动遥控器接收，这行或许比较适合放到AppMain里面？
//   }
// } *globals;

// //创建电机对象
// rm::device::M3508 wheel_motor_1(*(globals->can1),1,false);
// rm::device::M3508 wheel_motor_2(*(globals->can1),2,false);
// rm::device::M3508 wheel_motor_3(*(globals->can1),3,false);
// rm::device::M3508 wheel_motor_4(*(globals->can1),4,false);

// 创建PID控制器
//  rm::modules::PID pid_1(5,0,0,30000,0);
//  rm::modules::PID pid_2(5,0,0,30000,0);
//  rm::modules::PID pid_3(5,0,0,30000,0);
//  rm::modules::PID pid_4(5,0,0,30000,0);

#endif  // BOARDC_MAIN_HPP
