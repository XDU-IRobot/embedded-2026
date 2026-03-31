#ifndef BOARDC_MAIN_HPP
#define BOARDC_MAIN_HPP

#include <librm.hpp>
#include "can.h"
#include "usart.h"
#include "spi.h"
#include "timer_task.hpp"
#include "buzzer_controller.hpp"
#include "usbd_cdc_if.h"
#include "VOFA.hpp"
#include "aimbot_comm_can.hpp"
#include "CustomClient.hpp"
#include "State.hpp"
#include "CMS.H"
/*-------------------------------------------------
 *变量
 */
inline float follow_d = 0;

inline struct GlobalWarehouse {
  // 硬件接口 //
  rm::hal::Can *can1{nullptr}, *can2{nullptr};                       ///< CAN 总线接口
  rm::hal::Serial *dbus{nullptr}, *uart6{nullptr}, *uart1{nullptr};  ///< 遥控器串口接口

  // 设备 //
  rm::device::DR16 *rc{nullptr};  ///< 遥控器
  rm::device::VT03 *tc{nullptr};  // 图传遥控器
  // rm::device::GM6020 *yaw_motor{nullptr};                                              ///< 云台 Yaw 电机
  // rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *magazine_motor{nullptr};  ///< 云台 Pitch 电机
  rm::device::BMI088 *imu{nullptr};  ///< BMI088 IMU
  rm::device::AimbotCanCommunicator *aimbot_can_communicator{nullptr};
  rm::device::CustomClient *custom_client{nullptr};
  CMS *cms{nullptr};
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
  rm::modules::PID *pid_chassis_follow_pos{nullptr};
  rm::modules::PID *pid_chassis_follow_vel{nullptr};
  // 控制器 //
  rm::modules::MahonyAhrs ahrs{840.0f};  ///< mahony 姿态解算器，频率 1000Hz 840.0
  // 底盘功率检测
  rm::device::M3508 *chassis_motor[4] = {nullptr, nullptr, nullptr, nullptr};
  rm::modules::PID *velocity_pids[4] = {nullptr, nullptr, nullptr, nullptr};
  std::array<rm::modules::M3508PowerModel::MotorState, 4> *motor_states{nullptr};
  // 裁判系统
  rm::device::Referee<rm::device::RefereeRevision::kNewV110> ref;
  uint8_t rx_buffer[128]{0};

  rm::modules::LowPassFilterConstDt<float> gyro_z_filter;

  bool ui_send_choice{false};

  void Init() {
    can1 = new rm::hal::Can{hcan1};
    can2 = new rm::hal::Can{hcan2};
    dbus = new rm::hal::Serial{huart3, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
    uart6 = new rm::hal::Serial{huart6, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
    uart1 = new rm::hal::Serial{huart1, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
    aimbot_can_communicator = new rm::device::AimbotCanCommunicator{*can1};
    custom_client = new rm::device::CustomClient;
    cms = new CMS{*can2};
    // 遥控
    rc = new rm::device::DR16{*dbus};  // 设置了遥控器以及串口
    tc = new rm::device::VT03;
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
    pid_chassis_1 = new rm::modules::PID{20, 2, 4, 15000, 2};
    pid_chassis_2 = new rm::modules::PID{20, 2, 4, 15000, 2};
    pid_chassis_3 = new rm::modules::PID{40, 2, 4, 15000, 100};
    pid_chassis_4 = new rm::modules::PID{40, 2, 4, 15000, 100};

    pid_shooter_1 = new rm::modules::PID{30, 0.000001, 0, 10000, 1600};  // 20
    pid_shooter_2 = new rm::modules::PID{30, 0.000001, 0, 10000, 1600};  // 20
    pid_shooter_3 = new rm::modules::PID{30, 0.000001, 0, 10000, 1600};
    pid_shooter_4 = new rm::modules::PID{30, 0.000001, 0, 10000, 1600};
    pid_shooter_5 = new rm::modules::PID{30, 0.000001, 0, 10000, 1600};
    pid_shooter_6 = new rm::modules::PID{30, 0.000001, 0, 10000, 1600};

    // pid_magz_position = new rm::modules::PID{19, 0.001, 0.4, 6, 0};
    pid_magz_position = new rm::modules::PID{42, 0.001, 0.56, 16, 0};
    pid_magz_velocity = new rm::modules::PID{0.505, 0, 0.00002, 7, 0};

    // pid_yaw_position = new rm::modules::PID{60, 0.01, 3, 6, 0};
    // pid_yaw_velocity = new rm::modules::PID{1, 0, 0.001, 6, 0};
    pid_yaw_position = new rm::modules::PID{30, 0, 0, 10, 0};
    pid_yaw_velocity = new rm::modules::PID{11, 0, 0.03, 6, 0};
    pid_pitch_position = new rm::modules::PID{60, 0.5, 1.3, 1, 0.1};
    pid_pitch_velocity = new rm::modules::PID{9100, 3500, 40, 16000, 500};
    // pid_pitch_position = new rm::modules::PID{2000, 0, 0, 1500, 1000};
    // pid_pitch_velocity = new rm::modules::PID{100, 0, 0, 16000, 5000};

    // 底盘随动

    pid_chassis_follow_pos = new rm::modules::PID{600, 0, 0, 1500, 0};
    // pid_chassis_follow_pos = new rm::modules::PID{5000, 0, 100, 5000, 0};
    // pid_chassis_follow_pos = new rm::modules::PID{14000, 33600, 100, 16000, 10000};

    // pid_chassis_follow = new rm::modules::PID{19000, 5000, 210, 16000, 10000};

    pid_chassis_follow_vel = new rm::modules::PID{23, 0, 0.21, 16000, 0};
    // pid_chassis_follow_vel = new rm::modules::PID{500, 0, 0, 8000, 0};
    // 底盘电机
    for (int i = 0; i < 4; i++) {
      chassis_motor[i] = new rm::device::M3508(*can2, i + 1);
    }
    // 底盘功率检测
    for (int i = 0; i < 2; i++) {
      velocity_pids[i] = new rm::modules::PID(10, 0.5, 0, 16384, 5000);
    }
    for (int i = 2; i < 4; i++) {
      velocity_pids[i] = new rm::modules::PID(20, 0.5, 0, 16384, 5000);
    }
    motor_states = new std::array<rm::modules::M3508PowerModel::MotorState, 4>();

    can1->SetFilter(0, 0);
    can1->Begin();
    can2->SetFilter(0, 0);
    can2->Begin();
    rc->Begin();  // 启动遥控器接收，这行或许比较适合放到AppMain里面？
  }
} *globals;

// 底盘速度
inline rm::i16 Vx, Vy, Vw;
// 云台角度
inline float target_pos_yaw, last_target_pos_yaw, target_pos_pitch;
// 云台当前角度
inline float eulerangle_yaw, eulerangle_pitch, eulerangle_roll;
// imu陀螺仪
inline float Gy, Gz, Gx;
// 拨盘增加角度
inline float target_magz = 0;
inline float next_target_magz = 0;  //-6°
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
inline rm::i16 V_shooter_1 = -4605;
inline rm::i16 V_shooter_2 = -3770;  // 12m/s
inline rm::i16 e_area = 100;
inline rm::i16 limit = -3000;
// 摩擦轮速度监测
inline rm::i16 shooter_1;
inline rm::i16 shooter_2;
inline rm::i16 shooter_3;
inline rm::i16 shooter_4;
inline rm::i16 shooter_5;
inline rm::i16 shooter_6;
// 底盘力矩监测
inline rm::i16 P_chassis_1;
inline rm::i16 P_chassis_2;
inline rm::i16 P_chassis_3;
inline rm::i16 P_chassis_4;
// 底盘速度监测
inline rm::i16 V_chassis_1;
inline rm::i16 V_chassis_2;
inline rm::i16 V_chassis_3;
inline rm::i16 V_chassis_4;
// PIDerror
inline float error;
// pitch_out
inline float pitch_out;
inline float yaw_out;
// 电机状态
inline uint8_t yaw_state;
// 拨盘补偿
inline bool magz_compensation_flag{false};
inline int magz_compensation_count{0};
inline float magz_compensation = 0;
// 功率模型
inline rm::modules::M3508PowerModel power_model;
// 初始电流
inline float initial_currents[4];
// 输出电流
inline float output_currents[4];
// 输出功率限额
inline float power_limit = 50.0;

inline float gyro_z;

inline float average1 = 0;

enum class autoaim_state { kAutoAim_Disable, kAutoAim_Enable, kAutoAim_FIRE };

inline rm::f32 aimbot_pitch;
inline rm::f32 aimbot_yaw;
inline int aimbot_state_flag = 0;
inline int imu_count = 0;
inline float pos_target = 0;
inline float pos_real = 0;
inline float vel_target = 0;
inline float vel_real = 0;
inline bool power_management_gimbal_last;
inline bool power_management_shooter_last;
inline int16_t shooter_m = 0;
inline bool overpower=false;
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
// 底盘控制+底盘功率控制
void ChassisPower();
// 裁判系统
void Referee();
// 自瞄更新
void AutoaimUpdate();
void CANAutoaimUpdate();
// 自定义客户端
void CustomClientUpdate();
// VOFA监测
void VOFA();
// 超级电容
// void SuperCupUpdate();
// 随动监测
inline int follow;
inline bool cc_mouse_l;
inline int16_t cc_mouse_x;
inline int16_t cc_mouse_y;
inline bool key_w;
inline bool key_a;
inline bool key_s;
inline bool key_d;
inline bool key_q;
inline bool key_e;
inline int aimbot_target;
inline int aimbot_state;
inline int heat_limit;
inline int16_t heat_buffer;
inline bool follow_state{true};
inline float cms_v{0.0f};
inline float cms_i{0.0f};
#endif  // BOARDC_MAIN_HPP