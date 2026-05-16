#ifndef BOARDC_GIMBAL_HPP
#define BOARDC_GIMBAL_HPP
// 新librm库适配
#include <librm.hpp>
#include "can.h"
#include "usart.h"
#include "spi.h"
#include "timer_task.hpp"
#include "ControllerPidGimbal.hpp"
#include "ControllerPidAmmo.hpp"
#include "FreemasterDbug.hpp"
#include "Usb.hpp"
#include "WS2812b.hpp"

extern void FreemasterDebug();
extern AimbotFrame_SCM_t Aimbot;  // 自瞄数据引出
class Gimbal {
 public:
  double yaw = 0;    // imu yaw数据
  double roll = 0;   // imu roll数据
  double pitch = 0;  // imu pitch数据

  double yaw_ = 0;    // imu yaw数据(-pi到pi)
  double roll_ = 0;   // imu roll数据(-pi到pi)
  double pitch_ = 0;  // imu pitch数据(-pi到pi)

  double rc_yaw_data = 0;    // 遥控器yaw数据
  double rc_pitch_data = 0;  // 遥控器pitch数据

  bool vt03_flag_lf = 0;  // 左标志位
  bool vt03_flag_rh = 0;  // 右标志位

  bool vt03_last_fn_left = false;   // 左Fn上一帧状态
  bool vt03_last_fn_right = false;  // 右Fn上一帧状态

  bool DM_is_enable = false;  // 达秒使能标志位

  float pitch_min_pos = 3.00;        // pitch电机最小限位
  float pitch_max_pos = 3.75;        // pitch电机最大限位
                                     // 机械限位
  float yaw_center_encoder = 5.174;  // TODO云台机械中位对应的编码器角度
  float yaw_relative = 0.0f;         // TODO 当前云台相对机架夹角
  float yaw_min_limit = -2.30;       // TODO 左限位
  float yaw_max_limit = 2.30;        // TODO 右限位
  float yaw_delta = 0.0f;            // rc增加总量

  float dirl_speed = 5000;      // TODO 拨盘转速
  float redirl_speed = 1000;    // TODO 拨盘反转速
  float friction_speed = 6500;  // TODO 摩擦轮转速
  float shootstep = 100;        // TODO 手动调速步长
  int shootcnt = 0;             // 步长计数
  int shoottime = 150;          // TODO 弹速控制间隔
  int shoottime_ = shoottime;

  float spaver[10] = {0.0f};  // 弹速平均数组

  // 拨盘自动反转
  float auto_reverse_buffer[5] = {1.f, 2.f, 3.f, 4.f, 5.f};  // TODO 缓存区大小
  int auto_reverse_time_max = 150;                           // TODO 反转持续时间
  int auto_reverse_time = 0;                                 // 持续时间变量
  bool auto_reverse_flag = false;                            // 反转标志位

  // pitch补偿系数
  float pitch_torque = 0.0f;      // pitch电机力矩重力补偿量
  float pitch_torque_kp = 0.35f;  // TODO 重力补偿参数

  float pitch_cmd = 0.0f;       // pitch合输出
  float pitch_speed_tf = 0.0f;  // 速度正向输出
  float pitch_speed_kp = 0.1f;  // 速度输出比例系数

  // 滚转补偿参数（用 yaw/pitch 组合抵消小角度 roll）
  bool roll_comp_enable = true;  // TODO 滚转补偿开关
  float roll_comp_kp = 0.1f;     // TODO 补偿系数，rad_pitch_per_rad_roll
  float roll_comp_limit = 0.3f;  // TODO 最大补偿幅度（rad）

  int robot_id = 0;  // 裁判系统测试
  float rc_vt03_left_x = 0.0f;
  int cnt = 0;  // 进自瞄次数测试

  int led_blink_time = 0;  // LED闪烁计时器

  rm::hal::ThrottledCan<128> *can1{nullptr};  // CAN 总线接口
  rm::hal::ThrottledCan<128> *can2{nullptr};  // CAN 总线接口
  rm::hal::Serial<128> *dbus{nullptr};        // 遥控器串口接口
  rm::device::VT03 *vt03{nullptr};            // 图传对象

  rm::hal::SerialInterface *referee_uart;                                          // 裁判系统串口
  rm::device::Referee<rm::device::RefereeRevision::kNewV120> referee_data_buffer;  // 裁判系统数据缓冲区
  rm::hal::SerialInterface *vt03_uart;                                             // 图传串口

  rm::device::DeviceManager<1> device_rc;      // 遥控管理器，维护所有设备在线状态
  rm::device::DeviceManager<1> device_vt03;    // 新遥控器管理器
  rm::device::DeviceManager<2> device_gimbal;  // 云台管理器
  rm::device::DeviceManager<3> device_shoot;   // 发射管理器

  int time_ = 0;  // 系统心跳

  rm::device::BMI088 *imu{nullptr};      // IMU
  rm::modules::MahonyAhrs ahrs{500.0f};  // TODO Mahony滤波控制频率
  rm::device::DR16 *rc{nullptr};         // 遥控器

  rm::device::HipnucImuCan *imu_new{nullptr};  // ch040

  rm::device::GM6020 *yaw_motor{nullptr};                                           // 云台 Yaw 上电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr};  // 云台 Pitch 电机
  rm::device::M3508 *friction_left{nullptr};                                        // 左侧摩擦轮电机
  rm::device::M3508 *friction_right{nullptr};                                       // 右侧摩擦轮电机
  rm::device::M2006 *dial_motor{nullptr};                                           // 拨盘电机

  typedef enum {
    kNoForce,  // 云台无力
    kManual,   // 云台手动
    kAuto,     // 云台自瞄

    kStop,             // 发射机构无力
    kReady,            // 发射机构准备开火
    kFire              // 发射机构开火
  } StateMachineType;  // 遥控器状态机

  StateMachineType AmmoState_ = {kStop};       // 初始化发射机构状态
  StateMachineType GimbalState_ = {kNoForce};  // 初始化云台运动状态

  Gimbal2Dof gimbal_controller;  // 二轴云台PID控制器
  Shoot2Fric shoot_controller;   // 双摩擦轮发射机构控制器

  // 小角度 roll 补偿：将 roll 误差分解到 yaw/pitch
  // roll 中点 0，值域 [-π, π]
  std::pair<double, double> ApplyRollComp(double yaw_target, double pitch_target) {
    if (!roll_comp_enable) {
      return {yaw_target, pitch_target};
    }
    double roll_err = roll;
    roll_err = rm::modules::Clamp(roll_err, -roll_comp_limit, roll_comp_limit);

    // 近似分解：机体 roll 对于当前朝向 yaw，投影到 yaw/pitch
    double yaw_correction = roll_comp_kp * roll_err * std::sin(yaw_target);
    double pitch_correction = -roll_comp_kp * roll_err * std::cos(yaw_target);

    double new_yaw = rm::modules::Wrap(yaw_target + yaw_correction, -M_PI, M_PI);
    double new_pitch = rm::modules::Clamp(pitch_target + pitch_correction, pitch_min_pos, pitch_max_pos);
    return {new_yaw, new_pitch};
  }

  void GimbalInit() {
    time_ = 0;  // 系统心跳置0
    can1 = new rm::hal::ThrottledCan<128>{3000, hcan1};
    can2 = new rm::hal::ThrottledCan<128>{3000, hcan2};
    dbus = new rm::hal::Serial<128>{huart3, false, true};

    imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
    imu_new = new rm::device::HipnucImuCan{*can2, 8};
    rc = new rm::device::DR16{*dbus};
    vt03 = new rm::device::VT03;

    referee_uart = new rm::hal::Serial<128>{huart6, false, false};
    vt03_uart = new rm::hal::Serial<128>{huart1, true, true};

    yaw_motor = new rm::device::GM6020{*can2, 7};
    pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
        *can1, {0x05, 0x06, 10.0f, 20.0f, 10.0f, {0.0f, 10.0f}, {0.0f, 5.0f}}};

    friction_left = new rm::device::M3508{
        *can1,
        4,
    };
    friction_right = new rm::device::M3508{
        *can1,
        3,
    };
    dial_motor = new rm::device::M2006{*can2, 5};

    // 裁判系统串口接收
    const rm::hal::SerialRxCallbackFunction ref_rx_callback = [&](const etl::span<const uint8_t> &data) {
      for (const auto byte : data) {
        referee_data_buffer << byte;
      }
    };
    referee_uart->AttachRxCallback(ref_rx_callback);

    // 图传串口接收
    rm::hal::SerialRxCallbackFunction tc_rx_callback = [&](const etl::span<const uint8_t> &data) {
      for (const auto byte : data) {
        *vt03 << byte;
      }
    };
    vt03_uart->AttachRxCallback(tc_rx_callback);

    device_rc << rc;                                                // 副遥控器
    device_vt03 << vt03;                                            // 主遙控器
    device_gimbal << yaw_motor << pitch_motor;                      // 云台电机
    device_shoot << friction_left << friction_right << dial_motor;  // 发射机构电机

    can1->SetFilter(0, 0);  // 设置滤波器
    can1->Begin();
    can2->SetFilter(0, 0);  // 设置滤波器
    can2->Begin();
    rc->Begin();
    vt03_uart->Start();
    referee_uart->Start();

    GimbalPIDInit();
    AmmoPIDInit();

    gimbal_controller.Enable(false);  // 云台控制器
    pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);

    shoot_controller.Enable(false);                   // 控制器初始化
    shoot_controller.Arm(false);                      // 摩擦轮武装（允许转动）
    shoot_controller.SetMode(Shoot2Fric::kFullAuto);  // 连发模式
    shoot_controller.SetLoaderSpeed(0.0f);            // 拨盘目标线速度
    shoot_controller.SetArmSpeed(0.0f);               // 摩擦轮目标线速度
  }

  void GimbalPIDInit();

  void AmmoPIDInit();

  void RCStateUpdate();

  bool RcIsOnline();

  bool Vt03IsOnline();

  bool Rcchoose();

  float GetYawMotorAngleRad();

  void GimbalControl();

  void AmmoControl();

  void ShootSpeedControl();

  void Vt03Control();

  void WS2812Control();

  float SpeedAver();

  // 遥控器和imu数据解算+DjiMotor发信息
  void SubLoop500Hz();
  // DmMotor电机发信息
  void SubLoop250Hz();
  void SubLoop100Hz();
  void SubLoop50Hz();
  void SubLoop10Hz();
};

#endif  // BOARDC_GIMBAL_HPP
