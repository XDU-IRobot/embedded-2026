#ifndef BOARDC_GIMBAL_HPP
#define BOARDC_GIMBAL_HPP
// 新librm库适配
#include <librm.hpp>
#include "can.h"
#include "usart.h"
#include "timer_task.hpp"
#include "ControllerPidGimbal.hpp"
#include "ControllerPidAmmo.hpp"
#include "FreemasterDbug.hpp"
#include "Usb.hpp"
#include "WS2812b.hpp"
#include "dynamics.hpp"
#include "anglediff2.hpp"
#include "RcControl.hpp"
#include "UI/referee_user.hpp"
#include "ControllerFeedForward.hpp"

extern void FreemasterDebug();
extern AimbotFrame_SCM_t Aimbot;  // 自瞄数据引出
extern Gimbal2DofDynamics drone_gb;

class Gimbal {
 public:
  double pitch_ = 0;
  double roll_ = 0;
  double yaw_ = 0;

  double rc_yaw_data = 0;    // 遥控器yaw数据
  double rc_pitch_data = 0;  // 遥控器pitch数据

  bool vt03_flag_lf = 0;  // 左标志位
  bool vt03_flag_rh = 0;  // 右标志位

  bool vt03_last_fn_left = false;   // 左Fn上一帧状态
  bool vt03_last_fn_right = false;  // 右Fn上一帧状态

  bool DM_is_enable = false;  // 达秒使能标志位

  float pitch_min_pos = -0.80f;  // pitch电机最小限位
  float pitch_max_pos = 0.15f;   // pitch电机最大限位

  // 机械限位
  int yaw_center_encoder = 6870;  // TODO云台机械中位对应的编码器角度
  int yaw_encoder_last = 0;
  int yaw_abs = 0;
  int yaw_min_limit = -2000;  // TODO 左限位
  int yaw_max_limit = 4300;   // TODO 右限位
  float yaw_delta = 0.0f;     // rc增加总量

  float dirl_speed_base = 5000;
  float dirl_speed = 5000;      // TODO 拨盘转速
  float redirl_speed = 1000;    // TODO 拨盘反转速
  float friction_speed = 6200;  // TODO 摩擦轮转速
  float friction_speed_base = 6200;  // TODO 摩擦轮转速
  float shootstep = 50;        // TODO 手动调速步长
  int shootcnt = 0;             // 步长计数

  float spaver[5] = {0.0f};  // 弹速平均数组

  // 拨盘自动反转
  float auto_reverse_buffer[5] = {1.f, 2.f, 3.f, 4.f, 5.f};  // TODO 缓存区大小
  int auto_reverse_time_max = 150;                           // TODO 反转持续时间
  int auto_reverse_time = 0;                                 // 持续时间变量
  bool auto_reverse_flag = false;                            // 反转标志位

  // 发弹延迟测量（触发上升沿 → 摩擦轮转速下降）
  enum FireDelayState { kDelayIdle, kDelayWaiting };
  FireDelayState delay_state_ = kDelayIdle;
  bool delay_last_trigger_ = false;
  int delay_tick_count_ = 0;
  float delay_avg_ms_ = 0.0f;
  int delay_sample_count_ = 0;
  float delay_peak_rpm_ = 0.0f;      // 触发后 RPM 峰值
  float delay_drop_delta_ = 200.0f;  // 转速跌落阈值 (RPM)

  // pitch补偿系数
  float pitch_torque = 0.0f;       // pitch电机前馈补偿量
  float yaw_torque = 0.0f;         // yaw电机前馈补偿量
  float yaw_torque_kp = 10000.0f;  // TODO 力矩转电流输出环比例
  Eigen::Vector2f tau_ff;
  float yaw_tau2voltage = 0.0f;

  float pitch_cmd = 0.0f;       // pitch合输出
  float pitch_speed_tf = 0.0f;  // 速度正向输出
  float pitch_speed_kp = 0.1f;  // 速度输出比例系数

  // 滚转补偿参数（用 yaw/pitch 组合抵消小角度 roll）
  bool roll_comp_enable = false;             // TODO 滚转补偿开关
  float roll_comp_kp = 0.1f;                 // TODO 补偿系数，rad_pitch_per_rad_roll
  float roll_comp_limit = 0.3f;              // TODO 最大补偿幅度（rad）
  float roll_comp_target[2] = {0.0f, 0.0f};  // 储存补偿后的目标角度 0yaw,1pitch

  // 前馈手控微分项计算
  AngleDiff2 rc_yaw_diff;
  AngleDiff2 rc_pitch_diff;

  float rc_yaw_vel = 0.0f;
  float rc_yaw_acc = 0.0f;

  float rc_pitch_vel = 0.0f;
  float rc_pitch_acc = 0.0f;

  int robot_id = 0;  // 裁判系统测试
  int ID_last = 0;   // 红蓝方离线标识位
  float rc_vt03_left_x = 0.0f;
  int cnt = 0;  // 进自瞄次数测试

  bool Len_control = 0;                                  // 是否使用镜头标志位
  float len_speed = 700.0f;                              // 旋转速度
  float Len_buffer[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};  // 堵转编码器buffer
  bool lens_direction_ = true;                           // 镜头旋转方向: true=正向, false=反向

  float AimDistance = 0.0f;  // 自瞄预测距离

  int led_blink_time = 0;  // LED闪烁计时器

  rm::hal::ThrottledCan<128> *can1{nullptr};  // CAN 总线接口
  rm::hal::ThrottledCan<128> *can2{nullptr};  // CAN 总线接口
  rm::hal::Serial<128> *dbus{nullptr};        // 遥控器串口接口
  rm::device::VT03 *vt03{nullptr};            // 图传对象

  rm::hal::SerialInterface *referee_uart;                                          // 裁判系统串口
  rm::device::Referee<rm::device::RefereeRevision::kNewV120> referee_data_buffer;  // 裁判系统数据缓冲区
  rm::device::RefereeUser<rm::device::RefereeRevision::kNewV120> referee_user;
  rm::hal::SerialInterface *vt03_uart;  // 图传串口

  rm::device::DeviceManager<1> device_rc;      // 遥控管理器，维护所有设备在线状态
  rm::device::DeviceManager<1> device_vt03;    // 新遥控器管理器
  rm::device::DeviceManager<2> device_gimbal;  // 云台管理器
  rm::device::DeviceManager<3> device_shoot;   // 发射管理器

  uint16_t time_ = 0;  // 系统心跳

  rm::modules::MahonyAhrs ahrs{500.0f};  // TODO Mahony滤波控制频率
  rm::device::DR16 *rc{nullptr};         // 遥控器
  rm::device::ControlSource *control_rc{nullptr};

  rm::device::HipnucImuCan *imu_new{nullptr};  // ch040

  rm::device::GM6020 *yaw_motor{nullptr};                                           // 云台 Yaw 上电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr};  // 云台 Pitch 电机
  rm::device::M3508 *friction_left{nullptr};                                        // 左侧摩擦轮电机
  rm::device::M3508 *friction_right{nullptr};                                       // 右侧摩擦轮电机
  rm::device::M2006 *dial_motor{nullptr};                                           // 拨盘电机
  rm::device::M2006 *lens_motor{nullptr};

  typedef enum {
    kNoForce,  // 云台无力
    kManual,   // 云台手动
    kAuto,     // 云台自瞄

    kStop,             // 发射机构无力
    kReady,            // 发射机构准备开火
    kFire              // 发射机构开火
  } StateMachineType;  // 遥控器状态机

  StateMachineType AmmoState_ = {kStop};               // 初始化发射机构状态
  StateMachineType GimbalState_ = {kNoForce};          // 初始化云台运动状态
  StateMachineType last_ammo_state_for_lens_ = kStop;  // 上一帧发射状态，用于镜头方向边沿检测

  Gimbal2Dof gimbal_controller;  // 二轴云台PID控制器
  Shoot2Fric shoot_controller;   // 双摩擦轮发射机构控制器
  u_int8_t dataBox[128];

  Feedforward yaw_ff;

  bool vt03_last_r_key = false;

  void GimbalInit() {
    yaw_ff.Init(0.002, 0.2);

    time_ = 0;  // 系统心跳置0
    referee_user.attachReferee(&referee_data_buffer);
    can1 = new rm::hal::ThrottledCan<128>{6000, hcan1};
    can2 = new rm::hal::ThrottledCan<128>{6000, hcan2};
    dbus = new rm::hal::Serial<128>{huart3, false, true};

    imu_new = new rm::device::HipnucImuCan{*can2, 8};
    rc = new rm::device::DR16{*dbus};
    vt03 = new rm::device::VT03;

    referee_uart = new rm::hal::Serial<128>{huart6, true, true};
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
    lens_motor = new rm::device::M2006{*can1, 1};

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
        referee_data_buffer << byte;
      }
    };
    vt03_uart->AttachRxCallback(tc_rx_callback);
    referee_data_buffer.AttachCallback([ObjectPtr = &referee_user]<typename T0, typename T1>(T0 &&PH1, T1 &&PH2) {
      ObjectPtr->AttachCallback(std::forward<T0>(PH1), std::forward<T1>(PH2));
    });

    device_rc << rc;                                                // 副遥控器
    device_vt03 << vt03;                                            // 主遙控器
    device_gimbal << yaw_motor << pitch_motor;                      // 云台电机
    device_shoot << friction_left << friction_right << dial_motor;  // 发射机构电机

    control_rc = new ControlSource;
    rc->SetHeartbeatTimeout(std::chrono::milliseconds(1000));
    vt03->SetHeartbeatTimeout(std::chrono::milliseconds(1000));

    can1->SetFilter(0, 0);  // 设置滤波器
    can1->Begin();
    can2->SetFilter(0, 0);  // 设置滤波器
    can2->Begin();
    rc->Begin();
    vt03_uart->Start();
    referee_uart->Start();
    GimbalPIDInit();
    AmmoPIDInit();

    gimbal_controller.Enable(false);              // 云台控制器
    gimbal_controller.EnableSpeedPid(true);       // 两轴都开启速度环（和以前一样）
    gimbal_controller.EnableYawCurrentPid(true);  // 仅 yaw 再串上电流环

    pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    rc_yaw_diff.SetFilter(0.35, 0.20);  // 前馈微分项滤波
    rc_pitch_diff.SetFilter(0.35, 0.20);

    shoot_controller.Enable(false);                   // 控制器初始化
    shoot_controller.Arm(false);                      // 摩擦轮武装（允许转动）
    shoot_controller.SetMode(Shoot2Fric::kFullAuto);  // 连发模式
    shoot_controller.SetLoaderSpeed(0.0f);            // 拨盘目标线速度
    shoot_controller.SetArmSpeed(0.0f);               // 摩擦轮目标线速度

    yaw_encoder_last = yaw_motor->encoder();
    int yaw_encoder_err = yaw_encoder_last - yaw_center_encoder;
    if (yaw_encoder_err >= 4000)
      yaw_encoder_err -= 8191;
    else if (yaw_encoder_err <= -4000)
      yaw_encoder_err += 8191;
    yaw_abs += yaw_encoder_err;
  }
  std::pair<double, double> ApplyRollComp(double yaw_target, double pitch_target) {
    if (!roll_comp_enable) {
      return {yaw_target, pitch_target};
    }
    double roll_err = roll_;
    roll_err = rm::modules::Clamp(roll_err, -roll_comp_limit, roll_comp_limit);

    // 近似分解：机体 roll 对于当前朝向 yaw，投影到 yaw/pitch
    double yaw_correction = roll_comp_kp * roll_err * std::sin(yaw_target);
    double pitch_correction = -roll_comp_kp * roll_err * std::cos(yaw_target);

    double new_yaw = rm::modules::Wrap(yaw_target + yaw_correction, -M_PI, M_PI);
    double new_pitch = rm::modules::Clamp(pitch_target + pitch_correction, pitch_min_pos, pitch_max_pos);
    return {new_yaw, new_pitch};
  }
  void RCStateUpdate() {
    switch (control_rc->switch_r()) {
      case rm::device::DR16::SwitchPosition::kUp:  // 发射控制逻辑
        AmmoState_ = kFire;
        break;
      case rm::device::DR16::SwitchPosition::kMid:
        AmmoState_ = kReady;
        break;
      default:
        AmmoState_ = kStop;
        break;
    }

    switch (control_rc->switch_l()) {
      case DR16::SwitchPosition::kUp:  // 上打完全自瞄
        GimbalState_ = kAuto;
        break;
      case DR16::SwitchPosition::kMid:  // 中位按下鼠标右键跟随
        if (control_rc->mouse_button_right())
          GimbalState_ = kAuto;
        else
          GimbalState_ = kManual;
        break;
      default:
        GimbalState_ = kNoForce;
        break;
    }
    if (!referee_data_buffer.data().robot_status.power_management_gimbal_output) GimbalState_ = kNoForce;
  }
  void UpdateRcAngleDiff(float yaw_data, float pitch_data, float dt) {
    rc_yaw_diff.Update(yaw_data, dt, true);
    rc_pitch_diff.Update(pitch_data, dt, false);

    rc_yaw_vel = rc_yaw_diff.vel();
    rc_yaw_acc = rc_yaw_diff.acc();

    rc_pitch_vel = rc_pitch_diff.vel();
    rc_pitch_acc = rc_pitch_diff.acc();
  }
  void GimbalPIDInit() {
    //电流环控制参数
    // // yaw
    // gimbal_controller.pid()
    //     .yaw_position.SetKp(12.0f)//12
    //     .SetKi(0.0f)
    //     .SetKd(2.0f)//2
    //     .SetMaxOut(3000.0f)
    //     .SetMaxIout(10.0f)
    //     .SetDiffLpfAlpha(0.01);
    // gimbal_controller.pid()
    //     .yaw_speed.SetKp(5500.0f)
    //     .SetKi(0.0f)
    //     .SetKd(500.0f)
    //     .SetMaxOut(16384.0f)
    //     .SetMaxIout(1000.0f)
    //     .SetDiffLpfAlpha(0.01);
    // gimbal_controller.pid()
    //     .yaw_current.SetKp(0.5f)
    //     .SetKi(0.0f)
    //     .SetKd(0.5f)
    //     .SetMaxOut(16384.0f)
    //     .SetMaxIout(1000.0f)
    //     .SetDiffLpfAlpha(0.01);
    gimbal_controller.pid()
        .yaw_position.SetKp(16.0f)
        .SetKi(0.0f)
        .SetKd(3.0f)
        .SetMaxOut(3000.0f)
        .SetMaxIout(10.0f)
        .SetDiffLpfAlpha(0.01);
    gimbal_controller.pid()
        .yaw_speed.SetKp(5000.0f)
        .SetKi(0.0f)
        .SetKd(700.0f)
        .SetMaxOut(25000.0f)
        .SetMaxIout(1000.0f)
        .SetDiffLpfAlpha(0.01);
    gimbal_controller.pid()
        .yaw_current.SetKp(0.5f)
        .SetKi(0.0f)
        .SetKd(0.5f)
        .SetMaxOut(25000.0f)
        .SetMaxIout(1000.0f)
        .SetDiffLpfAlpha(0.01);
    // pitch
    gimbal_controller.pid()
        .pitch_position.SetKp(22.f)
        .SetKi(0.0f)
        .SetKd(0.5f)
        .SetMaxOut(500.0f)
        .SetMaxIout(10.0f)
        .SetDiffLpfAlpha(0.05);
    gimbal_controller.pid().pitch_speed.SetKp(1.f).SetKi(0.0f).SetKd(0.001f).SetMaxOut(10.0f).SetMaxIout(5.0f);
  }
  void AmmoPIDInit() {
    shoot_controller.pid().fric_1_speed.SetKp(25.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(20000.0f).SetMaxIout(1000.0f);
    shoot_controller.pid().fric_2_speed.SetKp(25.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(20000.0f).SetMaxIout(1000.0f);
    shoot_controller.pid().loader_speed.SetKp(25.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(20000.0f).SetMaxIout(2000.0f);
  }
  void GimbalControl() {
    control_rc->act(*rc, *vt03);
    int yaw_encoder_current = yaw_motor->encoder();
    int yaw_encoder_err = yaw_encoder_current - yaw_encoder_last;
    if (yaw_encoder_err >= 4000)
      yaw_encoder_err -= 8191;
    else if (yaw_encoder_err <= -4000)
      yaw_encoder_err += 8191;
    yaw_abs += yaw_encoder_err;
    yaw_encoder_last = yaw_encoder_current;

    if (control_rc->key(DR16::Key::kB)) DM_is_enable = false;
    if (control_rc->key(DR16::Key::kG)) pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);

    if (GimbalState_ == kManual) {
      if (DM_is_enable == false) {
        if (pitch_motor->status() == static_cast<u8>(DmMotorStatus::kEnable)) {
          pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
          DM_is_enable = true;
        } else if (pitch_motor->status() != static_cast<u8>(DmMotorStatus::kDisable))
          pitch_motor->SendInstruction(DmMotorInstructions::kClearError);
        else
          pitch_motor->SendInstruction(DmMotorInstructions::kEnable);
        gimbal_controller.Enable(true);
        rc_yaw_data = yaw_;      // 第一次进入更新当前位置
        rc_pitch_data = pitch_;  // 使用 IMU pitch 作为初始姿态
        rc_pitch_data = modules::Clamp(rc_pitch_data, pitch_min_pos, pitch_max_pos);  // 对rc数据进行限位
      } else {
        yaw_delta = 0.0f;
        if (control_rc->key(rm::device::DR16::Key::kCtrl)) {
          if (control_rc->key(rm::device::DR16::Key::kW)) rc_pitch_data += 0.0001f;
          if (control_rc->key(rm::device::DR16::Key::kS)) rc_pitch_data -= 0.0001f;
          if (control_rc->key(rm::device::DR16::Key::kA)) yaw_delta += 0.0001f;
          if (control_rc->key(rm::device::DR16::Key::kD)) yaw_delta -= 0.0001f;
        } else {
          yaw_delta -= rm::modules::Map(control_rc->left_x(), -660, 660, -0.005f, 0.005f);      // dt7手控
          yaw_delta -= rm::modules::Map(control_rc->mouse_x(), -660, 660, -0.03f, 0.03f);       // dt7备份控制
          rc_pitch_data += rm::modules::Map(control_rc->left_y(), -660, 660, -0.005f, 0.005f);  // dt7手控
          rc_pitch_data += rm::modules::Map(control_rc->mouse_y(), -660, 660, -0.03f, 0.03f);   // dt7备份控制
        }
        if (yaw_abs >= yaw_max_limit && yaw_delta < 0.0f) {  // 机械限位返回逻辑
          yaw_delta = 0.0f;
        }
        if (yaw_abs <= yaw_min_limit && yaw_delta > 0.0f) {
          yaw_delta = 0.0f;
        }

        rc_yaw_data = rm::modules::Wrap(rc_yaw_data + yaw_delta, -M_PI, M_PI);
        rc_pitch_data = rm::modules::Clamp(rc_pitch_data, pitch_min_pos, pitch_max_pos);
        // 滚转补偿
        auto roll_comp = ApplyRollComp(rc_yaw_data, rc_pitch_data);
        roll_comp_target[0] = roll_comp.first;   // yaw
        roll_comp_target[1] = roll_comp.second;  // pitch

        // 前馈计算项
        UpdateRcAngleDiff(roll_comp.first, roll_comp.second, 0.002f);
        tau_ff =
            drone_gb.ComputeFf(-rm::modules::Wrap(yaw_motor->pos_rad() - 5.14, -M_PI, M_PI), -0.45 - pitch_motor->pos(),
                               rc_yaw_vel, rc_pitch_vel, rc_yaw_acc, rc_pitch_acc, Eigen::Vector3f(0.0f, 0.0f, -9.81f));
        yaw_tau2voltage = tau_ff.x() * 2530.0f + rc_yaw_vel * (60.0f / (2.0f * M_PI)) * 78.0f;  // 力矩转换控制电流
        yaw_tau2voltage = 0;
        // 设定目标，并计算
        gimbal_controller.SetTarget(roll_comp.first, roll_comp.second, 0, 0);
        gimbal_controller.Update(yaw_, -yaw_motor->rpm() * M_PI / 30.0, yaw_motor->current(), pitch_,
                                 -pitch_motor->vel(), 0, 1.f);
        yaw_motor->SetCurrent(rm::modules::Clamp(-gimbal_controller.output().yaw - yaw_tau2voltage, -25000,
                                                 25000));  // 设置输出电流并输出
      }
    } else if (GimbalState_ == kAuto) {
      // 自瞄模式控制
      if (DM_is_enable == false) {  // 使达妙电机使能
        if (pitch_motor->status() == static_cast<rm::u8>(rm::device::DmMotorStatus::kEnable)) {
          pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
          DM_is_enable = true;
        } else if (pitch_motor->status() != static_cast<rm::u8>(rm::device::DmMotorStatus::kDisable))
          pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
        else
          pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);

        gimbal_controller.Enable(true);
        rc_yaw_data = yaw_;
        rc_pitch_data = pitch_;  // 使用 IMU pitch 作为初始姿态
        rc_pitch_data = rm::modules::Clamp(rc_pitch_data, pitch_min_pos, pitch_max_pos);
      } else {
        if (Aimbot.AimbotState == 2 || Aimbot.AimbotState == 4) {
          rc_yaw_data = rm::modules::Wrap(Aimbot.TargetYawAngle, -M_PI, M_PI);
          rc_pitch_data = rm::modules::Clamp(Aimbot.TargetPitchAngle, pitch_min_pos, pitch_max_pos);
        } else {
          yaw_delta = 0.0f;
          if (control_rc->key(rm::device::DR16::Key::kCtrl)) {
            if (control_rc->key(rm::device::DR16::Key::kW)) rc_pitch_data += 0.0001f;
            if (control_rc->key(rm::device::DR16::Key::kS)) rc_pitch_data -= 0.0001f;
            if (control_rc->key(rm::device::DR16::Key::kA)) yaw_delta += 0.0001f;
            if (control_rc->key(rm::device::DR16::Key::kD)) yaw_delta -= 0.0001f;
          } else {
            yaw_delta -= rm::modules::Map(control_rc->left_x(), -660, 660, -0.005f, 0.005f);      // dt7手控
            yaw_delta -= rm::modules::Map(control_rc->mouse_x(), -660, 660, -0.03f, 0.03f);       // dt7备份控制
            rc_pitch_data += rm::modules::Map(control_rc->left_y(), -660, 660, -0.005f, 0.005f);  // dt7手控
            rc_pitch_data += rm::modules::Map(control_rc->mouse_y(), -660, 660, -0.03f, 0.03f);   // dt7备份控制
          }
          if (yaw_abs >= yaw_max_limit && yaw_delta < 0.0f) {  // 机械限位返回逻辑
            yaw_delta = 0.0f;
          }
          if (yaw_abs <= yaw_min_limit && yaw_delta > 0.0f) {
            yaw_delta = 0.0f;
          }

          rc_yaw_data = rm::modules::Wrap(rc_yaw_data + yaw_delta, -M_PI, M_PI);
          rc_pitch_data = rm::modules::Clamp(rc_pitch_data, pitch_min_pos, pitch_max_pos);
        }
        // 滚转补偿
        auto roll_comp = ApplyRollComp(rc_yaw_data, rc_pitch_data);
        roll_comp_target[0] = roll_comp.first;   // yaw
        roll_comp_target[1] = roll_comp.second;  // pitch

        // 前馈计算项
        UpdateRcAngleDiff(roll_comp.first, roll_comp.second, 0.002f);
        tau_ff = drone_gb.ComputeFf(-rm::modules::Wrap(yaw_motor->pos_rad() - 5.14, -M_PI, M_PI),
                                    -0.45 - pitch_motor->pos(), Aimbot.YawSpeed, Aimbot.PitchSpeed, Aimbot.YawAngSpeed,
                                    Aimbot.PitchAngSpeed, Eigen::Vector3f(0.0f, 0.0f, -9.81f));
        yaw_tau2voltage = tau_ff.x() * 2530.0f + Aimbot.YawSpeed * (60.0f / (2.0f * M_PI)) * 78.0f;  // 力矩转换控制电流
        yaw_tau2voltage = 0;
        // 设定目标，并计算
        gimbal_controller.SetTarget(roll_comp.first, roll_comp.second, 0, 0);
        gimbal_controller.Update(yaw_, -yaw_motor->rpm() * M_PI / 30.0, yaw_motor->current(), pitch_,
                                 -pitch_motor->vel(), 0, 1.f);
        yaw_motor->SetCurrent(rm::modules::Clamp(-gimbal_controller.output().yaw - yaw_tau2voltage, -25000,
                                                 25000));  // 设置输出电流并输出
      }
    } else {  // 失能
      if (DM_is_enable == true) {
        if (pitch_motor->status() == static_cast<rm::u8>(rm::device::DmMotorStatus::kDisable))
          DM_is_enable = false;
        else if (pitch_motor->status() != static_cast<rm::u8>(rm::device::DmMotorStatus::kEnable))
          pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
        else
          pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
        gimbal_controller.Enable(false);
        yaw_motor->SetCurrent(0);
      }
    }
  }
  void AmmoControl() {
    // 发射状态
    if (AmmoState_ == kFire) {
      shoot_controller.Enable(true);
      shoot_controller.Arm(true);
      shoot_controller.SetMode(Shoot2Fric::kFullAuto);

      if (control_rc->dial() >= 550 || control_rc->mouse_button_left()) {
        if (auto_reverse_flag) {
          shoot_controller.SetLoaderSpeed(-redirl_speed);
          auto_reverse_time--;
          auto_reverse_time < 1 ? auto_reverse_flag = false : auto_reverse_flag = true;
        } else {                        // 不反转
          if (GimbalState_ == kAuto) {  // 是自瞄下的状态
            if (Aimbot.AimbotState == 4) {
              shoot_controller.SetLoaderSpeed(dirl_speed);
            } else if (Aimbot.AimbotState == 2) {
              shoot_controller.SetLoaderSpeed(0.0f);
            } else {
              shoot_controller.SetLoaderSpeed(dirl_speed);
            }
          } else {  // 手动状态
            shoot_controller.SetLoaderSpeed(dirl_speed);
          }
        }
      } else if (control_rc->dial() <= -600) {
        shoot_controller.SetLoaderSpeed(-redirl_speed);
      } else {
        shoot_controller.SetLoaderSpeed(0.0f);
      }

      // 自动反转逻辑
      if (shoot_controller.GetLoaderSpeed() == dirl_speed) {
        auto_reverse_buffer[4] = auto_reverse_buffer[3];
        auto_reverse_buffer[3] = auto_reverse_buffer[2];
        auto_reverse_buffer[2] = auto_reverse_buffer[1];
        auto_reverse_buffer[1] = auto_reverse_buffer[0];
        auto_reverse_buffer[0] = dial_motor->encoder();
        if (auto_reverse_buffer[0] == auto_reverse_buffer[4]) {
          auto_reverse_flag = true;
          auto_reverse_time = auto_reverse_time_max;
        }
      }
      shoot_controller.SetArmSpeed(friction_speed);  // 摩擦轮目标线速度（rad/s 或你的系统单位）
      shoot_controller.Update(friction_left->rpm(), friction_right->rpm(), dial_motor->rpm());

      friction_left->SetCurrent((int16_t)rm::modules::Clamp(shoot_controller.output().fric_1, -10000, 10000));
      friction_right->SetCurrent((int16_t)rm::modules::Clamp(shoot_controller.output().fric_2, -10000, 10000));
      dial_motor->SetCurrent((int16_t)rm::modules::Clamp(shoot_controller.output().loader, -10000, 10000));

      // --- 发弹延迟测量 ---
      bool trigger_active = (control_rc->dial() >= 550 || control_rc->mouse_button_left());
      if (trigger_active && !delay_last_trigger_) {
        delay_state_ = kDelayWaiting;
        delay_tick_count_ = 0;
        delay_peak_rpm_ = fabs(friction_left->rpm());
      }
      delay_last_trigger_ = trigger_active;

      if (delay_state_ == kDelayWaiting) {
        delay_tick_count_++;
        float current_rpm = fabs(friction_left->rpm());
        if (current_rpm > delay_peak_rpm_) delay_peak_rpm_ = current_rpm;

        // 峰值接近目标转速 且 当前转速从峰值跌落超过阈值
        if (delay_peak_rpm_ >= friction_speed * 0.85f && current_rpm < delay_peak_rpm_ - delay_drop_delta_) {
          float delay_ms = delay_tick_count_ * 2.0f;
          delay_avg_ms_ = (delay_avg_ms_ * delay_sample_count_ + delay_ms) / (delay_sample_count_ + 1);
          delay_sample_count_++;
          delay_state_ = kDelayIdle;
        }
        if (delay_tick_count_ > 250) delay_state_ = kDelayIdle;  // 500ms 超时
      }
    }

    // 准备状态
    else if (AmmoState_ == kReady) {
      shoot_controller.Enable(true);
      shoot_controller.Arm(true);

      shoot_controller.SetMode(Shoot2Fric::kStop);
      shoot_controller.SetArmSpeed(0.0f);

      shoot_controller.Update(friction_left->rpm(), friction_right->rpm(), dial_motor->rpm());

      friction_left->SetCurrent((int16_t)rm::modules::Clamp(shoot_controller.output().fric_1, -10000, 10000));
      friction_right->SetCurrent((int16_t)rm::modules::Clamp(shoot_controller.output().fric_2, -10000, 10000));
      dial_motor->SetCurrent(0);
    }

    // 停止状态
    else {
      shoot_controller.Enable(false);
      shoot_controller.Arm(false);
      friction_left->SetCurrent(0);
      friction_right->SetCurrent(0);
      dial_motor->SetCurrent(0);
    }
  }
  void HeatLimit() {
    if (referee_data_buffer.data().power_heat_data.shooter_17mm_1_barrel_heat != 0 &&
        referee_data_buffer.data().robot_status.shooter_barrel_heat_limit != 0) {
      float percentage = (float)referee_data_buffer.data().power_heat_data.shooter_17mm_1_barrel_heat /
                         (float)referee_data_buffer.data().robot_status.shooter_barrel_heat_limit;
      if (percentage >= 1.0f) percentage = 1.0f;
      if (percentage >= 0.6)
        dirl_speed = dirl_speed_base - (percentage - 0.6) * 5000;
      else
        dirl_speed = dirl_speed_base;
    }
  }
  void ShootSpeedControl() {  // 弹速控制
    // static bool last_key_ctrl;
    static bool last_key_x{false};
    static bool last_key_c{false};
    static bool last_key_z{false};
    if (!control_rc->key(DR16::Key::kCtrl) && !last_key_x && control_rc->key(DR16::Key::kX)) {
      friction_speed -= shootstep;
      shootcnt -= 1;
    } else if (!control_rc->key(DR16::Key::kCtrl) && !last_key_c && control_rc->key(DR16::Key::kC)) {
      friction_speed += shootstep;
      shootcnt += 1;
    } else if (!control_rc->key(DR16::Key::kCtrl) && !last_key_z && control_rc->key(DR16::Key::kZ)) {
      friction_speed = friction_speed_base;
      shootcnt = 0;
    }
    last_key_x = control_rc->key(DR16::Key::kX);
    last_key_c = control_rc->key(DR16::Key::kC);
    last_key_z = control_rc->key(DR16::Key::kZ);
  }
  float SpeedAver() {
    float new_speed = referee_data_buffer.data().shoot_data.initial_speed;

    if (new_speed > 15 && new_speed != spaver[4]) {
      for (int i = 0; i < 4; i++) {
        spaver[i] = spaver[i + 1];
      }
      spaver[4] = new_speed;
    }

    // 计算平均值
    float sum = 0;
    int count = 0;
    for (int i = 0; i < 5; i++) {
      if (spaver[i] != 0) {
        sum += spaver[i];
        count++;
      }
    }
    return (count > 0) ? (sum / count) : 22.5f;
  }
  void LensControl() {
    // R键上升沿：翻转方向并启动电机
    if (control_rc->key(rm::device::DR16::Key::kR) && !vt03_last_r_key && pitch_ < -0.10f) {
      lens_direction_ = !lens_direction_;
      Len_control = 1;
      lens_motor->SetCurrent(lens_direction_ ? len_speed : -len_speed);
    }
    vt03_last_r_key = control_rc->key(rm::device::DR16::Key::kR);

    // 不在控制状态，不判断堵转
    if (!Len_control) {
      return;
    }
    // 更新编码器缓存
    Len_buffer[4] = Len_buffer[3];
    Len_buffer[3] = Len_buffer[2];
    Len_buffer[2] = Len_buffer[1];
    Len_buffer[1] = Len_buffer[0];
    Len_buffer[0] = lens_motor->encoder();
    // 堵转检测：编码器一段时间内几乎没变化则停转
    constexpr int kStallThreshold = 3;
    int delta = std::abs(static_cast<int>(Len_buffer[0]) - static_cast<int>(Len_buffer[4]));

    if (delta < kStallThreshold) {
      lens_motor->SetCurrent(0);
      Len_control = 0;
    }
  }
  void WS2812Control() {
    if (control_rc->key(rm::device::DR16::Key::kZ) && control_rc->key(rm::device::DR16::Key::kX) &&
        control_rc->key(rm::device::DR16::Key::kC)) {
      if (led_blink_time < 2) {
        Set_LED(0, 255, 0, 0);
        Set_LED(1, 255, 0, 0);
        Set_LED(2, 255, 0, 0);
        Set_LED(3, 255, 0, 0);
      } else if (led_blink_time < 4) {
        Set_LED(0, 0, 0, 0);
        Set_LED(1, 0, 0, 0);
        Set_LED(2, 0, 0, 0);
        Set_LED(3, 0, 0, 0);
      } else
        led_blink_time = 0;
      led_blink_time++;
    } else {
      // 前进后退
      if (!control_rc->key(rm::device::DR16::Key::kCtrl) && !control_rc->key(rm::device::DR16::Key::kShift) &&
          control_rc->key(rm::device::DR16::Key::kW) && !control_rc->key(rm::device::DR16::Key::kS))
        Set_LED(1, 0, 0, 255);
      else if (!control_rc->key(rm::device::DR16::Key::kCtrl) && !control_rc->key(rm::device::DR16::Key::kShift) &&
               !control_rc->key(rm::device::DR16::Key::kW) && control_rc->key(rm::device::DR16::Key::kS))
        Set_LED(1, 255, 0, 0);
      else
        Set_LED(1, 0, 0, 0);

      // 左右or偏航
      if (!control_rc->key(rm::device::DR16::Key::kCtrl) && !control_rc->key(rm::device::DR16::Key::kShift) &&
          control_rc->key(rm::device::DR16::Key::kA) ^ control_rc->key(rm::device::DR16::Key::kD)) {
        if (control_rc->key(rm::device::DR16::Key::kA)) {
          Set_LED(0, 0, 0, 0);
          Set_LED(2, 255, 255, 255);
        } else if (control_rc->key(rm::device::DR16::Key::kD)) {
          Set_LED(0, 255, 255, 255);
          Set_LED(2, 0, 0, 0);
        } else {
          Set_LED(0, 0, 0, 0);
          Set_LED(2, 0, 0, 0);
        }
      } else if (!control_rc->key(rm::device::DR16::Key::kCtrl) && control_rc->key(rm::device::DR16::Key::kShift) &&
                 control_rc->key(rm::device::DR16::Key::kA) ^ control_rc->key(rm::device::DR16::Key::kD)) {
        if (control_rc->key(rm::device::DR16::Key::kA)) {
          Set_LED(2, 0, 255, 255);
          Set_LED(0, 0, 0, 0);
        } else if (control_rc->key(rm::device::DR16::Key::kD)) {
          Set_LED(0, 0, 255, 255);
          Set_LED(2, 0, 0, 0);
        }
      } else {
        Set_LED(0, 0, 0, 0);
        Set_LED(2, 0, 0, 0);
      }

      // 上升
      if (!control_rc->key(rm::device::DR16::Key::kCtrl) && control_rc->key(rm::device::DR16::Key::kShift) &&
          control_rc->key(rm::device::DR16::Key::kW) && !control_rc->key(rm::device::DR16::Key::kS))
        Set_LED(3, 0, 0, 255);
      else if (!control_rc->key(rm::device::DR16::Key::kCtrl) && control_rc->key(rm::device::DR16::Key::kShift) &&
               !control_rc->key(rm::device::DR16::Key::kW) && control_rc->key(rm::device::DR16::Key::kS))
        Set_LED(3, 255, 0, 0);
      else
        Set_LED(3, 0, 0, 0);
    }
    Set_Brightness(25);
    WS2812_Send();
  }

  void SubLoop500Hz() {
    // ch040
    // pitch_ = -imu_new->pitch();  // （上正下负）（+-pi）
    // roll_ = -imu_new->roll();    //(左正右负)(+-pi)
    // yaw_ = imu_new->yaw();       //(左正右负)（+-pi）
    f32 euler_rpy_temp[3], quaternion_temp[4] = {imu_new->quat_w(), -imu_new->quat_x(), -imu_new->quat_y(),imu_new->quat_z()};
    modules::QuatToEuler(quaternion_temp, euler_rpy_temp);
    pitch_ = euler_rpy_temp[1];  // （上正下负）（+-pi）
    roll_ = euler_rpy_temp[0];    //(左正右负)(+-pi)
    yaw_ = euler_rpy_temp[2];       //(左正右负)（+-pi）

    GimbalImuSend(-imu_new->quat_x(), imu_new->quat_w(), imu_new->quat_z(), -imu_new->quat_y(), SpeedAver(),
                  referee_data_buffer.data().robot_status.robot_id);  // usb传输数据

    GimbalControl();                               // 云台控制更新
    AmmoControl();                                 // 发射机构更新
    rm::device::DjiMotorBase::SendCommand(*can1);  // 向大疆所有电机发数据
    rm::device::DjiMotorBase::SendCommand(*can2);  // 向大疆所有电机发数据
  }
  void SubLoop250Hz() {
    RCStateUpdate();

    // pitch负值向上输出
    pitch_torque = 1.2 * sin(pitch_ + 0.54);
    // if (GimbalState_ == kManual) {
    //   pitch_cmd = rm::modules::Clamp(-gimbal_controller.output().pitch - pitch_torque, -10, 10);  //
    //   发送达秒控制信息
    // } else {
    //   pitch_cmd = rm::modules::Clamp(-gimbal_controller.output().pitch - tau_ff.y() - pitch_torque, -10,
    //                                  10);  // 发送达秒控制信息
    // }
    pitch_cmd = rm::modules::Clamp(-gimbal_controller.output().pitch - pitch_torque, -10, 10);  // 发送达秒控制信息
    pitch_motor->SetMitCommand(0, 0, pitch_cmd, 0, 0);
    // pitch_motor->SetMitCommand(0, 0, -pitch_torque, 0, 0);

  }
  void SubLoop100Hz() {
    if (time_ % 5 == 0) {
      HeatLimit();
      ShootSpeedControl();  // 弹速手动控制
      FreemasterDebug();    // 调试更新
    }
  }
  void SubLoop50Hz() {
    if (time_ % 10 == 0) {
    }
  }
  void SubLoop10Hz() {
    if (time_ % 50 == 0) {
      WS2812Control();
      LensControl();
      time_=0;
    }
  }
};

#endif  // BOARDC_GIMBAL_HPP
