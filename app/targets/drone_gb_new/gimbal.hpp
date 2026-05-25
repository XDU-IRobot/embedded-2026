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

  float pitch_min_pos = -0.80f;      // pitch电机最小限位
  float pitch_max_pos = 0.25f;       // pitch电机最大限位
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
    double roll_err = roll_;
    roll_err = rm::modules::Clamp(roll_err, -roll_comp_limit, roll_comp_limit);

    // 近似分解：机体 roll 对于当前朝向 yaw，投影到 yaw/pitch
    double yaw_correction = roll_comp_kp * roll_err * std::sin(yaw_target);
    double pitch_correction = -roll_comp_kp * roll_err * std::cos(yaw_target);

    double new_yaw = rm::modules::Wrap(yaw_target + yaw_correction, -M_PI, M_PI);
    double new_pitch = rm::modules::Clamp(pitch_target + pitch_correction, pitch_min_pos, pitch_max_pos);
    return {new_yaw, new_pitch};
  }

  rm::hal::Serial<50> *refereeUart{nullptr};
  u_int8_t dataBox[128];

  void GimbalInit() {
    time_ = 0;  // 系统心跳置0
    can1 = new rm::hal::ThrottledCan<128>{3000, hcan1};
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
    rc_yaw_diff.SetFilter(0.35, 0.20);  // 前馈微分项滤波
    rc_pitch_diff.SetFilter(0.35, 0.20);

    shoot_controller.Enable(false);                   // 控制器初始化
    shoot_controller.Arm(false);                      // 摩擦轮武装（允许转动）
    shoot_controller.SetMode(Shoot2Fric::kFullAuto);  // 连发模式
    shoot_controller.SetLoaderSpeed(0.0f);            // 拨盘目标线速度
    shoot_controller.SetArmSpeed(0.0f);               // 摩擦轮目标线速度
  }

  void RCStateUpdate() {
    switch (rc->switch_r()) {
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
    switch (rc->switch_l()) {
      case rm::device::DR16::SwitchPosition::kUp:  // 上打完全自瞄
        GimbalState_ = kAuto;
        break;
      case rm::device::DR16::SwitchPosition::kMid:  // 中位按下鼠标右键跟随
        if (rc->mouse_button_right() || vt03->data().mouse_button_right)
          GimbalState_ = kAuto;
        else
          GimbalState_ = kManual;
        break;
      default:
        GimbalState_ = kNoForce;
        break;
    }
  }
  void Vt03Control() {
    // 左 Fn：云台状态切换
    // kNoForce -> kManual -> kNoForce
    if (vt03->data().left_button && !vt03_last_fn_left) {
      if (GimbalState_ == kNoForce) {
        GimbalState_ = kManual;
        vt03_flag_lf = 1;
      } else {
        GimbalState_ = kNoForce;
        vt03_flag_lf = 0;
      }
    }

    // 只有在左 Fn 开启手动控制后，右键才允许进入自瞄
    if (vt03_flag_lf) {
      if (vt03->data().mouse_button_right) {
        GimbalState_ = kAuto;
      } else {
        GimbalState_ = kManual;
      }
    }

    // 右 Fn：发射状态切换
    // kReady -> kFire -> kReady
    if (vt03->data().right_button && !vt03_last_fn_right) {
      if (AmmoState_ == kFire) {
        AmmoState_ = kReady;
        vt03_flag_rh = 0;
      } else {
        AmmoState_ = kFire;
        vt03_flag_rh = 1;
      }
    }

    vt03_last_fn_left = vt03->data().left_button;
    vt03_last_fn_right = vt03->data().right_button;
  }
  bool RcIsOnline() {  // 判断遥控器是否在线
    device_rc.Update();
    return rc->online_status() == rm::device::Device::kOk;
  }
  bool Vt03IsOnline() {  // 判断遥控器是否在线
    device_vt03.Update();
    return vt03->online_status() == rm::device::Device::kOk;
  }
  bool Rcchoose() {
    // 1标志vt03导出
    // 0标志rc导出
    if (Vt03IsOnline()) {  // 优先vt03导出键鼠数据
      return 1;
    }
    if (RcIsOnline()) {
      return 0;  // 在vt03断开数据且rc在线
    }
    return 1;  // 两者同时离线默认1
  }
  float GetYawMotorAngleRad() {  // 编码器返回角度
    return yaw_motor->encoder() * 2.0f * M_PI / 8192.0f;
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
    // // yaw
    // gimbal_controller.pid()
    //     .yaw_position.SetKp(300.0f)
    //     .SetKi(0.0f)
    //     .SetKd(12000.0f)
    //     .SetMaxOut(10000.0f)
    //     .SetMaxIout(1000.0f)
    //     .SetDiffLpfAlpha(0.01);
    // gimbal_controller.pid().yaw_speed.SetKp(350.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(25000.0f).SetMaxIout(1000.0f);
    // yaw
    gimbal_controller.pid()
        .yaw_position.SetKp(80.0f)
        .SetKi(0.0f)
        .SetKd(10000.0f)
        .SetMaxOut(10000.0f)
        .SetMaxIout(1000.0f)
        .SetDiffLpfAlpha(0.1);
    gimbal_controller.pid().yaw_speed.SetKp(500.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(25000.0f).SetMaxIout(1000.0f);
    // pitch
    gimbal_controller.pid()
        .pitch_position.SetKp(30.0f)
        .SetKi(0.0f)
        .SetKd(50.0f)
        .SetMaxOut(500.0f)
        .SetMaxIout(10.0f)
        .SetDiffLpfAlpha(0.01);
    gimbal_controller.pid().pitch_speed.SetKp(1.0f).SetKi(0.0f).SetKd(0.001f).SetMaxOut(10.0f).SetMaxIout(5.0f);
  }
  void AmmoPIDInit() {
    shoot_controller.pid().fric_1_speed.SetKp(18.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(20000.0f).SetMaxIout(1000.0f);
    shoot_controller.pid().fric_2_speed.SetKp(18.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(20000.0f).SetMaxIout(1000.0f);
    shoot_controller.pid().loader_speed.SetKp(15.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(20000.0f).SetMaxIout(2000.0f);
  }
  void GimbalControl() {
    if (GimbalState_ == kManual) {
      if (DM_is_enable == false) {
        pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
        DM_is_enable = true;
        gimbal_controller.Enable(true);
        rc_yaw_data = yaw_;      // 第一次进入更新当前位置
        rc_pitch_data = pitch_;  // 使用 IMU pitch 作为初始姿态

        rc_pitch_data = rm::modules::Clamp(rc_pitch_data, pitch_min_pos, pitch_max_pos);  // 对rc数据进行限位
      }
      yaw_relative = rm::modules::Wrap(GetYawMotorAngleRad() - yaw_center_encoder, -M_PI, M_PI);  // 相对机械中点误差
      yaw_delta = 0.0f;

      if (vt03->data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kCtrl)) {
        // CTRL held: 键盘控制(W/S/A/D), 遥控器和鼠标输入失效
        if (vt03->data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kW))
          rc_pitch_data -= 0.0001f;
        if (vt03->data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kS))
          rc_pitch_data += 0.0001f;
        if (vt03->data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kA)) yaw_delta += 0.0001f;
        if (vt03->data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kD)) yaw_delta -= 0.0001f;
      } else {
        if (Rcchoose()) {
          yaw_delta -= rm::modules::Map(vt03->data().left_y, -1, 1, -0.005f, 0.005f);         // vt03手控备份
          yaw_delta -= rm::modules::Map(vt03->data().mouse_x, -660, 660, -0.03f, 0.03f);      // vt03鼠标控制
          rc_pitch_data -= rm::modules::Map(vt03->data().left_x, -1, 1, -0.005f, 0.005f);     // vt03手控备份
          rc_pitch_data -= rm::modules::Map(vt03->data().mouse_y, -660, 660, -0.03f, 0.03f);  // vt03鼠标控制
        } else {
          yaw_delta -= rm::modules::Map(rc->left_x(), -660, 660, -0.005f, 0.005f);      // dt7手控
          yaw_delta -= rm::modules::Map(rc->mouse_x(), -660, 660, -0.03f, 0.03f);       // dt7备份控制
          rc_pitch_data += rm::modules::Map(rc->left_y(), -660, 660, -0.005f, 0.005f);  // dt7手控
          rc_pitch_data += rm::modules::Map(rc->mouse_y(), -660, 660, -0.03f, 0.03f);   // dt7备份控制
        }
      }

      if (yaw_relative >= yaw_max_limit && yaw_delta < 0.0f) {  // 机械限位返回逻辑
        yaw_delta = 0.0f;
      }
      if (yaw_relative <= yaw_min_limit && yaw_delta > 0.0f) {
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
      yaw_tau2voltage = tau_ff.x() * 2530.0f + Aimbot.YawSpeed * (60.0f / (2.0f * M_PI)) * 78.0f;  // 力矩转换控制电流

      // 设定目标，并计算
      gimbal_controller.SetTarget(roll_comp.first, roll_comp.second, 0, 0);
      gimbal_controller.Update(yaw_, -yaw_motor->rpm() * M_PI / 30.0, pitch_, -pitch_motor->vel(), 1.f);
      yaw_motor->SetCurrent(rm::modules::Clamp(-gimbal_controller.output().yaw - yaw_tau2voltage, -25000,
                                               25000));  // 设置输出电流并输出
      // yaw_motor->SetCurrent(rm::modules::Clamp(-gimbal_controller.output().yaw , -25000,
      //                                          25000));  // 设置输出电流并输出

    } else if (GimbalState_ == kAuto) {  // 自瞄模式控制
      if (DM_is_enable == false) {       // 使达妙电机使能
        pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
        DM_is_enable = true;
        gimbal_controller.Enable(true);
        rc_yaw_data = yaw_;
        rc_pitch_data = pitch_;  // 使用 IMU pitch 作为初始姿态
        rc_pitch_data = rm::modules::Clamp(rc_pitch_data, pitch_min_pos, pitch_max_pos);
      }
      if (Aimbot.AimbotState == 2 || Aimbot.AimbotState == 4) {
        rc_yaw_data = rm::modules::Wrap(Aimbot.TargetYawAngle, -M_PI, M_PI);

        rc_pitch_data = rm::modules::Clamp(Aimbot.TargetPitchAngle, pitch_min_pos, pitch_max_pos);
      } else {  // 非自瞄状态自动切入手控
        yaw_relative = rm::modules::Wrap(GetYawMotorAngleRad() - yaw_center_encoder, -M_PI, M_PI);  // 相对机械中点误差
        yaw_delta = 0.0f;
        if (Rcchoose()) {
          yaw_delta -= rm::modules::Map(vt03->data().left_y, -1, 1, -0.005f, 0.005f);         // vt03手控备份
          yaw_delta -= rm::modules::Map(vt03->data().mouse_x, -660, 660, -0.03f, 0.03f);      // vt03鼠标控制
          rc_pitch_data -= rm::modules::Map(vt03->data().left_x, -1, 1, -0.005f, 0.005f);     // vt03手控备份
          rc_pitch_data -= rm::modules::Map(vt03->data().mouse_y, -660, 660, -0.03f, 0.03f);  // vt03鼠标控制
        } else {
          yaw_delta -= rm::modules::Map(rc->left_x(), -660, 660, -0.005f, 0.005f);      // dt7手控
          yaw_delta -= rm::modules::Map(rc->mouse_x(), -660, 660, -0.03f, 0.03f);       // dt7备份控制
          rc_pitch_data += rm::modules::Map(rc->left_y(), -660, 660, -0.005f, 0.005f);  // dt7手控
          rc_pitch_data += rm::modules::Map(rc->mouse_y(), -660, 660, -0.03f, 0.03f);   // dt7备份控制
        }

        if (yaw_relative >= yaw_max_limit && yaw_delta < 0.0f) {  // 机械限位返回逻辑
          yaw_delta = 0.0f;
        }
        if (yaw_relative <= yaw_min_limit && yaw_delta > 0.0f) {
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

      // 设定目标，并计算
      gimbal_controller.SetTarget(roll_comp.first, roll_comp.second, 0, 0);
      gimbal_controller.Update(yaw_, -yaw_motor->rpm(), pitch_, -pitch_motor->vel(), 1.f);
      yaw_motor->SetCurrent(rm::modules::Clamp(-gimbal_controller.output().yaw - yaw_tau2voltage, -25000,
                                               25000));  // 设置输出电流并输出
    } else {                                             // 失能
      if (DM_is_enable == true) {
        pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
        DM_is_enable = false;
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

      if (rc->dial() >= 550 || rc->mouse_button_left() || vt03->data().mouse_button_left || vt03->data().trigger) {
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
      } else if (rc->dial() <= -600) {
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
  void ShootSpeedControl() {  // 弹速控制
    shoottime_--;
    if (shoottime_ < 0) {
      if ((vt03->data().keyboard_key & (1u << 11)) && (vt03->data().keyboard_key & (1u << 5))) {
        friction_speed -= shootstep;
        shootcnt += 1;
      } else if ((vt03->data().keyboard_key & (1u << 12)) && (vt03->data().keyboard_key & (1u << 5))) {
        friction_speed += shootstep;
        shootcnt -= 1;
      } else if (vt03->data().keyboard_key & (1u << 13) && (vt03->data().keyboard_key & (1u << 5))) {
        friction_speed = 6500;
        shootcnt = 0;
      }
      shoottime_ = shoottime;
    }
  }
  float SpeedAver() {
    float new_speed = referee_data_buffer.data().shoot_data.initial_speed;

    // 如果数据有效且与上次记录不同，则更新滑动窗口
    if (new_speed > 0 && new_speed != spaver[9]) {
      for (int i = 0; i < 9; i++) {
        spaver[i] = spaver[i + 1];
      }
      spaver[9] = new_speed;
    }

    // 计算平均值
    float sum = 0;
    int count = 0;
    for (int i = 0; i < 10; i++) {
      if (spaver[i] != 0) {
        sum += spaver[i];
        count++;
      }
    }
    return (count > 0) ? (sum / count) : 0.0f;
  }
  void WS2812Control() {
    auto key = vt03->data().keyboard_key;
    bool w_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kW);
    bool a_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kA);
    bool s_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kS);
    bool d_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kD);
    bool q_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kQ);
    bool e_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kE);
    bool shift_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kShift);
    bool ctrl_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kCtrl);
    bool z_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kZ);
    bool x_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kX);
    bool c_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kC);

    if (z_pressed && x_pressed && c_pressed) {
      if (led_blink_time < 5) {
        Set_LED(0, 255, 0, 0);
        Set_LED(1, 255, 0, 0);
        Set_LED(2, 255, 0, 0);
        Set_LED(3, 255, 0, 0);
      } else if (led_blink_time < 10) {
        Set_LED(0, 0, 0, 0);
        Set_LED(1, 0, 0, 0);
        Set_LED(2, 0, 0, 0);
        Set_LED(3, 0, 0, 0);
      } else
        led_blink_time = 0;
      led_blink_time++;
    }
    // 前进后退
    if (!ctrl_pressed && w_pressed && !s_pressed)
      Set_LED(1, 0, 255, 0);
    else if (!ctrl_pressed && !w_pressed && s_pressed)
      Set_LED(1, 255, 0, 0);
    else
      Set_LED(1, 255, 255, 0);

    // 左右or偏航
    if (!ctrl_pressed && a_pressed ^ d_pressed) {
      if (a_pressed) {
        Set_LED(0, 0, 0, 0);
        Set_LED(3, 255, 255, 255);
      } else if (d_pressed) {
        Set_LED(0, 255, 255, 255);
        Set_LED(3, 0, 0, 0);
      } else {
        Set_LED(0, 0, 0, 0);
        Set_LED(3, 0, 0, 0);
      }
    } else if (q_pressed ^ e_pressed) {
      if (q_pressed) {
        if (led_blink_time < 5)
          Set_LED(3, 255, 255, 255);
        else if (led_blink_time < 10)
          Set_LED(3, 0, 0, 0);
        else
          led_blink_time = 0;
        led_blink_time++;
        Set_LED(0, 0, 0, 0);
      } else if (e_pressed) {
        if (led_blink_time < 5)
          Set_LED(0, 255, 255, 255);
        else if (led_blink_time < 10)
          Set_LED(0, 0, 0, 0);
        else
          led_blink_time = 0;
        led_blink_time++;
        Set_LED(3, 0, 0, 0);
      }
    } else {
      Set_LED(0, 0, 0, 0);
      Set_LED(3, 0, 0, 0);
    }

    // 上升
    if (shift_pressed)
      Set_LED(2, 255, 255, 255);
    else
      Set_LED(2, 0, 0, 0);

    Set_Brightness(10);
    WS2812_Send();
  }
  bool ID() {
    if (referee_data_buffer.data().robot_status.robot_id != 0) {
      if (referee_data_buffer.data().robot_status.robot_id == 106) {
        ID_last = 1;
        return 1;  // 蓝方
      }
      ID_last = 0;
      return 0;  // 红方
    }
    return ID_last;
  }

  void SubLoop500Hz() {
    // ch040
    pitch_ = -imu_new->pitch();  // （上正下负）（+-pi）
    roll_ = -imu_new->roll();    //(左正右负)(+-pi)
    yaw_ = imu_new->yaw();       //(左正右负)（+-pi）

    GimbalImuSend(-imu_new->quat_x(), imu_new->quat_w(), imu_new->quat_z(), -imu_new->quat_y(), SpeedAver(),
                  referee_data_buffer.data().robot_status.robot_id);  // usb传输数据

    GimbalControl();                               // 云台控制更新
    AmmoControl();                                 // 发射机构更新
    rm::device::DjiMotorBase::SendCommand(*can1);  // 向大疆所有电机发数据
    rm::device::DjiMotorBase::SendCommand(*can2);  // 向大疆所有电机发数据
  }
  // DmMotor电机发信息
  void SubLoop250Hz() {
    if (time_ % 2 == 0) {
      if (!Rcchoose()) {
        RCStateUpdate();  // dt7控制更新
      } else {
        Vt03Control();  // vt03控制更新
      }
      // pitch负值向上输出
      pitch_torque = 1 * sin(pitch_ + 0.628);
      if (GimbalState_ == kManual) {
        pitch_cmd = rm::modules::Clamp(-gimbal_controller.output().pitch - pitch_torque, -10, 10);  // 发送达秒控制信息
      } else {
        pitch_cmd = rm::modules::Clamp(-gimbal_controller.output().pitch - tau_ff.y() - pitch_torque, -10,
                                       10);  // 发送达秒控制信息
      }
      pitch_motor->SetMitCommand(0, 0, pitch_cmd, 0, 0);  // 合输出
    }
  }
  void SubLoop100Hz() {
    if (time_ % 5 == 0) {
      ShootSpeedControl();  // 弹速手动控制
      FreemasterDebug();    // 调试更新
    }
  }

  void SubLoop50Hz() {
    if (time_ % 10 == 0) {
      // robot_id = referee_data_buffer.data().robot_status.robot_id;  // 裁判系统测试
    }
  }
  uint8_t test_ui_num = 0;
  void SubLoop10Hz() {
    if (time_ % 50 == 0) {
      test_ui_num++;
      WS2812Control();
      time_ = 0;
    }
  }
};

#endif  // BOARDC_GIMBAL_HPP
