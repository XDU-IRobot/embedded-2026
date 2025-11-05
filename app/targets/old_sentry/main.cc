#include <librm.hpp>

#include "can.h"
#include "usart.h"
#include "spi.h"

#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "timer_task.hpp"
#include "device_manager.hpp"
#include "controllers/gimbal_double_yaw.hpp"
#include "controllers/shoot_3fric.hpp"

using namespace rm;

// 状态机
typedef enum {
  NO_FORCE = 0u,  // 无力模式
  TEST,           // 调试模式
  MATCH,          // 比赛模式

  GB_REMOTE,    // 云台遥控模式
  GB_NAVIGATE,  // 云台导航模式
  GB_SCAN,      // 扫描模式
  GB_AIMBOT,    // 云台自瞄模式

  CS_REMOTE,    // 底盘遥控模式
  CS_NAVIGATE,  // 底盘导航模式
} StateMachineType;

class RcTcRefereeData {
 public:
  RcTcRefereeData() = delete;

  explicit RcTcRefereeData(rm::hal::SerialInterface &serial);

  void Begin();

  void RxCallback(const std::vector<u8> &data, u16 rx_len);

 private:
  rm::hal::SerialInterface *serial_;
};

class Gimbal {
 public:
  StateMachineType GimbalMove_ = {NO_FORCE};  // 云台运动状态

 private:
  struct ChassisRequestState_t {
    i8 ChassisMoveXRequest;  // x轴运动控制
    i8 ChassisMoveYRequest;  // y轴运动控制
    u8 ChassisStateRequest;  // 底盘运动状态：无力，测试，随动，小陀螺正转，小陀螺反转
    u8 UiChange;             // 开启Ui
    u8 GetTargetFlag;        // 自瞄状态
    u8 SuggestFireFlag;      // 建议开火
    i8 AimSpeedChange;       // 转速等级
    i8 reserve[1];           // 保留位
  };

  f32 gimbal_yaw_target_ = 0.0f;    // 云台yaw轴遥控数据
  f32 gimbal_pitch_target_ = 0.0f;  // 云台pitch轴遥控数据

  i16 ammo_flag_rc_ = 0;  // 摩擦轮遥控数据位

  i8 aim_speed_change_ = 0;  // 摩擦轮转速改变值

  f32 ammo_left_speed_ = 0.0f;   // 左摩擦轮速度
  f32 ammo_right_speed_ = 0.0f;  // 右摩擦轮速度

  u16 heat_limit_ = 0;       // 热量上限值
  u16 heat_real_ = 0;        // 热量实时值
  i16 heat_last_ = 0;        // 上一次热量值
  i16 heat_delay_time_ = 0;  // 热量发送延迟时间

  i16 back_turn_time_ = 0;  // 拨盘反转时长
  i16 back_turn_flag_ = 0;  // 拨盘反转触发时长 100

  f32 rotor_position_ = 0.0f;         // 拨盘位置
  f32 last_rotor_position_ = 0.0f;    // 上一次拨盘位置
  f32 rotor_target_position_ = 0.0f;  // 拨盘目标位置
  u32 rotor_circle_flag_ = 0;         // 拨盘过圈标志

  u32 shoot_flag_ = 0;       // 开火标志
  u32 shoot_flag_last_ = 0;  // 上一次开火标志

  bool speed_change_flag_ = false;  // 速度模式改变标志

  bool single_wheel_flag_ = false;   // 单轮模式标志
  bool single_wheel_state_ = false;  // 单轮模式状态

  bool DMEnable_ = false;  // 4310电机使能标志

  bool DF_flag_ = false;   // 大符标志
  bool XF_flag_ = false;   // 小符标志
  bool DF_state_ = false;  // 大符状态
  bool XF_state_ = false;  // 小符状态

  const f32 once_circle_ = 17000.0f;          // 单圈编码值 17000f
  const f32 k_yaw_speed_ = 1.2f;              // 云台yaw轴速度前馈系数 1.2f
  const f32 k_yaw_current_ = -0.5f;           // 云台yaw轴电流前馈系数 -0.5f
  const f32 k_chassis_xy_rc_ = 100.0f;        // 底盘xy轴遥控前馈系数 100.0f
  const f32 ammo_init_speed_ = 6300.0f;       // 摩擦轮初始速度 6300.0f
  const f32 k_ammo_speed_change_ = 20.0f;     // 摩擦轮速度改变系数 20.0f
  const f32 rotor_position_dalte_ = 2000.0f;  // 拨盘位置误差值 2000.0f
  const f32 rotor_init_speed_[4] = {2000.0f, 2500.0f, 2000.0f, -1000.0f};
  // 拨盘初始速度速度 {低等级正转，高等级正转，单发，反转}{2000.0f, 2500.0f, 2000.0f, -1000.0f}
  const f32 sensitivity_x_ = 0.3f;           // 云台x轴灵敏度 0.3f
  const f32 sensitivity_y_ = 0.2f;           // 云台y轴灵敏度 0.2f
  const f32 k_mouse_sensitivity_x_ = 10.0f;  // 鼠标x轴灵敏系数 10.0f
  const f32 k_mouse_sensitivity_y_ = 10.0f;  // 鼠标y轴灵敏系数 10.0f
  const f32 highest_pitch_angle_ = 35.0f;    // 云台pitch轴最高 35.0f
  const f32 lowest_pitch_angle_ = 30.0f;     // 云台pitch轴最低 30.0f

 public:
  static void GimbalTask();

 private:
  static void GimbalInit();

  static void GimbalStateUpdate();

  static void GimbalRCDataUpdate();

  static void GimbalEnableUpdate();

  static void GimbalMatchUpdate();

  static void GimbalDisableUpdate();

  static void AmmoEnableUpdate();

  static void AmmoDisableUpdate();

  static void RotorEnableUpdate();

  static void RotorDisableUpdate();

  static void DaMiaoMotorEnable();

  static void DaMiaoMotorDisable();

  static void AimbotDateUpdate();

  static void MovePIDUpdate();
} *gimbal;

class Chassis {
 public:
  StateMachineType ChassisMove_ = {NO_FORCE};  // 底盘运动状态
 private:
  f32 chassis_x_rc_ = 0.0f;  // 底盘x轴遥控数据
  f32 chassis_y_rc_ = 0.0f;  // 底盘y轴遥控数据

 public:
 private:
  static void ChassisStateUpdate();

  static void ChassisEnableUpdate();

  static void ChassisMatchUpdate();

  static void ChassisDisableUpdate();
} *chassis;

struct GlobalWarehouse {
  AsyncBuzzer *buzzer{nullptr};  ///< 蜂鸣器
  LED *led{nullptr};             ///< RGB LED灯

  // 硬件接口 //
  rm::hal::Can *can1{nullptr}, *can2{nullptr};  ///< CAN 总线接口
  rm::hal::Serial *dbus{nullptr};               ///< 遥控器串口接口

  // 设备 //
  DeviceManager<10> device_manager;  ///< 设备管理器，维护所有设备在线状态
  rm::device::DR16 *rc{nullptr};     ///< 遥控器
  rm::device::Referee<rm::device::RefereeRevision::kV170> referee_data_buffer;
  rm::device::GM6020 *up_yaw_motor{nullptr};                                           ///< 云台 Yaw 上电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *down_yaw_motor{nullptr};  ///< 云台 Yaw 下电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr};     ///< 云台 Pitch 电机
  rm::device::BMI088 *imu{nullptr};

  // 控制器 //
  GimbalDoubleYaw gimbal_controller;  ///< 二轴双 Yaw 云台控制器
  Shoot3Fric shoot_controller{9};     ///< 三摩擦轮发射机构控制器，9发拨盘
  rm::modules::MahonyAhrs ahrs{1000.0f};

  // 常、变量 //
  const f32 yaw_gyro_bias_ = 0.0015f;  // 偏航角（角度值）的陀螺仪偏移量
  int DM_enable_flag_ = 0;
  StateMachineType StateMachine_ = {NO_FORCE};  // 当前状态

  // 函数 //
  void Init();

  void GimbalPIDInit();

  void RCStateUpdate();
} *globals;

void MainLoop() {
  globals->imu->Update();
  globals->ahrs.Update(  //
      rm::modules::ImuData6Dof{globals->imu->gyro_y(), globals->imu->gyro_z(),
                               globals->imu->gyro_x() + globals->yaw_gyro_bias_, globals->imu->accel_y(),
                               globals->imu->accel_z(), globals->imu->accel_x()});
  if (globals->DM_enable_flag_ == 0) {
    // 使达妙电机使能
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    globals->DM_enable_flag_ = 1;
  } else {
    globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
  }
}

extern "C" [[noreturn]] void AppMain(void) {
  globals = new GlobalWarehouse;
  gimbal = new Gimbal;
  chassis = new Chassis;
  globals->Init();

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_1000hz.SetPrescalerAndPeriod(84 - 1, 1000 - 1);  // 84MHz / 84 / 1000 = 1kHz
  mainloop_1000hz.Start();

  globals->buzzer->Beep(2, 40);
  (*globals->led)(0xff00ff00);

  for (;;) {
    __WFI();
  }
}

void GlobalWarehouse::Init() {
  buzzer = new AsyncBuzzer;
  led = new LED;

  can1 = new rm::hal::Can{hcan1};
  can2 = new rm::hal::Can{hcan2};
  dbus = new rm::hal::Serial{huart3, 18, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};

  rc = new rm::device::DR16{*dbus};
  up_yaw_motor = new rm::device::GM6020{*can1, 5};
  down_yaw_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
      *can2, {0x05, 0x04, 12.5f, 30.0f, 10.0f, std::make_pair(0.0f, 500.0f), std::make_pair(0.0f, 5.0f)}};
  pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
      *can1, {0x03, 0x02, 12.5f, 30.0f, 10.0f, std::make_pair(0.0f, 500.0f), std::make_pair(0.0f, 5.0f)}};
  imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};

  device_manager << rc << up_yaw_motor << down_yaw_motor << pitch_motor;

  can1->SetFilter(0, 0);
  can1->Begin();
  can2->SetFilter(0, 0);
  can2->Begin();
  rc->Begin();
  buzzer->Init();
  led->Init();

  GimbalPIDInit();
}

void GlobalWarehouse::GimbalPIDInit() {
  // 初始化PID
  // 上部 Yaw PID 参数
  gimbal_controller.pid().up_yaw_position.SetKp(15.0f);  // 位置环
  gimbal_controller.pid().up_yaw_position.SetKi(0.0f);
  gimbal_controller.pid().up_yaw_position.SetKd(2000.0f);
  gimbal_controller.pid().up_yaw_position.SetMaxOut(15000.0f);
  gimbal_controller.pid().up_yaw_position.SetMaxIout(360.0f);
  gimbal_controller.pid().up_yaw_speed.SetKp(220.0f);  // 速度环
  gimbal_controller.pid().up_yaw_speed.SetKi(0.0f);
  gimbal_controller.pid().up_yaw_speed.SetKd(0.0f);
  gimbal_controller.pid().up_yaw_speed.SetMaxOut(25000.0f);
  gimbal_controller.pid().up_yaw_speed.SetMaxIout(360.0f);
  // 下部 Yaw PID 参数
  gimbal_controller.pid().down_yaw_position.SetKp(15.0f);  // 位置环
  gimbal_controller.pid().down_yaw_position.SetKi(0.0f);
  gimbal_controller.pid().down_yaw_position.SetKd(2000.0f);
  gimbal_controller.pid().down_yaw_position.SetMaxOut(15000.0f);
  gimbal_controller.pid().down_yaw_position.SetMaxIout(360.0f);
  gimbal_controller.pid().down_yaw_speed.SetKp(220.0f);  // 速度环
  gimbal_controller.pid().down_yaw_speed.SetKi(0.0f);
  gimbal_controller.pid().down_yaw_speed.SetKd(0.0f);
  gimbal_controller.pid().down_yaw_speed.SetMaxOut(25000.0f);
  gimbal_controller.pid().down_yaw_speed.SetMaxIout(360.0f);
  // pitch PID 参数
  gimbal_controller.pid().pitch_position.SetKp(15.0f);  // 位置环
  gimbal_controller.pid().pitch_position.SetKi(0.0f);
  gimbal_controller.pid().pitch_position.SetKd(2000.0f);
  gimbal_controller.pid().pitch_position.SetMaxOut(15000.0f);
  gimbal_controller.pid().pitch_position.SetMaxIout(360.0f);
  gimbal_controller.pid().pitch_speed.SetKp(220.0f);  // 速度环
  gimbal_controller.pid().pitch_speed.SetKi(0.0f);
  gimbal_controller.pid().pitch_speed.SetKd(0.0f);
  gimbal_controller.pid().pitch_speed.SetMaxOut(25000.0f);
  gimbal_controller.pid().pitch_speed.SetMaxIout(360.0f);
}

RcTcRefereeData::RcTcRefereeData(rm::hal::SerialInterface &serial) : serial_(&serial) {
  static rm::hal::SerialRxCallbackFunction rx_callback =
      std::bind(&RcTcRefereeData::RxCallback, this, std::placeholders::_1, std::placeholders::_2);
  this->serial_->AttachRxCallback(rx_callback);
}

void RcTcRefereeData::Begin() { this->serial_->Begin(); }

void RcTcRefereeData::RxCallback(const std::vector<u8> &data, u16 rx_len) {
  for (u16 i = 0; i < rx_len; i++) {
    globals->referee_data_buffer << data.at(i);
  }
}

void GlobalWarehouse::RCStateUpdate() {
  if (globals->referee_data_buffer.data().robot_status.power_management_gimbal_output == 0) {
    globals->StateMachine_ = NO_FORCE;
  } else {
    switch (globals->rc->switch_r()) {
      case rm::device::DR16::SwitchPosition::kUp:
        // 右拨杆打到最上侧挡位
        if (globals->rc->switch_r() == rm::device::DR16::SwitchPosition::kUp) {
          globals->StateMachine_ = NO_FORCE;
        } else if (globals->rc->switch_r() == rm::device::DR16::SwitchPosition::kMid) {
          globals->StateMachine_ = NO_FORCE;
        } else {
          globals->StateMachine_ = MATCH;  // 左拨杆拨到下侧，进入比赛模式，此时全部系统都上电工作
        }
        break;

      case rm::device::DR16::SwitchPosition::kMid:
        // 右拨杆打到中间挡位
        if (globals->rc->switch_r() == rm::device::DR16::SwitchPosition::kUp) {
          globals->StateMachine_ = TEST;
          gimbal->GimbalMove_ = GB_AIMBOT;
          chassis->ChassisMove_ = CS_REMOTE;
        } else if (globals->rc->switch_r() == rm::device::DR16::SwitchPosition::kMid) {
          globals->StateMachine_ = TEST;
          gimbal->GimbalMove_ = GB_NAVIGATE;
          chassis->ChassisMove_ = CS_NAVIGATE;
        } else {
          globals->StateMachine_ = TEST;  // 左拨杆拨到下侧，进入测试模式
          gimbal->GimbalMove_ = GB_REMOTE;
          chassis->ChassisMove_ = CS_REMOTE;
        }
        break;

      case rm::device::DR16::SwitchPosition::kDown:
        // 右拨杆打到最下侧挡位，此时全部全部无力
        globals->StateMachine_ = NO_FORCE;
        break;

      default:
        globals->StateMachine_ = NO_FORCE;  // 如果遥控器离线，进入无力模式
        break;
    }
  }
}

void Gimbal::GimbalTask() {}

void Gimbal::GimbalInit() {
  gimbal->gimbal_yaw_target_ = globals->ahrs.euler_angle().yaw;      // 云台yaw初始化
  gimbal->gimbal_pitch_target_ = globals->ahrs.euler_angle().pitch;  // 云台pitch初始化
}

void Gimbal::GimbalStateUpdate() {
  switch (globals->StateMachine_) {
    case NO_FORCE:            // 无力模式下，所有电机失能
      GimbalDisableUpdate();  // 云台电机失能计算
      AmmoDisableUpdate();    // 摩擦轮机构失能计算
      RotorDisableUpdate();   // 拨盘失能计算
      break;

    case TEST:  // 测试模式下，发射系统与拨盘电机失能
      switch (gimbal->GimbalMove_) {
        case GB_REMOTE:
          GimbalEnableUpdate();  // 云台电机使能计算
          AmmoDisableUpdate();   // 摩擦轮机构失能计算
          RotorDisableUpdate();  // 拨盘失能计算
          break;

        case GB_AIMBOT:
        case GB_SCAN:
          GimbalEnableUpdate();  // 云台电机使能计算
          AmmoEnableUpdate();    // 摩擦轮机构使能计算
          RotorEnableUpdate();   // 拨盘使能计算
          break;

        default:
          GimbalDisableUpdate();  // 云台电机失能计算
          AmmoDisableUpdate();    // 摩擦轮机构失能计算
          RotorDisableUpdate();   // 拨盘失能计算
          break;
      }
      break;

    case MATCH:             // 比赛模式下，所有电机正常工作
      GimbalMatchUpdate();  // 云台电机使能计算
      // AmmoEnableUpdate();                  // 摩擦轮机构使能计算
      // RotorEnableUpdate();                 // 拨盘使能计算
      break;

    default:                  // 错误状态，所有电机失能
      GimbalDisableUpdate();  // 云台电机失能计算
      AmmoDisableUpdate();    // 摩擦轮机构失能计算
      RotorDisableUpdate();   // 拨盘失能计算
      break;
  }
}

void Gimbal::GimbalRCDataUpdate() {
  gimbal->gimbal_yaw_target_ -= rm::modules::Map(globals->rc->left_x(), -660, 660,  //
                                                 -gimbal->sensitivity_x_, gimbal->sensitivity_x_);
  gimbal->gimbal_pitch_target_ -= rm::modules::Map(globals->rc->left_y(), -660, 660,  //
                                                   -gimbal->sensitivity_y_, gimbal->sensitivity_y_);
  gimbal->gimbal_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_yaw_target_, 0.0f, 360.0f);  // yaw轴周期限制
  gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_, -gimbal->highest_pitch_angle_,
                                                    gimbal->lowest_pitch_angle_);  // pitch轴限位
}

void Gimbal::GimbalEnableUpdate() {
  DaMiaoMotorEnable();
  GimbalRCDataUpdate();
  // // GimabalImu.mode = 0x00;
  globals->gimbal_controller.Enable(true);
  if (gimbal->GimbalMove_ == GB_REMOTE) {
    globals->gimbal_controller.SetTarget(2600.0f, gimbal->gimbal_yaw_target_, gimbal->gimbal_pitch_target_);
    globals->gimbal_controller.Update(globals->up_yaw_motor->encoder(), globals->up_yaw_motor->rpm(),
                                      globals->down_yaw_motor->pos(), globals->down_yaw_motor->vel(),
                                      globals->pitch_motor->pos(), globals->pitch_motor->vel());
  } else if (gimbal->GimbalMove_ == GB_SCAN) {
  } else if (gimbal->GimbalMove_ == GB_AIMBOT) {
  } else {
  }
}

void Gimbal::GimbalMatchUpdate() {}

void Gimbal::GimbalDisableUpdate() {}

void Gimbal::AmmoEnableUpdate() {}

void Gimbal::AmmoDisableUpdate() {}

void Gimbal::RotorEnableUpdate() {}

void Gimbal::RotorDisableUpdate() {}

void Gimbal::DaMiaoMotorEnable() {}

void Gimbal::DaMiaoMotorDisable() {}

void Gimbal::AimbotDateUpdate() {
  // if ((Aimbot.AimbotState >> 0) & 0x01) {
  //     gimbal->gimbal_yaw_rc = Aimbot.Yaw;
  //     gimbal->gimbal_pitch_rc = Aimbot.Pitch;
  // }
}

void Gimbal::MovePIDUpdate() {
  // gimbal->gimbal_yaw_rc = LoopConstrain(gimbal->gimbal_yaw_rc, 0.0f, 360.0f);  // yaw轴周期限制
  // // gimbal->gimbal_yaw_rc =
  // //     LoopConstrain(gimbal->gimbal_yaw_rc, (f32)gimbal->down_yaw_motor.encoder() / 8191.0f * 360.0f - 45.0f,
  // //                   (f32)gimbal->up_yaw_motor.encoder() / 8191.0f * 360.0f + 45.0f);  // yaw轴周期限制
  // gimbal->gimbal_pitch_rc = Constrain(gimbal->gimbal_pitch_rc, -35.0f, 28.0f);  // pitch轴限位
  // gimbal->up_yaw_pid_position_.Update(gimbal->gimbal_yaw_rc, yaw);
  // gimbal->up_yaw_pid_speed_.Update(gimbal->up_yaw_pid_position_.value() + kmove_yaw_speed *
  // gimbal->up_yaw_motor.rpm(),
  //                                  gimbal->up_yaw_motor.rpm());
  // gimbal->pitch_pid_position_.Update(-gimbal->gimbal_pitch_rc, -pitch);
  // gimbal->pitch_pid_speed_.Update(gimbal->pitch_pid_position_.value(), pitch_motor_->vel());
}

void Chassis::ChassisStateUpdate() {
  switch (globals->StateMachine_) {
    case NO_FORCE:
      ChassisDisableUpdate();  // 底盘电机失能计算
      break;

    case TEST:
      switch (chassis->ChassisMove_) {
        case CS_REMOTE:
        case CS_NAVIGATE:
          ChassisEnableUpdate();  // 底盘电机使能计算
          break;

        default:                   // 错误状态，所有电机失能
          ChassisDisableUpdate();  // 底盘电机失能计算
          break;
      }
      break;

    case MATCH:              // 比赛模式下，所有电机正常工作
      ChassisMatchUpdate();  // 底盘电机使能计算
      break;

    default:                   // 错误状态，所有电机失能
      ChassisDisableUpdate();  // 底盘电机失能计算
      break;
  }
}

void Chassis::ChassisEnableUpdate() {}

void Chassis::ChassisMatchUpdate() {}

void Chassis::ChassisDisableUpdate() {}
