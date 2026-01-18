#include "main.hpp"
#include "can.h"
#include "usart.h"
#include "spi.h"
#include "timer_task.hpp"
#include "tim.h"

// 底盘速度
rm::i16 Vx, Vy, Vw;
// 拨盘增加角度
float target_magz;
float target_velocity;
// 左摇杆状态
rm::device::DR16::SwitchPosition l_switch_position_now = rm::device::DR16::SwitchPosition::kUnknown;
rm::device::DR16::SwitchPosition l_switch_position_last = rm::device::DR16::SwitchPosition::kUnknown;

// 扳机状态
//  enum class Trigger_state:rm::usize {
//    kOff = 0u,
//    kOn,
//  };

// rm::usize trigger_now = 0u;
// rm::usize trigger_last = 0u;

float pos;

int counter = 0;

struct GlobalWarehouse {
  // 硬件接口 //
  rm::hal::Can *can1{nullptr}, *can2{nullptr};  ///< CAN 总线接口
  rm::hal::Serial *dbus{nullptr};               ///< 遥控器串口接口

  // 设备 //
  rm::device::DR16 *rc{nullptr};  ///< 遥控器
  // rm::device::GM6020 *yaw_motor{nullptr};                                              ///< 云台 Yaw 电机
  // rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *magazine_motor{nullptr};  ///< 云台 Pitch 电机
  rm::device::BMI088 *imu{nullptr};  ///< BMI088 IMU

  // 创建电机对象
  rm::device::M3508 *wheel_motor_1{nullptr};
  rm::device::M3508 *wheel_motor_2{nullptr};
  rm::device::M3508 *wheel_motor_3{nullptr};
  rm::device::M3508 *wheel_motor_4{nullptr};
  rm::device::M3508 *wheel_motor_5{nullptr};
  rm::device::M3508 *wheel_motor_6{nullptr};
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *magazine_motor{nullptr};

  // 创建PID控制器
  rm::modules::PID *pid_1{nullptr};
  rm::modules::PID *pid_2{nullptr};
  rm::modules::PID *pid_3{nullptr};
  rm::modules::PID *pid_4{nullptr};

  rm::modules::PID *pid_magz_position{nullptr};
  rm::modules::PID *pid_magz_velocity{nullptr};

  // 控制器 //
  rm::modules::MahonyAhrs ahrs{1000.f};  ///< mahony 姿态解算器，频率 1000Hz

  void Init() {
    can1 = new rm::hal::Can{hcan1};
    can2 = new rm::hal::Can{hcan2};
    dbus = new rm::hal::Serial{huart3, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
    // 遥控
    rc = new rm::device::DR16{*dbus};  // 设置了遥控器以及用了串口
    // 电机
    wheel_motor_1 = new rm::device::M3508{*can1, 1, false};
    wheel_motor_2 = new rm::device::M3508{*can1, 2, false};
    wheel_motor_3 = new rm::device::M3508{*can1, 3, false};
    wheel_motor_4 = new rm::device::M3508{*can1, 4, false};
    wheel_motor_5 = new rm::device::M3508{*can1, 5, false};
    wheel_motor_6 = new rm::device::M3508{*can1, 6, false};

    magazine_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
        *can2, {0x12, 0x03, 3.141593f, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}}};

    // PID控制器
    pid_1 = new rm::modules::PID{20, 2, 4, 30000, 2};
    pid_2 = new rm::modules::PID{20, 2, 4, 30000, 2};
    pid_3 = new rm::modules::PID{25, 2, 4, 30000, 2};
    pid_4 = new rm::modules::PID{25, 2, 4, 30000, 2};

    pid_magz_position = new rm::modules::PID{10, 0.1, 0.4, 20, 1};
    pid_magz_velocity = new rm::modules::PID{0.3, 0, 0.002, 10, 0};

    can1->SetFilter(0, 0);
    can1->Begin();
    can2->SetFilter(0, 0);
    can2->Begin();
    rc->Begin();  // 启动遥控器接收，这行或许比较适合放到AppMain里面？
  }
} *globals;

// 姿态解算任务
void MainLoop() {
  // 底盘电机反馈值
  int16_t rpm_1 = globals->wheel_motor_1->rpm();
  int16_t rpm_2 = globals->wheel_motor_2->rpm();
  int16_t rpm_3 = globals->wheel_motor_3->rpm();
  int16_t rpm_4 = globals->wheel_motor_4->rpm();

  // 遥控器输入底盘速度
  Vx = globals->rc->left_x() * 30000 / 660;
  Vy = globals->rc->left_y() * 30000 / 660;
  // Vw = globals->rc->dial() * 30000 / 660;

  // 移动逻辑
  // rm::i16 V_wheel_1 = -Vy + Vx - Vw;
  // rm::i16 V_wheel_2 = Vy + Vx - Vw;
  // rm::i16 V_wheel_3 = Vy - 0.5 * Vw;
  // rm::i16 V_wheel_4 = -Vy - 0.5 * Vw;
  rm::i16 V_wheel_1 = -Vy - Vx;
  rm::i16 V_wheel_2 = Vy - Vx;
  rm::i16 V_wheel_3 = Vy - 0.5 * Vx;
  rm::i16 V_wheel_4 = -Vy - 0.5 * Vx;
  if (globals->rc->switch_r() == rm::device::DR16::SwitchPosition::kDown) {
    globals->pid_1->out() == 0;
    globals->pid_2->out() == 0;
    globals->pid_3->out() == 0;
    globals->pid_4->out() == 0;

    // 给底盘电机发送指令
    globals->wheel_motor_1->SetCurrent(static_cast<int16_t>(globals->pid_1->out()));
    globals->wheel_motor_2->SetCurrent(static_cast<int16_t>(globals->pid_2->out()));
    globals->wheel_motor_3->SetCurrent(static_cast<int16_t>(globals->pid_3->out()));
    globals->wheel_motor_4->SetCurrent(static_cast<int16_t>(globals->pid_4->out()));
  } else {
    // 目标速度PID
    globals->pid_1->Update(V_wheel_1, rpm_1);
    globals->pid_2->Update(V_wheel_2, rpm_2);
    globals->pid_3->Update(V_wheel_3, rpm_3);
    globals->pid_4->Update(V_wheel_4, rpm_4);

    // 给底盘电机发送指令
    globals->wheel_motor_1->SetCurrent(static_cast<int16_t>(globals->pid_1->out()));
    globals->wheel_motor_2->SetCurrent(static_cast<int16_t>(globals->pid_2->out()));
    globals->wheel_motor_3->SetCurrent(static_cast<int16_t>(globals->pid_3->out()));
    globals->wheel_motor_4->SetCurrent(static_cast<int16_t>(globals->pid_4->out()));
  }

  rm::device::DjiMotor<>::SendCommand();

  // 拨盘电机逻辑
  l_switch_position_last = l_switch_position_now;
  l_switch_position_now = globals->rc->switch_l();
  if (l_switch_position_now == rm::device::DR16::SwitchPosition::kMid) {
    // 初始化射击模式
    if (l_switch_position_last != rm::device::DR16::SwitchPosition::kMid) {
      globals->magazine_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
      globals->magazine_motor->SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      target_magz = 0.0f;
    }
    // 按下扳机(延时1s)
    if (counter == 0) {
      if (globals->rc->dial() >= 500 || globals->rc->dial() < -500) {
        // 增加60°
        target_magz -= 1.0472 /*（π/3）*/;
        if (target_magz <= -3.141593 /*（π）*/) {
          target_magz += 3.141593 * 2;
        }
        counter = 250;
      }
    } else {
      counter--;
    }
    // 拨盘电机反馈
    pos = globals->magazine_motor->pos();        // rad
    float vel = globals->magazine_motor->vel();  // rad/s
    // 拨盘电机串级PID（开循环）
    globals->pid_magz_position->SetCircular(true).SetCircularCycle(3.141593 * 2);
    globals->pid_magz_position->Update(target_magz, pos, 0.002);
    target_velocity = globals->pid_magz_position->out();
    globals->pid_magz_velocity->Update(target_velocity, vel, 0.002);
    // 发送CAN
    if (globals->rc->switch_r() == rm::device::DR16::SwitchPosition::kDown) {
      globals->magazine_motor->SetPosition(0, 0, 0, 0, 0);
    } else {
      //
      globals->magazine_motor->SetPosition(0, 0, globals->pid_magz_velocity->out(), 0, 0);
    }
  }
  // 失能
  if (l_switch_position_last == rm::device::DR16::SwitchPosition::kMid &&
      l_switch_position_now != rm::device::DR16::SwitchPosition::kMid) {
    globals->magazine_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    globals->magazine_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    globals->magazine_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
  }
}

extern "C" [[noreturn]] void AppMain(void) {
  /*启动CAN总线
   *启动遥控器
   */
  globals = new GlobalWarehouse;
  globals->Init();

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_1000hz.SetPrescalerAndPeriod(168, 1000 - 1);  // 84MHz / 84 / 1000 = 1kHz
  mainloop_1000hz.Start();                               // 启动定时器

  for (;;) {
    __WFI();
  }
}