#include "main.hpp"
#include "timer_task.hpp"
#include "tim.h"

extern "C" [[noreturn]] void AppMain(void) {
  /*启动CAn总线
   *启动遥控器
   */
  globals = new GlobalWarehouse;
  globals->Init();

  // 创建电机对象
  rm::device::M3508 wheel_motor_1(*(globals->can1), 1, false);
  rm::device::M3508 wheel_motor_2(*(globals->can1), 2, false);
  rm::device::M3508 wheel_motor_3(*(globals->can1), 3, false);
  rm::device::M3508 wheel_motor_4(*(globals->can1), 4, false);

  // 创建PID控制器
  rm::modules::PID pid_1(5, 0, 0, 30000, 0);
  rm::modules::PID pid_2(5, 0, 0, 30000, 0);
  rm::modules::PID pid_3(5, 0, 0, 30000, 0);
  rm::modules::PID pid_4(5, 0, 0, 30000, 0);

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_1000hz.SetPrescalerAndPeriod(84 - 1, 1000 - 1);  // 84MHz / 84 / 1000 = 1kHz
  mainloop_1000hz.Start();                                  // 启动定时器

  // 底盘速度
  rm::i16 Vx, Vy, Vw;

  for (;;) {
    // 底盘电机反馈值
    int16_t rpm_1 = wheel_motor_1.rpm();
    int16_t rpm_2 = wheel_motor_2.rpm();
    int16_t rpm_3 = wheel_motor_3.rpm();
    int16_t rpm_4 = wheel_motor_4.rpm();

    // 遥控器输入底盘速度
    Vx = globals->rc->left_x();
    Vy = globals->rc->left_y();
    Vw = globals->rc->dial();

    // 底盘电机PID
    rm::i16 V_wheel_1 = -Vy + Vx - Vw;
    rm::i16 V_wheel_2 = Vy - Vx + Vw;
    rm::i16 V_wheel_3 = Vy + 0.5 * Vw;
    rm::i16 V_wheel_4 = Vy - 0.5 * Vw;

    pid_1.Update(V_wheel_1, rpm_1);
    pid_2.Update(V_wheel_2, rpm_2);
    pid_3.Update(V_wheel_3, rpm_3);
    pid_4.Update(V_wheel_4, rpm_4);

    // 给底盘电机发送指令
    wheel_motor_1.SetCurrent(static_cast<int16_t>(pid_1.out()));
    wheel_motor_2.SetCurrent(static_cast<int16_t>(pid_2.out()));
    wheel_motor_3.SetCurrent(static_cast<int16_t>(pid_3.out()));
    wheel_motor_4.SetCurrent(static_cast<int16_t>(pid_4.out()));

    rm::device::DjiMotor<>::SendCommand();
  }
}