#include "main.hpp"
#include "can.h"
#include "usart.h"
#include "spi.h"
#include "timer_task.hpp"
#include "tim.h"

struct GlobalWarehouse {

  // 硬件接口 //
  rm::hal::Can *can1{nullptr}, *can2{nullptr};  ///< CAN 总线接口
  rm::hal::Serial *dbus{nullptr};               ///< 遥控器串口接口

  // 设备 //
  rm::device::DR16 *rc{nullptr};           ///< 遥控器
  rm::device::GM6020 *yaw_motor{nullptr};  ///< 云台 Yaw 电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *magazine_motor{nullptr};  ///< 云台 Pitch 电机
  rm::device::BMI088 *imu{nullptr};                                                 ///< BMI088 IMU

  // 控制器 //
  rm::modules::MahonyAhrs ahrs{1000.f};  ///< mahony 姿态解算器，频率 1000Hz

  void Init() {

    can1 = new rm::hal::Can{hcan1};
    can2 = new rm::hal::Can{hcan2};
    dbus = new rm::hal::Serial{huart3, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};

    rc = new rm::device::DR16{*dbus};//设置了遥控器以及用了串口
    yaw_motor = new rm::device::GM6020{*can1, 1};
    magazine_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{*can2, {}};
    imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};


    can1->SetFilter(0, 0);
    can1->Begin();
    can2->SetFilter(0, 0);
    can2->Begin();
    rc->Begin();  // 启动遥控器接收，这行或许比较适合放到AppMain里面？
  }
} *globals;

//姿态解算任务
void MainLoop() {
  globals->imu->Update();
  globals->ahrs.Update(rm::modules::ImuData6Dof{-globals->imu->gyro_y(),   //
                                                -globals->imu->gyro_x(),   //
                                                -globals->imu->gyro_z(),   //
                                                -globals->imu->accel_y(),  //
                                                -globals->imu->accel_x(),  //
                                                -globals->imu->accel_z()});
}

extern "C" [[noreturn]] void AppMain(void) {
  /*启动CAn总线
   *启动遥控器
   */
  globals = new GlobalWarehouse;
  globals->Init();

  //创建电机对象
  rm::device::M3508 wheel_motor_1(*(globals->can1),1,false);
  rm::device::M3508 wheel_motor_2(*(globals->can1),2,false);
  rm::device::M3508 wheel_motor_3(*(globals->can1),3,false);
  rm::device::M3508 wheel_motor_4(*(globals->can1),4,false);

  //创建PID控制器
  rm::modules::PID pid(5,0,0,30000,0);

  //创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
    &htim13,                                   //
    etl::delegate<void()>::create<MainLoop>()  //
};
  mainloop_1000hz.SetPrescalerAndPeriod(84 - 1, 1000 - 1);  // 84MHz / 84 / 1000 = 1kHz
  mainloop_1000hz.Start();//启动定时器

  //创建底盘速度
  float Vx=globals->rc->left_x();
  float Vy=globals->rc->left_y();

  for (;;) {
    //底盘电机PID
    float V_wheel_1=Vy-Vx;
    float V_wheel_2=Vy+Vx;
    float V_wheel_3=Vy-0.5*Vx;
    float V_wheel_4=Vy+0.5*Vx;
    pid.Update(globals->rc.)


    rm::device::DjiMotor<>::SendCommand();

    //底盘电机反馈值
    int16_t rpm_1 = wheel_motor_1.rpm();
    int16_t rpm_2 = wheel_motor_2.rpm();
    int16_t rpm_3 = wheel_motor_3.rpm();
    int16_t rpm_4 = wheel_motor_4.rpm();

  }
}