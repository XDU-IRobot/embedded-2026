#include <librm.hpp>

#include "can.h"
#include "usart.h"
#include "spi.h"

#include "timer_task.hpp"

#include "main.hpp"
#include "Gimbal.hpp"
#include "Chassis.hpp"
#include "Referee.hpp"

using namespace rm;

void MainLoop() {
    globals->imu->Update();
    globals->ahrs.Update(rm::modules::ImuData6Dof{
        globals->imu->gyro_y(), globals->imu->gyro_z(), globals->imu->gyro_x() + gimbal->yaw_gyro_bias_,
        globals->imu->accel_y(), globals->imu->accel_z(), globals->imu->accel_x()
    });
    GimbalImuSend();
    globals->RCStateUpdate();
    gimbal->GimbalTask();
    chassis->ChassisTask();
    rm::device::DjiMotor<>::SendCommand(); // 发送电流
}

extern "C" [[noreturn]] void AppMain(void) {
    globals = new GlobalWarehouse;
    gimbal = new Gimbal;
    chassis = new Chassis;
    globals->Init();
    rm::hal::Serial referee_uart(huart6, 128, hal::stm32::UartMode::kDma, hal::stm32::UartMode::kDma);
    RcTcRefereeData rcdata(referee_uart);
    rcdata.Begin();

    // 创建主循环定时任务，定频1khz
    TimerTask mainloop_1000hz{
        &htim13, //
        etl::delegate<void()>::create<MainLoop>() //
    };
    mainloop_1000hz.SetPrescalerAndPeriod(84 - 1, 1000 - 1); // 84MHz / 84 / 1000 = 1kHz
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
    imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};

    rc = new rm::device::DR16{*dbus};
    up_yaw_motor = new rm::device::GM6020{*can1, 5};
    down_yaw_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
        *can2, {0x05, 0x04, 12.5f, 30.0f, 10.0f, std::make_pair(0.0f, 500.0f), std::make_pair(0.0f, 5.0f)}
    };
    pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
        *can1, {0x03, 0x02, 12.5f, 30.0f, 10.0f, std::make_pair(0.0f, 500.0f), std::make_pair(0.0f, 5.0f)}
    };

    steer_lf = new rm::device::GM6020{*can2, 3};
    steer_rf = new rm::device::GM6020{*can2, 1};
    steer_lb = new rm::device::GM6020{*can2, 4};
    steer_rb = new rm::device::GM6020{*can2, 2};
    wheel_lf = new rm::device::M3508{*can2, 1};
    wheel_rf = new rm::device::M3508{*can2, 3};
    wheel_lb = new rm::device::M3508{*can2, 2};
    wheel_rb = new rm::device::M3508{*can2, 4};

    device_manager << rc // 遥控器
            << up_yaw_motor << down_yaw_motor << pitch_motor // 云台电机
            << friction_left << friction_right << dial_motor // 发射机构电机
            << steer_lf << steer_rf << steer_lb << steer_rb // 底盘舵电机
            << wheel_lf << wheel_rf << wheel_lb << wheel_rb; // 底盘轮电机

    can1->SetFilter(0, 0);
    can1->Begin();
    can2->SetFilter(0, 0);
    can2->Begin();
    rc->Begin();
    buzzer->Init();
    led->Init();

    globals->GimbalPIDInit();
    globals->ChassisPIDInit();
    gimbal->GimbalInit();
    chassis->ChassisInit();
}

void GlobalWarehouse::GimbalPIDInit() {
    // 初始化PID
    // 上部 Yaw PID 参数
    gimbal_controller.pid().up_yaw_position.SetKp(0.10f); // 位置环 0.1f 0.0f 0.01f
    gimbal_controller.pid().up_yaw_position.SetKi(0.0f);
    gimbal_controller.pid().up_yaw_position.SetKd(0.0f);
    gimbal_controller.pid().up_yaw_position.SetMaxOut(35000.0f);
    gimbal_controller.pid().up_yaw_position.SetMaxIout(0.0f);
    gimbal_controller.pid().up_yaw_speed.SetKp(400.0f); // 速度环 400.0f 0.0f 320.0f
    gimbal_controller.pid().up_yaw_speed.SetKi(0.0f);
    gimbal_controller.pid().up_yaw_speed.SetKd(320.0f);
    gimbal_controller.pid().up_yaw_speed.SetMaxOut(16384.0f);
    gimbal_controller.pid().up_yaw_speed.SetMaxIout(0.0f);
    // 下部 Yaw PID 参数
    gimbal_controller.pid().down_yaw_position.SetKp(20.0f); // 位置环 20.0f 0.0f 3000.0f
    gimbal_controller.pid().down_yaw_position.SetKi(0.0f);
    gimbal_controller.pid().down_yaw_position.SetKd(3000.0f);
    gimbal_controller.pid().down_yaw_position.SetMaxOut(10000.0f);
    gimbal_controller.pid().down_yaw_position.SetMaxIout(0.0f);
    gimbal_controller.pid().down_yaw_speed.SetKp(1.5f); // 速度环 1.5f 0.0f 3.2f
    gimbal_controller.pid().down_yaw_speed.SetKi(0.0f);
    gimbal_controller.pid().down_yaw_speed.SetKd(3.2f);
    gimbal_controller.pid().down_yaw_speed.SetMaxOut(10.0f);
    gimbal_controller.pid().down_yaw_speed.SetMaxIout(0.0f);
    // pitch PID 参数
    gimbal_controller.pid().pitch_position.SetKp(15.0f); // 位置环 15.0f 0.0f 80.0f
    gimbal_controller.pid().pitch_position.SetKi(0.0f);
    gimbal_controller.pid().pitch_position.SetKd(60.0f);
    gimbal_controller.pid().pitch_position.SetMaxOut(10000.0f);
    gimbal_controller.pid().pitch_position.SetMaxIout(0.0f);
    gimbal_controller.pid().pitch_speed.SetKp(1.2f); // 速度环 1.2f 0.0f 1.0f
    gimbal_controller.pid().pitch_speed.SetKi(0.0f);
    gimbal_controller.pid().pitch_speed.SetKd(1.0f);
    gimbal_controller.pid().pitch_speed.SetMaxOut(10.0f);
    gimbal_controller.pid().pitch_speed.SetMaxIout(0.0f);
}

void GlobalWarehouse::ChassisPIDInit() {
    chassis_controller.pid().lf_steer_position.SetKp(50.0f); // 位置环 0.0f 0.0f 0.0f
    chassis_controller.pid().lf_steer_position.SetKi(0.0f);
    chassis_controller.pid().lf_steer_position.SetKd(1.0f);
    chassis_controller.pid().lf_steer_position.SetMaxOut(10000.0f);
    chassis_controller.pid().lf_steer_position.SetMaxIout(0.0f);
    chassis_controller.pid().lf_steer_speed.SetKp(300.0f); // 速度环 0.0f 0.0f 0.0f
    chassis_controller.pid().lf_steer_speed.SetKi(0.0f);
    chassis_controller.pid().lf_steer_speed.SetKd(1.0f);
    chassis_controller.pid().lf_steer_speed.SetMaxOut(10000.0f);
    chassis_controller.pid().lf_steer_speed.SetMaxIout(0.0f);
    chassis_controller.pid().rf_steer_position.SetKp(50.0f); // 位置环 0.0f 0.0f 0.0f
    chassis_controller.pid().rf_steer_position.SetKi(0.0f);
    chassis_controller.pid().rf_steer_position.SetKd(1.0f);
    chassis_controller.pid().rf_steer_position.SetMaxOut(10000.0f);
    chassis_controller.pid().rf_steer_position.SetMaxIout(0.0f);
    chassis_controller.pid().rf_steer_speed.SetKp(300.0f); // 速度环 0.0f 0.0f 0.0f
    chassis_controller.pid().rf_steer_speed.SetKi(0.0f);
    chassis_controller.pid().rf_steer_speed.SetKd(1.0f);
    chassis_controller.pid().rf_steer_speed.SetMaxOut(10000.0f);
    chassis_controller.pid().rf_steer_speed.SetMaxIout(0.0f);
    chassis_controller.pid().lb_steer_position.SetKp(50.0f); // 位置环 0.0f 0.0f 0.0f
    chassis_controller.pid().lb_steer_position.SetKi(0.0f);
    chassis_controller.pid().lb_steer_position.SetKd(1.0f);
    chassis_controller.pid().lb_steer_position.SetMaxOut(10000.0f);
    chassis_controller.pid().lb_steer_position.SetMaxIout(0.0f);
    chassis_controller.pid().lb_steer_speed.SetKp(300.0f); // 速度环 0.0f 0.0f 0.0f
    chassis_controller.pid().lb_steer_speed.SetKi(0.0f);
    chassis_controller.pid().lb_steer_speed.SetKd(1.0f);
    chassis_controller.pid().lb_steer_speed.SetMaxOut(10000.0f);
    chassis_controller.pid().lb_steer_speed.SetMaxIout(0.0f);
    chassis_controller.pid().rb_steer_position.SetKp(50.0f); // 位置环 0.0f 0.0f 0.0f
    chassis_controller.pid().rb_steer_position.SetKi(0.0f);
    chassis_controller.pid().rb_steer_position.SetKd(1.0f);
    chassis_controller.pid().rb_steer_position.SetMaxOut(10000.0f);
    chassis_controller.pid().rb_steer_position.SetMaxIout(0.0f);
    chassis_controller.pid().rb_steer_speed.SetKp(300.0f); // 速度环 0.0f 0.0f 0.0f
    chassis_controller.pid().rb_steer_speed.SetKi(0.0f);
    chassis_controller.pid().rb_steer_speed.SetKd(1.0f);
    chassis_controller.pid().rb_steer_speed.SetMaxOut(10000.0f);
    chassis_controller.pid().rb_steer_speed.SetMaxIout(0.0f);
    chassis_controller.pid().lf_wheel.SetKp(3.0f); // 速度环 0.0f 0.0f 0.0f
    chassis_controller.pid().lf_wheel.SetKi(0.0f);
    chassis_controller.pid().lf_wheel.SetKd(0.5f);
    chassis_controller.pid().lf_wheel.SetMaxOut(6000.0f);
    chassis_controller.pid().lf_wheel.SetMaxIout(0.0f);
    chassis_controller.pid().rf_wheel.SetKp(3.0f); // 速度环 0.0f 0.0f 0.0f
    chassis_controller.pid().rf_wheel.SetKi(0.0f);
    chassis_controller.pid().rf_wheel.SetKd(0.5f);
    chassis_controller.pid().rf_wheel.SetMaxOut(6000.0f);
    chassis_controller.pid().rf_wheel.SetMaxIout(0.0f);
    chassis_controller.pid().lb_wheel.SetKp(3.0f); // 速度环 0.0f 0.0f 0.0f
    chassis_controller.pid().lb_wheel.SetKi(0.0f);
    chassis_controller.pid().lb_wheel.SetKd(0.5f);
    chassis_controller.pid().lb_wheel.SetMaxOut(6000.0f);
    chassis_controller.pid().lb_wheel.SetMaxIout(0.0f);
    chassis_controller.pid().rb_wheel.SetKp(3.0f); // 速度环 0.0f 0.0f 0.0f
    chassis_controller.pid().rb_wheel.SetKi(0.0f);
    chassis_controller.pid().rb_wheel.SetKd(0.5f);
    chassis_controller.pid().rb_wheel.SetMaxOut(6000.0f);
    chassis_controller.pid().rb_wheel.SetMaxIout(0.0f);
}

void GlobalWarehouse::RCStateUpdate() {
    // if (globals->referee_data_buffer.data().robot_status.power_management_gimbal_output == 0) {
    //     globals->StateMachine_ = NO_FORCE;
    // } else {
    switch (globals->rc->switch_r()) {
        case rm::device::DR16::SwitchPosition::kUp:
            // 右拨杆打到最上侧挡位
            switch (globals->rc->switch_l()) {
                case rm::device::DR16::SwitchPosition::kDown:
                    globals->StateMachine_ = MATCH; // 左拨杆拨到下侧，进入比赛模式，此时全部系统都上电工作
                    break;
                case rm::device::DR16::SwitchPosition::kMid:
                case rm::device::DR16::SwitchPosition::kUp:
                default:
                    globals->StateMachine_ = NO_FORCE; // 左拨杆拨到下侧，进入比赛模式，此时全部系统都上电工作
                    break;
            }
            break;

        case rm::device::DR16::SwitchPosition::kMid:
            // 右拨杆打到中间挡位
            switch (globals->rc->switch_l()) {
                case rm::device::DR16::SwitchPosition::kDown:
                    globals->StateMachine_ = TEST; // 左拨杆拨到下侧，进入测试模式
                    gimbal->GimbalMove_ = GB_REMOTE;
                    chassis->ChassisMove_ = CS_REMOTE;
                    break;
                case rm::device::DR16::SwitchPosition::kMid:
                    globals->StateMachine_ = TEST;
                    gimbal->GimbalMove_ = GB_SCAN;
                    chassis->ChassisMove_ = CS_NAVIGATE;
                    break;
                case rm::device::DR16::SwitchPosition::kUp:
                    globals->StateMachine_ = TEST;
                    gimbal->GimbalMove_ = GB_AIMBOT;
                    chassis->ChassisMove_ = CS_REMOTE;
                    break;
                default:
                    globals->StateMachine_ = NO_FORCE; // 左拨杆拨到下侧，进入比赛模式，此时全部系统都上电工作
                    break;
            }
            break;

        case rm::device::DR16::SwitchPosition::kDown:
        default:
            globals->StateMachine_ = NO_FORCE; // 如果遥控器离线，进入无力模式
            break;
    }
    // }
}
