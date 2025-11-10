#include <librm.hpp>

#include "can.h"
#include "usart.h"
#include "spi.h"

#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "timer_task.hpp"
#include "sparse_value_watcher.hpp"
#include "device_manager.hpp"
#include "controllers/gimbal_2dof.hpp"

float Map(float value, float from_min, float from_max, float to_min, float to_max) {
    return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min;
}
float LoopConstrain(float input, float min_value, float max_value) {
    float cycle = max_value - min_value;
    if (cycle < 0) {
        return input;
    }

    if (input > max_value) {
        while (input > max_value) {
            input -= cycle;
        }
    } else if (input < min_value) {
        while (input < min_value) {
            input += cycle;
        }
    }
    return input;
}
float Constrain(float input, float min_value, float max_value) {
    if (input < min_value) {
        return min_value;
    } else if (input > max_value) {
        return max_value;
    } else {
        return input;
    }
}


struct GlobalWarehouse
{
    AsyncBuzzer *buzzer{nullptr}; ///< 蜂鸣器
    LED *led{nullptr}; ///< RGB LED灯

    // 硬件接口 //
    rm::hal::Can *can1{nullptr}; ///< CAN 总线接口
    rm::hal::Serial *dbus{nullptr}; ///< 遥控器串口接口

    // 设备 //
    DeviceManager<10> device_manager; ///< 设备管理器，维护所有设备在线状态
    rm::device::DR16 *rc{nullptr}; ///< 遥控器

    rm::device::GM6020 *yaw_motor{nullptr};
    // rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr}; ///< 云台 Pitch 电机
    // rm::device::M3508 *left_fric_motor{nullptr}, *right_fric_motor{nullptr}; ///< 摩擦轮电机
    // rm::device::M2006 *driver_motor{nullptr};
    rm::device::BMI088 *imu{nullptr}; ///< BMI088 IMU

    // 控制器 //
    Gimbal2Dof gimbal_controller;
    SparseValueWatcher<rm::device::DR16::SwitchPosition> rc_l_switch_watcher, rc_r_switch_watcher;
    rm::modules::MahonyAhrs ahrs{1000.f}; ///< mahony 姿态解算器，频率 1000Hz

    float rm_yaw=0.0;
    float rm_pitch=0.0;

    void Init()
    {
        buzzer = new AsyncBuzzer;
        led = new LED;

        can1 = new rm::hal::Can{hcan1};
        dbus = new rm::hal::Serial{huart3, 36, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};

        rc = new rm::device::DR16{*dbus};
        yaw_motor = new rm::device::GM6020{*can1, 2};
        // pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{*can1, {}};
        // left_fric_motor = new rm::device::M3508{*can1, 4};
        // right_fric_motor = new rm::device::M3508{*can1, 3};
        // driver_motor = new rm::device::M2006{*can1, 1};
        imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};

        device_manager << rc << yaw_motor ;//<< pitch_motor << left_fric_motor << right_fric_motor; //<<driver_motor;

        can1->SetFilter(0, 0);
        can1->Begin();
        buzzer->Init();
        led->Init();
        rc->Begin(); // 启动遥控器接收，这行或许比较适合放到AppMain里面？
    }
} *globals;

void MainLoop()
{
    globals->imu->Update();
    globals->ahrs.Update(rm::modules::ImuData6Dof{
        -globals->imu->gyro_x(), //
        -globals->imu->gyro_y(), //
        globals->imu->gyro_z(), //
        -globals->imu->accel_x(), //
        -globals->imu->accel_y(), //
        globals->imu->accel_z()
    });

    if (globals->device_manager.all_device_ok())
    {
        (*globals->led)(0xff00ff00); // 绿灯代表所有设备在线
    }
    else
    {
        (*globals->led)(0xffff0000); // 红灯代表有设备离线
        globals->gimbal_controller.Enable(false);
    }

    globals->rc_l_switch_watcher.Update(globals->rc->switch_l());
    globals->rc_r_switch_watcher.Update(globals->rc->switch_r());

    globals->rm_yaw += Map(globals->rc->left_x() / 660.f * 20.f, -660, 660, -0.3f, 0.3f);  // 遥控器映射yaw轴
    globals->rm_yaw = LoopConstrain(globals->rm_yaw, 0, 360);
    globals->rm_pitch += Map(globals->rc->left_y() / 660.f * 20.f, -660, 660, -0.3f, 0.3f);  // 遥控器映射pitch轴
    globals->rm_pitch = Constrain(globals->rm_pitch, 150.0f, 190.0f);

    globals->gimbal_controller.SetTarget(globals->rm_yaw,globals->rm_pitch);
    globals->gimbal_controller.Update(-57.3 * -globals->ahrs.euler_angle().roll + 180,
                                      globals->yaw_motor->rpm() * (2.f * M_PI / 60.f),
                                      -57.3 * globals->ahrs.euler_angle().pitch + 180,
                                      0
                                      // globals->pitch_motor->vel() * (2.f * M_PI / 60.f)
    );

    globals->yaw_motor->SetCurrent(globals->gimbal_controller.output().yaw);
    //globals->pitch_motor->SetPosition(0, 0, globals->gimbal_controller.output().pitch, 0, 0);
    rm::device::DjiMotor<>::SendCommand();
}

extern "C" [[noreturn]] void AppMain(void)
{
    globals = new GlobalWarehouse;
    globals->Init();

    globals->rc_r_switch_watcher.OnValueChange(
    etl::delegate<void(const rm::device::DR16::SwitchPosition &, const rm::device::DR16::SwitchPosition &)>::create(
        [&](const rm::device::DR16::SwitchPosition &old_value, const rm::device::DR16::SwitchPosition &new_value)
        {
            globals->buzzer->Beep(1);
        }));
    globals->rc_l_switch_watcher.OnValueChange(
        etl::delegate<void(const rm::device::DR16::SwitchPosition &, const rm::device::DR16::SwitchPosition &)>::create(
            [&](const rm::device::DR16::SwitchPosition &old_value, const rm::device::DR16::SwitchPosition &new_value)
            {
                if (new_value == rm::device::DR16::SwitchPosition::kUp || new_value ==
                    rm::device::DR16::SwitchPosition::kMid)
                {
                    globals->buzzer->Beep(2, 35);
                    globals->gimbal_controller.Enable(true);
                    globals->rm_yaw=-57.3 * -globals->ahrs.euler_angle().roll + 180;
                    globals->rm_pitch=-57.3 * globals->ahrs.euler_angle().pitch + 180;
                }
                else
                {
                    globals->buzzer->Beep(1);
                    globals->gimbal_controller.Enable(false);
                }
            }));

    auto &pids = globals->gimbal_controller.pid();
    pids.yaw_position.SetKp(5.f).SetKi(0.f).SetKd(0.f).SetMaxOut(2000.f).SetMaxIout(0.f);
    pids.yaw_speed.SetKp(5.f).SetKi(0.f).SetKd(0.f).SetMaxOut(2000.f).SetMaxIout(0.f);
    pids.pitch_position.SetKp(0.05f).SetKi(0.f).SetKd(0.f).SetMaxOut(2000.f).SetMaxIout(0.f);
    pids.pitch_speed.SetKp(0.1f).SetKi(0.f).SetKd(0.f).SetMaxOut(2000.f).SetMaxIout(0.f);

    // 创建主循环定时任务，定频1khz
    TimerTask mainloop_1000hz{
        &htim13, //
        etl::delegate<void()>::create<MainLoop>() //
    };
    mainloop_1000hz.SetPrescalerAndPeriod(84 - 1, 1000 - 1); // 84MHz / 84 / 1000 = 1kHz
    mainloop_1000hz.Start();

    for (;;)
    {
        __WFI();
    }
}