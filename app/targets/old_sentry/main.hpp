#ifndef MAIN_HPP
#define MAIN_HPP

#include <librm.hpp>

#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "device_manager.hpp"
#include "controllers/gimbal_double_yaw.hpp"
#include "controllers/shoot_3fric.hpp"

#include "USB.hpp"

// 状态机
typedef enum {
    NO_FORCE = 0u, // 无力模式
    TEST, // 调试模式
    MATCH, // 比赛模式

    GB_REMOTE, // 云台遥控模式
    GB_SCAN, // 扫描模式
    GB_AIMBOT, // 云台自瞄模式

    CS_REMOTE, // 底盘遥控模式
    CS_NAVIGATE, // 底盘导航模式
} StateMachineType;

inline struct GlobalWarehouse {
    AsyncBuzzer *buzzer{nullptr}; ///< 蜂鸣器
    LED *led{nullptr}; ///< RGB LED灯

    // 硬件接口 //
    rm::hal::Can *can1{nullptr}, *can2{nullptr}; ///< CAN 总线接口
    rm::hal::Serial *dbus{nullptr}; ///< 遥控器串口接口

    // 设备 //
    DeviceManager<10> device_manager; ///< 设备管理器，维护所有设备在线状态
    rm::device::DR16 *rc{nullptr}; ///< 遥控器
    rm::device::Referee<rm::device::RefereeRevision::kV170> referee_data_buffer;
    rm::device::GM6020 *up_yaw_motor{nullptr}; ///< 云台 Yaw 上电机
    rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *down_yaw_motor{nullptr}; ///< 云台 Yaw 下电机
    rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr}; ///< 云台 Pitch 电机
    rm::device::BMI088 *imu{nullptr};

    // 控制器 //
    GimbalDoubleYaw gimbal_controller; ///< 二轴双 Yaw 云台控制器
    Shoot3Fric shoot_controller{9}; ///< 三摩擦轮发射机构控制器，9发拨盘
    rm::modules::MahonyAhrs ahrs{1000.0f};

    // USB //
    AimbotFrame_SCM_t Aimbot;
    GimbalImuFrame_SCM_t Imu;
    // 状态机 //
    StateMachineType StateMachine_ = {NO_FORCE}; // 当前状态

    // 函数 //
    void Init();

    void GimbalPIDInit();

    void RCStateUpdate();
} *globals;

#endif //MAIN_HPP