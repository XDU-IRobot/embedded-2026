#ifndef CHASSIS_HPP
#define CHASSIS_HPP

#include <librm.hpp>

#include "main.hpp"

using namespace rm;

inline class Chassis {
public:
    StateMachineType ChassisMove_ = {NO_FORCE}; // 底盘运动状态
private:
    f32 chassis_target_x_ = 0.0f; // 底盘x轴目标值
    f32 chassis_target_y_ = 0.0f; // 底盘y轴目标值
    f32 chassis_target_w_ = 0.0f; // 底盘转动目标值

    f32 down_yaw_delta_ = 0.0f; // 下部yaw轴随动差值

    f32 steer_init_angle_lf_ = 3720.0f; // 轮子电机初始角度
    f32 steer_init_angle_rf_ = 490.0f;
    f32 steer_init_angle_lb_ = 3810.0f;
    f32 steer_init_angle_rb_ = 1180.0f;

    bool rotate_flag_ = false; // 小陀螺模式标识位

    const f32 front_down_yaw_angle_ = 1.57f; // 前方下yaw轴角度


public:
    void ChassisInit();

    void ChassisTask();

private:
    void ChassisStateUpdate();

    void ChassisRCDataUpdate();

    void ChassisNavigateDataUpdate();

    void ChassisMovePIDUpdate();

    void ChassisMatchUpdate();

    void ChassisEnableUpdate();

    void ChassisDisableUpdate();
} *chassis;

#endif  // CHASSIS_HPP
