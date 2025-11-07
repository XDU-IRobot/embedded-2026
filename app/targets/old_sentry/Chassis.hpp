#ifndef CHASSIS_HPP
#define CHASSIS_HPP

#include <librm.hpp>

#include "main.hpp"

using namespace rm;

inline class Chassis {
public:
    StateMachineType ChassisMove_ = {NO_FORCE}; // 底盘运动状态
private:
    f32 chassis_x_rc_ = 0.0f; // 底盘x轴遥控数据
    f32 chassis_y_rc_ = 0.0f; // 底盘y轴遥控数据

public:
private:
    void ChassisStateUpdate();

    void ChassisEnableUpdate();

    void ChassisMatchUpdate();

    void ChassisDisableUpdate();
} *chassis;


#endif // CHASSIS_HPP