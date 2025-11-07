#include "Chassis.hpp"
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
