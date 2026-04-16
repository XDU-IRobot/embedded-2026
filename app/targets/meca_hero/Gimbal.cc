#include "Gimbal.hpp"
#include "main.hpp"

void GimbalHero::EnableUpdate() {
  globals->gimbal_motor_yaw->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  globals->gimbal_motor_yaw->SendInstruction(rm::device::DmMotorInstructions::kEnable);
}

void GimbalHero::UnableUpdate() {
  globals->gimbal_motor_yaw->SetMitCommand(0, 0, 0, 0, 0);
  globals->gimbal_motor_pitch->SetCurrent(0);
  globals->gimbal_motor_yaw->SendInstruction(rm::device::DmMotorInstructions::kDisable);
  target_pos_yaw = -globals->ahrs.euler_angle().yaw;
  target_pos_pitch = -globals->ahrs.euler_angle().pitch;
}

void GimbalHero::AhrsUpdate() {
  globals->imu->Update();
  globals->ahrs.Update(rm::modules::ImuData6Dof{
      globals->imu->gyro_x(), globals->imu->gyro_y(),
      gyro_z = globals->gyro_z_filter.apply(globals->imu->gyro_z()) + globals->gyro_rectification,
      globals->imu->accel_x(), globals->imu->accel_y(), globals->imu->accel_z()});
  eulerangle_yaw = -globals->ahrs.euler_angle().yaw;
  eulerangle_pitch = -globals->ahrs.euler_angle().pitch;
  eulerangle_roll = -globals->ahrs.euler_angle().roll;
}

void GimbalHero::AimbotControl() {}

bool GimbalHero::Enable() {
  // 主状态检测
  if (globals->state_machine.getMainState() == StateMachine::MainState::kOffline) {
    UnableUpdate();
    return false;
  }
  // Gimbal状态检测
  switch (globals->state_machine.getGimbalState()) {
    default:
      UnableUpdate();
      return false;
    case StateMachine::GimbalState::kOffline:
      UnableUpdate();
      return false;
    case StateMachine::GimbalState::kNoForce:
      UnableUpdate();
      return false;
    case StateMachine::GimbalState::kWaiting:
      EnableUpdate();
      return true;
    case StateMachine::GimbalState::kNormal:
      return true;
    case StateMachine::GimbalState::kAimbot:
      return true;
    case StateMachine::GimbalState::kRadarAimbot:
      return true;
  }
}
// 主循环
void GimbalHero::GimbalUpdate() {
  AhrsUpdate();
  if (!Enable()) return;
}