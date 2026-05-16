#include "gimbal.hpp"
#include "UI.hpp"
// 子线程
//  遥控器和imu数据解算+DjiMotor发信息
void Gimbal::SubLoop500Hz() {
  // imu数据处理
  imu->Update();
  ahrs.Update(rm::modules::ImuData6Dof{imu->gyro_y(), imu->gyro_x(), -imu->gyro_z() + 0.0036f, imu->accel_y(),
                                       imu->accel_x(), -imu->accel_z()});
  pitch = ahrs.euler_angle().pitch + M_PI;
  yaw = ahrs.euler_angle().yaw + M_PI;
  roll = ahrs.euler_angle().roll + M_PI;

  GimbalImuSend(ahrs.quaternion().w, ahrs.quaternion().x, ahrs.quaternion().y, ahrs.quaternion().z, SpeedAver(),
                referee_data_buffer.data().robot_status.robot_id);  // usb传输数据
  if (!Rcchoose()) {
    RCStateUpdate();  // dt7控制更新
  } else {
    Vt03Control();  // vt03控制更新
  }
  GimbalControl();                               // 云台控制更新
  AmmoControl();                                 // 发射机构更新
  rm::device::DjiMotorBase::SendCommand(*can1);  // 向大疆所有电机发数据
  rm::device::DjiMotorBase::SendCommand(*can2);  // 向大疆所有电机发数据
}
// DmMotor电机发信息
void Gimbal::SubLoop250Hz() {
  if (time_ % 2 == 0) {
    pitch_cmd = rm::modules::Clamp(-pitch_torque + gimbal_controller.output().pitch, -10, 10);  // 发送达秒控制信息
    pitch_motor->SetMitCommand(0, 0, pitch_cmd, 0, 0);                                          // 合输出

    // pitch_speed_tf = rm::modules::Clamp(pitch_speed_kp * tanh(pitch_motor->vel()),-10,10);

    // pitch_motor->SetMitCommand(0, 0, gimbal_controller.output().pitch, 0, 0);
    // pitch_motor->SetMitCommand(0, 0,-pitch_torque, 0, 0);

    // if (pitch<=3.82&&pitch>=3.00) {//摩擦补偿测试
    //   pitch_motor->SetMitCommand(0, 0,pitch_speed_tf-pitch_torque, 0, 0);
    // }
    // else {
    //   pitch_motor->SetMitCommand(0,0,0,0,0);
    // }
  }
}
void Gimbal::SubLoop100Hz() {
  if (time_ % 5 == 0) {
    ShootSpeedControl();  // 弹速手动控制
    FreemasterDebug();    // 调试更新
  }
}
void Gimbal::SubLoop50Hz() {
  if (time_ % 10 == 0) {
    // robot_id = referee_data_buffer.data().robot_status.robot_id;  // 裁判系统测试
  }
}
void Gimbal::SubLoop10Hz() {
  if (time_ % 50 == 0) {
    WS2812Control();
    Test_Draw_String(&referee_data_buffer, friction_speed, Aimbot.AimbotState);
    time_ = 0;
  }
}
