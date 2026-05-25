#include "gimbal.hpp"
#include "UI/UI.hpp"
// 子线程
//  遥控器和imu数据解算+DjiMotor发信息
void Gimbal::SubLoop500Hz() {
  // ch040
  pitch_ = -imu_new->pitch();  // （上正下负）（+-pi）
  roll_ = -imu_new->roll();    //(左正右负)(+-pi)
  yaw_ = imu_new->yaw();       //(左正右负)（+-pi）
  GimbalImuSend(-imu_new->quat_x(), imu_new->quat_w(), imu_new->quat_z(), -imu_new->quat_y(), SpeedAver(),
                referee_data_buffer.data().robot_status.robot_id);  // usb传输数据
  GimbalControl();                                                  // 云台控制更新
  AmmoControl();                                                    // 发射机构更新
  rm::device::DjiMotorBase::SendCommand(*can1);                     // 向大疆所有电机发数据
  rm::device::DjiMotorBase::SendCommand(*can2);                     // 向大疆所有电机发数据
}
// DmMotor电机发信息
void Gimbal::SubLoop250Hz() {
  if (time_ % 2 == 0) {
    if (!Rcchoose()) {
      RCStateUpdate();  // dt7控制更新
    } else {
      Vt03Control();  // vt03控制更新
    }
    // pitch负值向上输出
    pitch_cmd = rm::modules::Clamp(-gimbal_controller.output().pitch - pitch_torque, -10, 10);  // 发送达秒控制信息
    pitch_motor->SetMitCommand(0, 0, -gimbal_controller.output().pitch, 0, 0);                  // 合输出

    // pitch_motor->SetMitCommand(0, 0,-pitch_torque, 0, 0);//单重力补偿测试
  }
}
void Gimbal::SubLoop100Hz() {
  if (time_ % 5 == 0) {
    ShootSpeedControl();  // 弹速手动控制
    FreemasterDebug();    // 调试更新
  }
}
void Gimbal::SubLoop30Hz() {
  if (time_ % 17 == 0) {
    schedule.schedule();
  }
}
void Gimbal::SubLoop50Hz() {
  if (time_ % 10 == 0) {
    // robot_id = referee_data_buffer.data().robot_status.robot_id;  // 裁判系统测试
  }
}
uint8_t test_ui_num = 0;
void Gimbal::SubLoop10Hz() {
  if (time_ % 50 == 0) {
    test_ui_num++;
    WS2812Control();
    time_ = 0;
  }
}
