#include "FreertosDbug.hpp"

extern AimbotFrame_SCM_t Aimbot;
// freeMaster调试变量
double Ayaw;
double Apitch;
double Aroll;
double Aoutputyaw;
double Aoutputpitch;
double Arcyawdata;
double Arcpitchdata;
double Agx;
double Agy;
double Agz;
double Apitchpose;
double Atotalpitch;
double Atorque;
double vofa_pitch;
double vofa_current;
double Adirlrmp;
double Adirout;
double Aerr;
uint16_t Ashooter_17mm_1_barrel_heat;
uint8_t Aid;
uint8_t Ashoot_hz;
float Ashoot_speed;
i16 Adrmp;
i16 Armp;

float Aautopitch;
float Aautoyaw;
f32 Ax;
f32 Ay;
f32 Az;
f32 Gx;
f32 Gy;
f32 Gz;

// 调试接口函数
void FreemasterDebug() {
  Arcyawdata = gimbal->rc_yaw_data;
  Arcpitchdata = gimbal->rc_pitch_data;
  Ayaw = gimbal->yaw;
  // Apitch = rm::modules::Wrap(gimbal->pitch + gimbal->err_average, 0, 2 * M_PI);
  Apitch = gimbal->pitch;
  Aroll = gimbal->roll;
  Aoutputyaw = gimbal->gimbal_controller.output().yaw;
  Aoutputpitch = gimbal->gimbal_controller.output().pitch;
  Apitchpose = gimbal->pitch_motor->pos();
  Atorque = gimbal->pitch_torque;
  Adirlrmp = gimbal->dial_motor->encoder();
  Adirout = gimbal->shoot_controller.output().loader;
  Aerr = gimbal->err_average;
  Ashooter_17mm_1_barrel_heat = gimbal->referee_data_buffer.data().power_heat_data.shooter_17mm_1_barrel_heat;
  Aid = gimbal->referee_data_buffer.data().robot_status.robot_id;
  Ashoot_speed = gimbal->referee_data_buffer.data().shoot_data.initial_speed;
  Ashoot_hz = gimbal->referee_data_buffer.data().shoot_data.launching_frequency;
  Adrmp = gimbal->friction_left->rpm() + gimbal->friction_right->rpm();
  Armp = gimbal->friction_left->rpm();
  Aautopitch = Aimbot.TargetPitchAngle + gimbal->err_average;
  Apitchpose = gimbal->pitch_motor->pos();

  Aautoyaw = Aimbot.TargetPitchAngle;
  Ayaw = gimbal->yaw;

  Ax = gimbal->imu->accel_x();
  Ay = gimbal->imu->accel_y();
  Az = gimbal->imu->accel_z();
  Gx = gimbal->imu->gyro_x();
  Gy = gimbal->imu->gyro_y();
  Gz = gimbal->imu->gyro_z();
}