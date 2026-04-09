#include "FreemasterDbug.hpp"

extern AimbotFrame_SCM_t Aimbot;
// freeMaster调试变量
double Ayaw;
double Apitch;
double Aroll;
double Aoutputyaw;
double Aoutputpitch;
double Arcyawdata;
float Arcpitchdata;
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
float Apidoutput;
float Asmcoutput;
float Aselfyawtarget;
float Aautoyawtarget;
float Aselfpitchtarget;
float Aautopitchtarget;
float Astasmcoutput;
float Ainit_speed;
float Aspeed_average;
float Apitch_err_average;
int Atrace = 0;
float Asmckp;
float Asmcsat;
float Asmci;
float Asmcff;
float Asendpitch;
float Asendyaw;
float Apitchff;

// 调试接口函数
void FreemasterDebug() {
  // Arcpitchdata =
  //     rm::modules::Wrap(gimbal->rc_pitch_data - gimbal->err_average, 0, 2 * M_PI);  // 使用 IMU pitch 作为初始姿态
  // Ayaw = gimbal->yaw;
  Arcyawdata = gimbal->rc_yaw_data;
  Apitch = modules::Wrap(-gimbal->pitch - 1.9101981, -M_PI, M_PI);
  Apitch_err_average = gimbal->err_average;
  Aroll = gimbal->roll;
  Aoutputyaw = gimbal->gimbal_controller.output().yaw;
  Aoutputpitch = gimbal->gimbal_controller.output().pitch;
  Atorque = gimbal->pitch_torque + gimbal->gimbal_controller.output().pitch;
  Adirlrmp = gimbal->dial_motor->encoder();
  Adirout = gimbal->shoot_controller.output().loader;
  Aerr = gimbal->err_average;
  Ashooter_17mm_1_barrel_heat = gimbal->referee_data_buffer.data().power_heat_data.shooter_17mm_1_barrel_heat;
  Aid = gimbal->referee_data_buffer.data().robot_status.robot_id;
  Ashoot_speed = gimbal->referee_data_buffer.data().shoot_data.initial_speed;
  Ashoot_hz = gimbal->referee_data_buffer.data().shoot_data.launching_frequency;
  Adrmp = gimbal->friction_left->rpm() + gimbal->friction_right->rpm();
  Armp = gimbal->friction_left->rpm();
  Apitchpose = gimbal->pitch_motor->pos();
  Ayaw = gimbal->yaw;
  Aautoyawtarget = rm::modules::Wrap(Aimbot.TargetYawAngle + M_PI, 0, 2 * M_PI);
  Aselfyawtarget = gimbal->yaw;
  Aautopitchtarget = rm::modules::Wrap(Aimbot.TargetPitchAngle + gimbal->err_average + M_PI, 0, 2 * M_PI);
  Aselfpitchtarget = rm::modules::Wrap(gimbal->pitch + gimbal->err_average, 0, 2 * M_PI);

  Apidoutput = gimbal->gimbal_controller.output().yaw;
  Asmcoutput = gimbal->gimbal_controller_SMC.output().yaw;
  Astasmcoutput = gimbal->gimbal_controller_STASMC.output().yaw;
  Ainit_speed = gimbal->referee_data_buffer.data().shoot_data.initial_speed;
  Aspeed_average = gimbal->fire_speed_average;
}