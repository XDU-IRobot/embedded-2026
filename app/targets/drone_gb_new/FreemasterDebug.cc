#include "FreemasterDbug.hpp"

double Apitch_ = 0;  // 实际位置
double Ayaw_ = 0;

double Arc_pitch = 0;  // 目标位置
double Arc_yaw = 0;

int16_t Arc_dirl = 0;  // 波轮数据

int16_t Arpm_right = 0;  // 电机转速
int16_t Arpm_left = 0;

int16_t Arc_leftx = 0;  // 开关状态
int16_t Arc_lefty = 0;

float Apitch_torque_ = 0.0f;  // 重力补偿力矩

float Aw_z = 0.0f;

float Aoutput_yaw = 0.0f;
float Aoutput_pitch = 0.0f;
float Apitch_cmd = 0.0f;

float Apitch_speed_tf = 0.0f;
float Arobot_id = 0.0f;

// 调试接口函数
void FreemasterDebug() {
  Ayaw_ = gimbal->yaw;  // 实际
  Apitch_ = gimbal->pitch;

  Arc_yaw = gimbal->rc_yaw_data;      // 遥控
  Arc_pitch = gimbal->rc_pitch_data;  //

  Arc_leftx = gimbal->rc->left_x();  // 开关
  Arc_lefty = gimbal->rc->left_y();

  Arc_dirl = gimbal->rc->dial();
  ;

  Arpm_left = gimbal->friction_left->rpm();
  Arpm_right = gimbal->friction_right->rpm();

  Apitch_torque_ = gimbal->pitch_torque;  // 重力补偿

  Aw_z = gimbal->imu->gyro_z();

  Aoutput_yaw = gimbal->gimbal_controller.output().yaw;
  Aoutput_pitch = gimbal->gimbal_controller.output().pitch;
  Apitch_cmd = gimbal->pitch_cmd;

  Apitch_speed_tf = gimbal->pitch_speed_tf;  // 摩擦阻力补偿

  Arobot_id = gimbal->robot_id;  // 裁判系统测试
}