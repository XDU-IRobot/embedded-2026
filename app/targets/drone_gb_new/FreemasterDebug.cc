#include "FreemasterDbug.hpp"
  float Apitch_ = 0;  // 实际位置(-pi到pi)
  float Ayaw_ = 0;
  float Aroll_ = 0;

double Arc_pitch = 0;  // 目标位置
double Arc_yaw = 0;

float Apitch_torque_ = 0.0f;  // 重力补偿力矩
float Ayaw_torque_ = 0.0f;
// pid输出
float Aoutput_yaw = 0.0f;
float Aoutput_pitch = 0.0f;
float Apitch_cmd = 0.0f;
float Apid_yaw_position = 0.0f;//yaw位置环
// 裁判系统测试
float Arobot_id = 0.0f;
float Ashootspeed = 0.0f;
// 自瞄数据输出
float Atargetpitch = 0.0f;
float Atagetyaw = 0.0f;
uint8_t Aaimbotflag = 0;
uint8_t Aaimfireflag = 0;
// yaw编码器rad
float Ayaw_position = 0;
float Ayaw_relative = 0.0f;
// rc是否在线
bool Arc_online = 0;
bool Avt03_online = 0;
// 手调弹速计数
int Aspeedcnt = 0;
// 速度和位置返回值
float Apitchspeed = 0.0f;
float Apitchposition = 0.0f;
// pitch输出
float Apitchposout = 0.0f;
float Apitchoutp = 0.0f;
float Apitchouti = 0.0f;
float Apitchoutd = 0.0f;
// 检测can总线发送数据
float Acan1tx = 0.0f;
float Acan2tx = 0.0f;
float Acan1drop = 0.0f;
float Acan2drop = 0.0f;
float Acan2drop1 = 0.0f;
float Acan2drop2 = 0.0f;
float Acan1queue = 0.0f;
float Acan2queue = 0.0f;
uint16_t Aui_game_time = 0;
uint8_t Aui_game_progress = 0;
// 红蓝方
uint8_t Aid = 0;
// 调试接口函数
void FreemasterDebug() {
  Ayaw_ = gimbal->yaw_;  // 实际
  Apitch_ = gimbal->pitch_;
  Aroll_ = gimbal->roll_;

  Arc_yaw = gimbal->rc_yaw_data;      // 遥控
  Arc_pitch = gimbal->rc_pitch_data;  //

  Apitch_torque_ = gimbal->pitch_torque;  // 前馈补偿
  Ayaw_torque_ = gimbal->yaw_torque;      // 前馈补偿

  Aoutput_yaw = gimbal->gimbal_controller.output().yaw;
  Aoutput_pitch = gimbal->gimbal_controller.output().pitch;
  Apitch_cmd = gimbal->pitch_cmd;

  Arobot_id = gimbal->referee_data_buffer.data().robot_status.robot_id;  // 裁判系统测试
  Ashootspeed = gimbal->referee_data_buffer.data().shoot_data.initial_speed;//裁判系统弹速

  Aaimbotflag = Aimbot.AimbotState;  // 自瞄回传数据测试
  Aaimfireflag = Aimbot.AutoFire;
  Atargetpitch = Aimbot.TargetPitchAngle;
  Atagetyaw = Aimbot.TargetYawAngle;

  Ayaw_position = gimbal->GetYawMotorAngleRad();
  Ayaw_relative = gimbal->yaw_relative;

  Apid_yaw_position = gimbal->gimbal_controller.pid().yaw_position.out();

  Arc_online = gimbal->RcIsOnline();
  Avt03_online = gimbal->Vt03IsOnline();

  Aspeedcnt = gimbal->shootcnt;

  Apitchspeed = gimbal->pitch_motor->vel();
  Apitchposition = gimbal->pitch_motor->pos();

  Apitchoutp = gimbal->gimbal_controller.pid().pitch_position.p_out();
  Apitchouti = gimbal->gimbal_controller.pid().pitch_position.i_out();
  Apitchoutd = *(gimbal->gimbal_controller.pid().pitch_position.d_out());

  Acan1tx = gimbal->can1->stats().tx_fps;
  Acan1drop = gimbal->can1->stats().drop_total_fps;
  Acan1queue = gimbal->can1->stats().enqueue_fps;

  Acan2tx = gimbal->can2->stats().tx_fps;
  Acan2drop = gimbal->can2->stats().drop_total_fps;
  Acan2drop1 = gimbal->can2->stats().drop_expired_fps;
  Acan2drop2 = gimbal->can2->stats().drop_full_fps;
  Acan2queue = gimbal->can2->stats().enqueue_fps;

  Aid = gimbal->ID();
  Ayaw_torque_ = gimbal->yaw_torque;
}