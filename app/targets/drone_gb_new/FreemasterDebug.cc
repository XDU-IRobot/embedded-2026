#include "FreemasterDbug.hpp"
double Apitch = 0;  // 实际位置
double Ayaw = 0;

float Apitch_ = 0;  // 实际位置(-pi到pi)
float Ayaw_ = 0;

double Arc_pitch = 0;  // 目标位置
double Arc_yaw = 0;

int16_t Arc_dirl = 0;  // 波轮数据

int16_t Arpm_right = 0;  // 电机转速
int16_t Arpm_left = 0;

int16_t Arc_leftx = 0;  // 开关状态
int16_t Arc_lefty = 0;

float Apitch_torque_ = 0.0f;  // 重力补偿力矩
// z轴角速度
float Aw_z = 0.0f;
// pid输出
float Aoutput_yaw = 0.0f;
float Aoutput_pitch = 0.0f;
float Apitch_cmd = 0.0f;
// 摩擦补偿
float Apitch_speed_tf = 0.0f;
// 裁判系统测试
float Arobot_id = 0.0f;
float Ashootspeed = 0.0f;
// vt03调试数据
int Arc_vt03_cnt = 0;
int Arc_vt03_cnt1 = 0;
float Arc_vt03_left_x = 0.0f;
float Arc_vt03_left_y = 0.0f;
int16_t Arc_vt03_mou_x = 0;
int16_t Arc_vt03_mou_y = 0;
bool Arc_vt03_left = 0;
bool Arc_vt03_right = 0;
bool AFn_left = 0;
// 自瞄数据输出
float Atargetpitch = 0.0f;
float Atagetyaw = 0.0f;
uint8_t Aaimbotflag = 0;
uint8_t Aaimfireflag = 0;
// yaw编码器rad
float Ayaw_position = 0;
float Ayaw_relative = 0.0f;
// pid输出
float Apid_yaw_position = 0.0f;
// rc是否在线
bool Arc_online = 0;
bool Avt03_online = 0;
// 手调弹速计数
int Aspeedcnt = 0;
// 键盘输出测试
bool AkQ = 0;
bool AkE = 0;
bool AkR = 0;
// 进自瞄次数
int Acnt_ = 0;
// 速度和位置返回值
float Apitchspeed = 0.0f;
float Apitchposition = 0.0f;
// pitch输出
float Apitchposout = 0.0f;
float Apitchoutp = 0.0f;
float Apitchouti = 0.0f;
float Apitchoutd = 0.0f;
//检测can总线发送数据
float Acan1tx = 0.0f;
float Acan2tx = 0.0f;
float Acan1drop = 0.0f;
float Acan2drop = 0.0f;
float Acan1queue = 0.0f;
float Acan2queue = 0.0f;
// 调试接口函数
void FreemasterDebug() {
  Ayaw_ = gimbal->yaw;  // 实际
  Apitch_ = gimbal->pitch;

  Arc_yaw = gimbal->rc_yaw_data;      // 遥控
  Arc_pitch = gimbal->rc_pitch_data;  //

  Arc_leftx = gimbal->rc->left_x();  // 开关
  Arc_lefty = gimbal->rc->left_y();

  Arc_dirl = gimbal->rc->dial();

  Arpm_left = gimbal->friction_left->rpm();
  Arpm_right = gimbal->friction_right->rpm();

  Apitch_torque_ = gimbal->pitch_torque;  // 重力补偿

  Aw_z = gimbal->imu->gyro_z();

  Aoutput_yaw = gimbal->gimbal_controller.output().yaw;
  Aoutput_pitch = gimbal->gimbal_controller.output().pitch;
  Apitch_cmd = gimbal->pitch_cmd;

  Apitch_speed_tf = gimbal->pitch_speed_tf;  // 摩擦阻力补偿

  Arobot_id = gimbal->robot_id;  // 裁判系统测试
  Ashootspeed = gimbal->referee_data_buffer.data().shoot_data.initial_speed;

  Arc_vt03_cnt = gimbal->rx_vt03->rx_callback_cnt;  // 图传系统测试
  Arc_vt03_cnt1 = gimbal->rx_vt03->rx_byte_cnt;
  Arc_vt03_left_x = gimbal->vt03->data().left_x;
  Arc_vt03_left_y = gimbal->vt03->data().left_y;

  Arc_vt03_mou_x = gimbal->vt03->data().mouse_x;
  Arc_vt03_mou_y = gimbal->vt03->data().mouse_y;
  Arc_vt03_left = gimbal->vt03->data().mouse_button_left;
  Arc_vt03_right = gimbal->vt03->data().mouse_button_right;

  Aaimbotflag = Aimbot.AimbotState;  // 自瞄回传数据测试
  Aaimfireflag = Aimbot.AutoFire;
  Atargetpitch = Aimbot.TargetPitchAngle + M_PI;
  Atagetyaw = Aimbot.TargetYawAngle + M_PI;

  Ayaw_position = gimbal->GetYawMotorAngleRad();
  Ayaw_relative = gimbal->yaw_relative;

  Apid_yaw_position = gimbal->gimbal_controller.pid().yaw_position.out();

  Arc_online = gimbal->RcIsOnline();
  Avt03_online = gimbal->Vt03IsOnline();

  Aspeedcnt = gimbal->shootcnt;
  AkR = gimbal->vt03->data().keyboard_key << 8;
  AkE = gimbal->vt03->data().keyboard_key << 7;
  AkQ = gimbal->vt03->data().keyboard_key << 6;

  Acnt_ = gimbal->cnt;
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
  Acan2queue = gimbal->can2->stats().enqueue_fps;

}