#include "gimbal.hpp"
//遥控器数据更新以及编码器解算
void Gimbal::RCStateUpdate() {
  switch (rc->switch_r()) {
    case rm::device::DR16::SwitchPosition::kUp:  // 发射控制逻辑
      AmmoState_ = kFire;
      break;
    case rm::device::DR16::SwitchPosition::kMid:
      AmmoState_ = kReady;
      break;
    default:
      AmmoState_ = kStop;
      break;
  }
  switch (rc->switch_l()) {
    case rm::device::DR16::SwitchPosition::kUp:  // 上打完全自瞄
      GimbalState_ = kAuto;
      break;
    case rm::device::DR16::SwitchPosition::kMid:  // 中位按下鼠标右键跟随
      if (rc->mouse_button_right() || vt03_date_.mouse_button_right)
        GimbalState_ = kAuto;
      else
        GimbalState_ = kManual;
      break;
    default:
      GimbalState_ = kNoForce;
      break;
  }
}

bool Gimbal::RcIsOnline() {  // 判断遥控器是否在线
  device_rc.Update();
  return rc->online_status() == rm::device::Device::kOk;
}

void Gimbal::VT03DateUpdate() {  // vt03数据获取
  vt03_date_.mouse_x = vt03->data().mouse_x;
  vt03_date_.mouse_y = vt03->data().mouse_y;
  vt03_date_.mouse_button_left = vt03->data().mouse_button_left;
  vt03_date_.mouse_button_right = vt03->data().mouse_button_right;
  vt03_date_.rc_left_x = vt03->data().left_x;
  vt03_date_.rc_left_y = vt03->data().left_y;
  vt03_date_.Fn_left = vt03->data().left_button;
  vt03_date_.Fn_right = vt03->data().right_button;
  vt03_date_.fric_fire = vt03->data().trigger;
}

float Gimbal::GetYawMotorAngleRad() {  // 编码器返回角度
  return yaw_motor->encoder() * 2.0f * M_PI / 8192.0f;
}
