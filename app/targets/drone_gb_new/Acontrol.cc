#include "gimbal.hpp"
// 控制逻辑
void Gimbal::GimbalControl() {
  if (GimbalState_ == kManual) {
    if (DM_is_enable == false) {
      pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
      DM_is_enable = true;
      gimbal_controller.Enable(true);
#if CONTROLLER_CHOICE==0
      rc_yaw_data = yaw;      // 第一次进入更新当前位置
      rc_pitch_data = pitch;  // 使用 IMU pitch 作为初始姿态
#elif CONTROLLER_CHOICE==1
      rc_yaw_data = yaw_;      // 第一次进入更新当前位置
      rc_pitch_data = pitch_;  // 使用 IMU pitch 作为初始姿态
#endif

      rc_pitch_data = rm::modules::Clamp(rc_pitch_data, pitch_min_pos, pitch_max_pos);  // 对rc数据进行限位
    }
    yaw_relative = rm::modules::Wrap(GetYawMotorAngleRad() - yaw_center_encoder, -M_PI, M_PI);  // 相对机械中点误差
    yaw_delta = 0.0f;

    if (vt03->data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kCtrl)) {
      // CTRL held: 键盘控制(W/S/A/D), 遥控器和鼠标输入失效
      if (vt03->data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kW)) rc_pitch_data -= 0.0001f;
      if (vt03->data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kS)) rc_pitch_data += 0.0001f;
      if (vt03->data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kA)) yaw_delta += 0.0001f;
      if (vt03->data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kD)) yaw_delta -= 0.0001f;
    } else {
      if (Rcchoose()) {
        yaw_delta -= rm::modules::Map(vt03->data().left_y, -1, 1, -0.005f, 0.005f);         // vt03手控备份
        yaw_delta -= rm::modules::Map(vt03->data().mouse_x, -660, 660, -0.03f, 0.03f);      // vt03鼠标控制
        rc_pitch_data -= rm::modules::Map(vt03->data().left_x, -1, 1, -0.005f, 0.005f);     // vt03手控备份
        rc_pitch_data -= rm::modules::Map(vt03->data().mouse_y, -660, 660, -0.03f, 0.03f);  // vt03鼠标控制
      } else {
        yaw_delta -= rm::modules::Map(rc->left_x(), -660, 660, -0.005f, 0.005f);      // dt7手控
        yaw_delta -= rm::modules::Map(rc->mouse_x(), -660, 660, -0.03f, 0.03f);       // dt7备份控制
        rc_pitch_data -= rm::modules::Map(rc->left_y(), -660, 660, -0.005f, 0.005f);  // dt7手控
        rc_pitch_data -= rm::modules::Map(rc->mouse_y(), -660, 660, -0.03f, 0.03f);   // dt7备份控制
      }
    }

    if (yaw_relative >= yaw_max_limit && yaw_delta < 0.0f) {  // 机械限位返回逻辑
      yaw_delta = 0.0f;
    }
    if (yaw_relative <= yaw_min_limit && yaw_delta > 0.0f) {
      yaw_delta = 0.0f;
    }

    rc_yaw_data = rm::modules::Wrap(rc_yaw_data + yaw_delta, 0, 2 * M_PI);
    rc_pitch_data = rm::modules::Clamp(rc_pitch_data, pitch_min_pos, pitch_max_pos);
    // 滚转补偿
    auto roll_comp = ApplyRollComp(rc_yaw_data, rc_pitch_data);
#if CONTROLLER_CHOICE==0
    // 设定目标，并计算
    gimbal_controller.SetTarget(roll_comp.first, roll_comp.second, 0, 0);
    gimbal_controller.Update(yaw, -yaw_motor->rpm(), pitch, pitch_motor->vel(), 1.f);
    yaw_motor->SetCurrent(rm::modules::Clamp(-gimbal_controller.output().yaw, -25000, 25000));  // 设置输出电流并输出
    // 重力补偿
    pitch_torque = pitch_torque_kp * cos(pitch - 3.14);  // 这里输出的力矩是反向
    pitch_torque = rm::modules::Clamp(pitch_torque, -3, 3);
#elif CONTROLLER_CHOICE==1
    // 设定目标，并计算
    gimbal_controller.SetTarget(roll_comp.first, roll_comp.second, 0, 0);
    gimbal_controller.Update(yaw_, -yaw_motor->rpm(), pitch_, pitch_motor->vel(), 1.f);
    yaw_motor->SetCurrent(rm::modules::Clamp(-gimbal_controller.output().yaw, -25000, 25000));  // 设置输出电流并输出
    // 重力补偿
    pitch_torque = pitch_torque_kp * cos(pitch_ - 3.14);  // 这里输出的力矩是反向
    pitch_torque = rm::modules::Clamp(pitch_torque, -3, 3);
#endif
  } else if (GimbalState_ == kAuto) {  // 自瞄模式控制
    if (DM_is_enable == false) {       // 使达妙电机使能
      pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
      DM_is_enable = true;
      gimbal_controller.Enable(true);
#if CONTROLLER_CHOICE==0
      rc_yaw_data = yaw;
      rc_pitch_data = pitch;  // 使用 IMU pitch 作为初始姿态
#elif CONTROLLER_CHOICE==1
      rc_yaw_data = yaw_;
      rc_pitch_data = pitch_;  // 使用 IMU pitch 作为初始姿态
#endif
      rc_pitch_data = rm::modules::Clamp(rc_pitch_data, pitch_min_pos, pitch_max_pos);
    }
    if (Aimbot.AimbotState == 2 || Aimbot.AimbotState == 4) {
      rc_yaw_data = Aimbot.TargetYawAngle + M_PI;
      rc_yaw_data = rm::modules::Wrap(rc_yaw_data, 0, 2 * M_PI);

      rc_pitch_data = Aimbot.TargetPitchAngle + M_PI;
      rc_pitch_data = rm::modules::Clamp(rc_pitch_data, pitch_min_pos, pitch_max_pos);
    } else {  // 非自瞄状态自动切入手控
      yaw_relative = rm::modules::Wrap(GetYawMotorAngleRad() - yaw_center_encoder, -M_PI, M_PI);  // 相对机械中点误差
      yaw_delta = 0.0f;
      if (Rcchoose()) {
        yaw_delta -= rm::modules::Map(vt03->data().left_y, -1, 1, -0.005f, 0.005f);         // vt03手控备份
        yaw_delta -= rm::modules::Map(vt03->data().mouse_x, -660, 660, -0.03f, 0.03f);      // vt03鼠标控制
        rc_pitch_data -= rm::modules::Map(vt03->data().left_x, -1, 1, -0.005f, 0.005f);     // vt03手控备份
        rc_pitch_data -= rm::modules::Map(vt03->data().mouse_y, -660, 660, -0.03f, 0.03f);  // vt03鼠标控制
      } else {
        yaw_delta -= rm::modules::Map(rc->left_x(), -660, 660, -0.005f, 0.005f);      // dt7手控
        yaw_delta -= rm::modules::Map(rc->mouse_x(), -660, 660, -0.03f, 0.03f);       // dt7备份控制
        rc_pitch_data -= rm::modules::Map(rc->left_y(), -660, 660, -0.005f, 0.005f);  // dt7手控
        rc_pitch_data -= rm::modules::Map(rc->mouse_y(), -660, 660, -0.03f, 0.03f);   // dt7备份控制
      }

      if (yaw_relative >= yaw_max_limit && yaw_delta < 0.0f) {  // 机械限位返回逻辑
        yaw_delta = 0.0f;
      }
      if (yaw_relative <= yaw_min_limit && yaw_delta > 0.0f) {
        yaw_delta = 0.0f;
      }

      rc_yaw_data = rm::modules::Wrap(rc_yaw_data + yaw_delta, 0, 2 * M_PI);
      rc_pitch_data = rm::modules::Clamp(rc_pitch_data, pitch_min_pos, pitch_max_pos);
    }
    // 滚转补偿
    auto roll_comp = ApplyRollComp(rc_yaw_data, rc_pitch_data);
#if CONTROLLER_CHOICE==0
    // 设定目标，并计算
    gimbal_controller.SetTarget(roll_comp.first, roll_comp.second, 0, 0);
    gimbal_controller.Update(yaw, -yaw_motor->rpm(), pitch, pitch_motor->vel(), 1.f);
    yaw_motor->SetCurrent(rm::modules::Clamp(-gimbal_controller.output().yaw, -25000, 25000));  // 设置输出电流并输出
    // 重力补偿
    pitch_torque = pitch_torque_kp * cos(pitch - 3.14);  // 这里输出的力矩是反向
    pitch_torque = rm::modules::Clamp(pitch_torque, -3, 3);
#elif CONTROLLER_CHOICE==1
    // 设定目标，并计算
    gimbal_controller.SetTarget(roll_comp.first, roll_comp.second, 0, 0);
    gimbal_controller.Update(yaw_, -yaw_motor->rpm(), pitch_, pitch_motor->vel(), 1.f);
    yaw_motor->SetCurrent(rm::modules::Clamp(-gimbal_controller.output().yaw, -25000, 25000));  // 设置输出电流并输出
    // 重力补偿
    pitch_torque = pitch_torque_kp * cos(pitch_ - 3.14);  // 这里输出的力矩是反向
    pitch_torque = rm::modules::Clamp(pitch_torque, -3, 3);
#endif
  } else {  // 失能
    if (DM_is_enable == true) {
      pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
      DM_is_enable = false;
      gimbal_controller.Enable(false);
      yaw_motor->SetCurrent(0);
    }
  }
}
void Gimbal::AmmoControl() {
  // 发射状态
  if (AmmoState_ == kFire) {
    shoot_controller.Enable(true);
    shoot_controller.Arm(true);
    shoot_controller.SetMode(Shoot2Fric::kFullAuto);

    if (rc->dial() >= 550 || rc->mouse_button_left() || vt03->data().mouse_button_left || vt03->data().trigger) {
      if (auto_reverse_flag) {
        shoot_controller.SetLoaderSpeed(-redirl_speed);
        auto_reverse_time--;
        auto_reverse_time < 1 ? auto_reverse_flag = false : auto_reverse_flag = true;
      } else {                        // 不反转
        if (GimbalState_ == kAuto) {  // 是自瞄下的状态
          if (Aimbot.AimbotState == 4) {
            shoot_controller.SetLoaderSpeed(dirl_speed);
          } else if (Aimbot.AimbotState == 2) {
            shoot_controller.SetLoaderSpeed(0.0f);
          } else {
            shoot_controller.SetLoaderSpeed(dirl_speed);
          }
        } else {  // 手动状态
          shoot_controller.SetLoaderSpeed(dirl_speed);
        }
      }
    } else if (rc->dial() <= -600) {
      shoot_controller.SetLoaderSpeed(-redirl_speed);
    } else {
      shoot_controller.SetLoaderSpeed(0.0f);
    }

    // 自动反转逻辑
    if (shoot_controller.GetLoaderSpeed() == dirl_speed) {
      auto_reverse_buffer[4] = auto_reverse_buffer[3];
      auto_reverse_buffer[3] = auto_reverse_buffer[2];
      auto_reverse_buffer[2] = auto_reverse_buffer[1];
      auto_reverse_buffer[1] = auto_reverse_buffer[0];
      auto_reverse_buffer[0] = dial_motor->encoder();
      if (auto_reverse_buffer[0] == auto_reverse_buffer[4]) {
        auto_reverse_flag = true;
        auto_reverse_time = auto_reverse_time_max;
      }
    }
    shoot_controller.SetArmSpeed(friction_speed);  // 摩擦轮目标线速度（rad/s 或你的系统单位）
    shoot_controller.Update(friction_left->rpm(), friction_right->rpm(), dial_motor->rpm());

    friction_left->SetCurrent((int16_t)rm::modules::Clamp(shoot_controller.output().fric_1, -10000, 10000));
    friction_right->SetCurrent((int16_t)rm::modules::Clamp(shoot_controller.output().fric_2, -10000, 10000));
    dial_motor->SetCurrent((int16_t)rm::modules::Clamp(shoot_controller.output().loader, -10000, 10000));

  }

  // 准备状态
  else if (AmmoState_ == kReady) {
    shoot_controller.Enable(true);
    shoot_controller.Arm(true);

    shoot_controller.SetMode(Shoot2Fric::kStop);
    shoot_controller.SetArmSpeed(0.0f);

    shoot_controller.Update(friction_left->rpm(), friction_right->rpm(), dial_motor->rpm());

    friction_left->SetCurrent((int16_t)rm::modules::Clamp(shoot_controller.output().fric_1, -10000, 10000));
    friction_right->SetCurrent((int16_t)rm::modules::Clamp(shoot_controller.output().fric_2, -10000, 10000));
    dial_motor->SetCurrent(0);
  }

  // 停止状态
  else {
    shoot_controller.Enable(false);
    shoot_controller.Arm(false);
    friction_left->SetCurrent(0);
    friction_right->SetCurrent(0);
    dial_motor->SetCurrent(0);
  }
}
void Gimbal::ShootSpeedControl() {  // 弹速控制
  shoottime_--;
  if (shoottime_ < 0) {
    if ((vt03->data().keyboard_key & (1u << 11)) && (vt03->data().keyboard_key & (1u << 5))) {
      friction_speed -= shootstep;
      shootcnt += 1;
    } else if ((vt03->data().keyboard_key & (1u << 12)) && (vt03->data().keyboard_key & (1u << 5))) {
      friction_speed += shootstep;
      shootcnt -= 1;
    } else if (vt03->data().keyboard_key & (1u << 13) && (vt03->data().keyboard_key & (1u << 5))) {
      friction_speed = 6500;
      shootcnt = 0;
    }
    shoottime_ = shoottime;
  }
}

float Gimbal::SpeedAver() {
  float new_speed = referee_data_buffer.data().shoot_data.initial_speed;

  // 如果数据有效且与上次记录不同，则更新滑动窗口
  if (new_speed > 0 && new_speed != spaver[9]) {
    for (int i = 0; i < 9; i++) {
      spaver[i] = spaver[i + 1];
    }
    spaver[9] = new_speed;
  }

  // 计算平均值
  float sum = 0;
  int count = 0;
  for (int i = 0; i < 10; i++) {
    if (spaver[i] != 0) {
      sum += spaver[i];
      count++;
    }
  }
  return (count > 0) ? (sum / count) : 0.0f;
}

void Gimbal::WS2812Control() {
  auto key = vt03->data().keyboard_key;
  bool w_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kW);
  bool a_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kA);
  bool s_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kS);
  bool d_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kD);
  bool q_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kQ);
  bool e_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kE);
  bool shift_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kShift);
  bool ctrl_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kCtrl);
  bool z_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kZ);
  bool x_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kX);
  bool c_pressed = key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kC);

  if (z_pressed && x_pressed && c_pressed) {
    if (led_blink_time < 5) {
      Set_LED(0, 255, 0, 0);
      Set_LED(1, 255, 0, 0);
      Set_LED(2, 255, 0, 0);
      Set_LED(3, 255, 0, 0);
    } else if (led_blink_time < 10) {
      Set_LED(0, 0, 0, 0);
      Set_LED(1, 0, 0, 0);
      Set_LED(2, 0, 0, 0);
      Set_LED(3, 0, 0, 0);
    } else
      led_blink_time = 0;
    led_blink_time++;
  }
  // 前进后退
  if (!ctrl_pressed && w_pressed && !s_pressed)
    Set_LED(1, 0, 255, 0);
  else if (!ctrl_pressed && !w_pressed && s_pressed)
    Set_LED(1, 255, 0, 0);
  else
    Set_LED(1, 255, 255, 0);

  // 左右or偏航
  if (!ctrl_pressed && a_pressed ^ d_pressed) {
    if (a_pressed) {
      Set_LED(0, 0, 0, 0);
      Set_LED(3, 255, 255, 255);
    } else if (d_pressed) {
      Set_LED(0, 255, 255, 255);
      Set_LED(3, 0, 0, 0);
    } else {
      Set_LED(0, 0, 0, 0);
      Set_LED(3, 0, 0, 0);
    }
  } else if (q_pressed ^ e_pressed) {
    if (q_pressed) {
      if (led_blink_time < 5)
        Set_LED(3, 255, 255, 255);
      else if (led_blink_time < 10)
        Set_LED(3, 0, 0, 0);
      else
        led_blink_time = 0;
      led_blink_time++;
      Set_LED(0, 0, 0, 0);
    } else if (e_pressed) {
      if (led_blink_time < 5)
        Set_LED(0, 255, 255, 255);
      else if (led_blink_time < 10)
        Set_LED(0, 0, 0, 0);
      else
        led_blink_time = 0;
      led_blink_time++;
      Set_LED(3, 0, 0, 0);
    }
  } else {
    Set_LED(0, 0, 0, 0);
    Set_LED(3, 0, 0, 0);
  }

  // 上升
  if (shift_pressed)
    Set_LED(2, 255, 255, 255);
  else
    Set_LED(2, 0, 0, 0);

  Set_Brightness(10);
  WS2812_Send();
}

bool Gimbal::ID() {
  if (referee_data_buffer.data().robot_status.robot_id != 0) {
    if (referee_data_buffer.data().robot_status.robot_id == 106) {
      ID_last = 1;
      return 1;  // 蓝方
    }
    ID_last = 0;
    return 0;  // 红方
  }
  return ID_last;
}