#include "main.hpp"
#include "Aimbot.h"
#include "usbd_cdc_if.h"
#include "VOFA.hpp"
#include "aimbot_comm_can.hpp"

void MagazineControl() {
  // 失能
  if (l_switch_position_now != rm::device::DR16::SwitchPosition::kUp ||
      r_switch_position_now == rm::device::DR16::SwitchPosition::kDown ||
      r_switch_position_now == rm::device::DR16::SwitchPosition::kUnknown) {
    static int low_f = 0;
    if (low_f > 9) {
      low_f = 0;
      globals->magazine_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    } else {
      low_f++;
    }

    // HAL_Delay(0);
    return;
  }
  // 启动
  if (l_switch_position_last != rm::device::DR16::SwitchPosition::kUp ||
      (power_management_shooter_last == 0 && globals->ref.data().robot_status.power_management_shooter_output == 1)) {
    target_magz = globals->magazine_motor->pos();
    // 检测当前角度，防止大幅转动
    while (next_target_magz < globals->magazine_motor->pos()) {
      next_target_magz += 1.0472 /*（π/3）*/;
    }
    while (next_target_magz > globals->magazine_motor->pos()) {
      next_target_magz -= 1.0472 /*（π/3）*/;
    }
    if (next_target_magz < -3.141593) {
      next_target_magz += 2 * 3.141593;
    }

    //---
    // if (globals->magazine_motor->pos()-target_magz   < -3.141593 / 6) {
    //   target_magz -= 3.141593 / 3;
    //   if (target_magz < -3.141593) {
    //     target_magz += 2 * 3.141593;
    //   }
    //   next_target_magz = target_magz - 3.141593 / 3;
    //   if (next_target_magz < -3.141593) {
    //     next_target_magz += 2 * 3.141593;
    //   }
    // } else {
    //   next_target_magz = target_magz - 3.141593 / 3;
    //   if (next_target_magz > 3.141593) {
    //     next_target_magz += 2 * 3.141593;
    //   }
    //   target_magz = globals->magazine_motor->pos();
    // }
    // 使能
    globals->magazine_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
    globals->magazine_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    // 防丝杆抖动
    target_pos_pitch = -globals->ahrs.euler_angle().pitch;
  }
  // 拨盘电机逻辑
  // 堵转检测
  // if (rm::modules::Wrap(target_magz - globals->magazine_motor->pos(), -3.141593, 3.141593) < -3.141593/18) {
  //   target_magz = globals->magazine_motor->pos() +3.141593 / 90;
  // }
  // 堵转检测
  // if (rm::modules::Wrap(target_magz - globals->magazine_motor->pos(), -3.141593, 3.141593) < -(3.141593 / 36)) {
  //   magz_compensation_count++;
  //   if (magz_compensation_count == 500) {
  //     magz_compensation += target_magz - globals->magazine_motor->pos();
  //     target_magz = globals->magazine_motor->pos();
  //   }
  // } else {
  //   magz_compensation_count = 0;
  // }
  // 按下扳机(延时1s)
  if (counter == 0) {
    if ((globals->rc->dial() >= 500 || globals->rc->dial() < -500 || globals->rc->mouse_button_left() ||
         globals->tc->data().mouse_button_left || globals->custom_client->mouse_left() ||
         (globals->aimbot_can_communicator->aimbot_state() == 0x03 &&
          (r_switch_position_now == rm::device::DR16::SwitchPosition::kUp ||
           globals->tc->data().keyboard_key & static_cast<int16_t>(VT03::KeyboardKey::kCtrl)))) &&
        (globals->ref.data().robot_status.shooter_barrel_heat_limit >=
         globals->ref.data().power_heat_data.shooter_42mm_barrel_heat + 100) &&
        (shooter_1 < -3000 && shooter_4 < -3000)) {
      target_magz = next_target_magz;
      counter = 420;
    }
  } else if (counter == 100) {
    if (rm::modules::Wrap(target_magz - globals->magazine_motor->pos(), -3.141593, 3.141593) < -3.141593 / 18) {
      target_magz = globals->magazine_motor->pos() + 3.141593 / 18;
    } else {
      next_target_magz -= 3.141593 / 3;
      if (next_target_magz < -3.141593) {
        next_target_magz += 2 * 3.141593;
      }
    }
    counter--;
  } else {
    counter--;
  }

  // 拨盘电机串级PID（开循环）
  globals->pid_magz_position->SetCircular(true).SetCircularCycle(3.141593 * 2);
  globals->pid_magz_position->Update(target_magz, globals->magazine_motor->pos(), 0.001);
  target_velocity = globals->pid_magz_position->out();
  globals->pid_magz_velocity->Update(target_velocity, globals->magazine_motor->vel(), 0.001);
  static int low_f_1;
  if (low_f_1 > 1) {
    globals->magazine_motor->SetMitCommand(0, 0, globals->pid_magz_velocity->out(), 0, 0);
  } else {
    low_f_1++;
  }
}

/*----------------------------------------------------*/
// 摩擦轮逻辑
void ShooterControl() {
  // 速度监测
  shooter_1 = globals->shooter_motor_1->rpm();
  shooter_2 = globals->shooter_motor_2->rpm();
  shooter_3 = globals->shooter_motor_3->rpm();
  shooter_4 = globals->shooter_motor_4->rpm();
  shooter_5 = globals->shooter_motor_5->rpm();
  shooter_6 = globals->shooter_motor_6->rpm();
  // 失能
  if (l_switch_position_now != rm::device::DR16::SwitchPosition::kUp ||
      r_switch_position_now == rm::device::DR16::SwitchPosition::kDown ||
      r_switch_position_now == rm::device::DR16::SwitchPosition::kUnknown) {
    // 给shooter电机发送指令
    if (shooter_6 < limit) {
      globals->pid_shooter_1->Update(limit, globals->shooter_motor_1->rpm());
      globals->pid_shooter_2->Update(limit, globals->shooter_motor_2->rpm());
      globals->pid_shooter_3->Update(limit, globals->shooter_motor_3->rpm());
      globals->pid_shooter_4->Update(limit, globals->shooter_motor_4->rpm());
      globals->pid_shooter_5->Update(limit, globals->shooter_motor_5->rpm());
      globals->pid_shooter_6->Update(limit, globals->shooter_motor_6->rpm());
      globals->shooter_motor_1->SetCurrent(globals->pid_shooter_1->out());
      globals->shooter_motor_2->SetCurrent(globals->pid_shooter_2->out());
      globals->shooter_motor_3->SetCurrent(globals->pid_shooter_3->out());
      globals->shooter_motor_4->SetCurrent(globals->pid_shooter_4->out());
      globals->shooter_motor_5->SetCurrent(globals->pid_shooter_5->out());
      globals->shooter_motor_6->SetCurrent(globals->pid_shooter_6->out());
    } else {
      // globals->pid_shooter_1->Update(0, globals->shooter_motor_1->rpm());
      // globals->pid_shooter_2->Update(0, globals->shooter_motor_2->rpm());
      // globals->pid_shooter_3->Update(0, globals->shooter_motor_3->rpm());
      // globals->pid_shooter_4->Update(0, globals->shooter_motor_4->rpm());
      // globals->pid_shooter_5->Update(0, globals->shooter_motor_5->rpm());
      // globals->pid_shooter_6->Update(0, globals->shooter_motor_6->rpm());
      // globals->shooter_motor_1->SetCurrent(globals->pid_shooter_1->out());
      // globals->shooter_motor_2->SetCurrent(globals->pid_shooter_2->out());
      // globals->shooter_motor_3->SetCurrent(globals->pid_shooter_3->out());
      // globals->shooter_motor_4->SetCurrent(globals->pid_shooter_4->out());
      // globals->shooter_motor_5->SetCurrent(globals->pid_shooter_5->out());
      // globals->shooter_motor_6->SetCurrent(globals->pid_shooter_6->out());

      globals->shooter_motor_1->SetCurrent(0);
      globals->shooter_motor_2->SetCurrent(0);
      globals->shooter_motor_3->SetCurrent(0);
      globals->shooter_motor_4->SetCurrent(0);
      globals->shooter_motor_5->SetCurrent(0);
      globals->shooter_motor_6->SetCurrent(0);
    }

    // // 目标速度PID

    // globals->pid_shooter_2->Update(0, globals->shooter_motor_2->rpm());
    // globals->pid_shooter_3->Update(0, globals->shooter_motor_3->rpm());
    // globals->pid_shooter_4->Update(0, globals->shooter_motor_4->rpm());
    // globals->pid_shooter_5->Update(0, globals->shooter_motor_5->rpm());
    // globals->pid_shooter_6->Update(0, globals->shooter_motor_6->rpm());
    // // 给shooter电机发送指令
    // globals->shooter_motor_1->SetCurrent(static_cast<int16_t>(globals->pid_shooter_1->out()));
    // globals->shooter_motor_2->SetCurrent(static_cast<int16_t>(globals->pid_shooter_2->out()));
    // globals->shooter_motor_3->SetCurrent(static_cast<int16_t>(globals->pid_shooter_3->out()));
    // globals->shooter_motor_4->SetCurrent(static_cast<int16_t>(globals->pid_shooter_4->out()));
    // globals->shooter_motor_5->SetCurrent(static_cast<int16_t>(globals->pid_shooter_5->out()));
    // globals->shooter_motor_6->SetCurrent(static_cast<int16_t>(globals->pid_shooter_6->out()));
    return;
  }
  // 摩擦轮逻辑
  if (globals->tc->key_once(device::VT03::KeyboardKey::kE)) {
    shooter_m -= 10;
  }
  if (globals->tc->key_once(device::VT03::KeyboardKey::kQ)) {
    shooter_m += 10;
  }
  // 目标速度PID
  if (shooter_6 > limit) {
    globals->pid_shooter_1->Update(limit, globals->shooter_motor_1->rpm());
    globals->pid_shooter_2->Update(limit, globals->shooter_motor_2->rpm());
    globals->pid_shooter_3->Update(limit, globals->shooter_motor_3->rpm());
    globals->pid_shooter_4->Update(limit, globals->shooter_motor_4->rpm());
    globals->pid_shooter_5->Update(limit, globals->shooter_motor_5->rpm());
    globals->pid_shooter_6->Update(limit, globals->shooter_motor_6->rpm());
  } else {
    globals->pid_shooter_1->Update(V_shooter_1 + 1.5 * shooter_m, globals->shooter_motor_1->rpm());
    globals->pid_shooter_2->Update(V_shooter_1 + 1.5 * shooter_m, globals->shooter_motor_2->rpm());
    globals->pid_shooter_3->Update(V_shooter_1 + 1.5 * shooter_m, globals->shooter_motor_3->rpm());
    globals->pid_shooter_4->Update(V_shooter_2 + shooter_m, globals->shooter_motor_4->rpm());
    globals->pid_shooter_5->Update(V_shooter_2 + shooter_m, globals->shooter_motor_5->rpm());
    globals->pid_shooter_6->Update(V_shooter_2 + shooter_m, globals->shooter_motor_6->rpm());
  }

  // 给shooter电机发送指令
  globals->shooter_motor_1->SetCurrent(static_cast<int16_t>(globals->pid_shooter_1->out()));
  globals->shooter_motor_2->SetCurrent(static_cast<int16_t>(globals->pid_shooter_2->out()));
  globals->shooter_motor_3->SetCurrent(static_cast<int16_t>(globals->pid_shooter_3->out()));
  globals->shooter_motor_4->SetCurrent(static_cast<int16_t>(globals->pid_shooter_4->out()));
  globals->shooter_motor_5->SetCurrent(static_cast<int16_t>(globals->pid_shooter_5->out()));
  globals->shooter_motor_6->SetCurrent(static_cast<int16_t>(globals->pid_shooter_6->out()));
}

/*----------------------------------------------------*/
// void ChassisControl_deprecated() {
//   // chassis_1 = globals->chassis_motor_1->rpm();
//   // chassis_2 = globals->chassis_motor_2->rpm();
//   // chassis_3 = globals->chassis_motor_3->rpm();
//   // chassis_4 = globals->chassis_motor_4->rpm();
//   // 失能
//   if (r_switch_position_now == rm::device::DR16::SwitchPosition::kDown ||
//       r_switch_position_now == rm::device::DR16::SwitchPosition::kUnknown) {
//     // // 给底盘电机发送指令
//     // globals->pid_chassis_1->Update(0, globals->chassis_motor_1->rpm());
//     // globals->pid_chassis_2->Update(0, globals->chassis_motor_2->rpm());
//     // globals->pid_chassis_3->Update(0, globals->chassis_motor_3->rpm());
//     // globals->pid_chassis_4->Update(0, globals->chassis_motor_4->rpm());
//     // 给chassis电机发送指令
//     globals->chassis_motor_1->SetCurrent(static_cast<int16_t>(0));
//     globals->chassis_motor_2->SetCurrent(static_cast<int16_t>(0));
//     globals->chassis_motor_3->SetCurrent(static_cast<int16_t>(0));
//     globals->chassis_motor_4->SetCurrent(static_cast<int16_t>(0));
//     return;
//   }
//   // 底盘随动
//   if (globals->rc->switch_l() == rm::device::DR16::SwitchPosition::kUp) {
//     globals->pid_chassis_follow->SetCircular(true).SetCircularCycle(3.141593 * 2);
//     globals->pid_chassis_follow->Update(1.5708, globals->gimbal_motor_yaw->pos(),
//                                         0.001); // 云台正位为电机编码器的-90°
//     Vw = globals->pid_chassis_follow->out();
//   } else {
//     Vw = 0;
//   }
//
//   // 遥控器输入底盘速度
//   Vx = globals->rc->left_x() * 10000 / 660;
//   Vy = globals->rc->left_y() * 10000 / 660;
//
//   rm::i16 V_wheel_1 = -Vy + Vx + 0.8 * Vw;
//   rm::i16 V_wheel_2 = Vy + Vx + 0.8 * Vw;
//   rm::i16 V_wheel_3 = Vy - 0.6 * Vx + 0.3 * Vw;
//   rm::i16 V_wheel_4 = -Vy - 0.6 * Vx + 0.3 * Vw;
//
//   // 目标速度PID
//   globals->pid_chassis_1->Update(V_wheel_1, globals->chassis_motor_1->rpm());
//   globals->pid_chassis_2->Update(V_wheel_2, globals->chassis_motor_2->rpm());
//   globals->pid_chassis_3->Update(V_wheel_3, globals->chassis_motor_3->rpm());
//   globals->pid_chassis_4->Update(V_wheel_4, globals->chassis_motor_4->rpm());
//
//   // 给chassis电机发送指令
//   globals->chassis_motor_1->SetCurrent(static_cast<int16_t>(globals->pid_chassis_1->out()));
//   globals->chassis_motor_2->SetCurrent(static_cast<int16_t>(globals->pid_chassis_2->out()));
//   globals->chassis_motor_3->SetCurrent(static_cast<int16_t>(globals->pid_chassis_3->out()));
//   globals->chassis_motor_4->SetCurrent(static_cast<int16_t>(globals->pid_chassis_4->out()));
//   // 发送CAN信号
//   rm::device::DjiMotorBase::SendCommand();
// }

/*----------------------------------------------------*/
inline f32 pitch_ff = 2000;

void GimbalControl() {
  // IMU解算
  globals->imu->Update();
  globals->ahrs.Update(rm::modules::ImuData6Dof{globals->imu->gyro_x(), globals->imu->gyro_y(),
                                                gyro_z = globals->gyro_z_filter.apply(globals->imu->gyro_z()) -
                                                         /*0.00425*/ 0.00005 - eulerangle_pitch / 0.6644 * 0.0042
                                                // - average1
                                                // - globals->ahrs.euler_angle().pitch * average1 * 10 //2°
                                                ,
                                                globals->imu->accel_x(), globals->imu->accel_y(),
                                                globals->imu->accel_z()});
  eulerangle_yaw = -globals->ahrs.euler_angle().yaw;
  eulerangle_pitch = -globals->ahrs.euler_angle().pitch;
  eulerangle_roll = -globals->ahrs.euler_angle().roll;

  // 监测imu
  Gy = globals->imu->gyro_y();
  Gz = gyro_z;
  // globals->gyro_z_filter.apply(globals->imu->gyro_z());
  Gx = globals->imu->gyro_x();
  // 是否启动
  if (r_switch_position_now == rm::device::DR16::SwitchPosition::kDown ||
      r_switch_position_now == rm::device::DR16::SwitchPosition::kUnknown) {
    globals->gimbal_motor_yaw->SetMitCommand(0, 0, 0, 0, 0);
    globals->gimbal_motor_pitch->SetCurrent(0);
    globals->gimbal_motor_yaw->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    target_pos_yaw = -globals->ahrs.euler_angle().yaw;
    target_pos_pitch = -globals->ahrs.euler_angle().pitch;
    // HAL_Delay(0);
    return;
  }

  // 云台电机逻辑

  // 启动云台
  if (r_switch_position_last == rm::device::DR16::SwitchPosition::kDown ||
      (power_management_gimbal_last == 0 && globals->ref.data().robot_status.power_management_gimbal_output == 1)) {
    globals->gimbal_motor_yaw->SendInstruction(rm::device::DmMotorInstructions::kClearError);
    globals->gimbal_motor_yaw->SendInstruction(rm::device::DmMotorInstructions::kEnable);
  }

  // 遥控器输入云台角度
  aimbot_state_flag = globals->aimbot_can_communicator->aimbot_target();
  if (aimbot_state_flag > 0 &&
      (((globals->rc->dial() >= 500 || globals->rc->dial() <= -500) &&
        l_switch_position_now != device::DR16::SwitchPosition::kUp) ||
       r_switch_position_now == device::DR16::SwitchPosition::kUp || globals->rc->mouse_button_right() ||
       globals->tc->data().mouse_button_right || globals->custom_client->mouse_right() ||
       globals->tc->data().keyboard_key & static_cast<int16_t>(VT03::KeyboardKey::kCtrl))) {
    target_pos_yaw = -globals->aimbot_can_communicator->yaw() / 57.3;

    //-aimbot.USB_Rx.YawRelativeAngle;usb

    target_pos_pitch = -globals->aimbot_can_communicator->pitch() / 57.3;

    //-aimbot.USB_Rx.PitchRelativeAngle;usb
    aimbot_state_flag = 0;
  } else {
    // tc->rc->cc
    if ((globals->tc->data().mouse_x != 0 || globals->tc->data().mouse_y != 0) && globals->tc->offline_count < 93) {
      target_pos_yaw += static_cast<float>(globals->rc->right_x()) * 0.000005 +
                        static_cast<float>(globals->tc->data().mouse_x) / 32768 * 0.8;
      target_pos_pitch += static_cast<float>(globals->rc->right_y()) * 0.0000005 +
                          static_cast<float>(globals->tc->data().mouse_y) / 32768 * 0.5;
    } else if (globals->rc->mouse_x() != 0 || globals->rc->mouse_y() != 0) {
      target_pos_yaw += static_cast<float>(globals->rc->right_x()) * 0.000005 +
                        static_cast<float>(globals->rc->mouse_x()) / 32768.0 * 3;  // ≈0.003/per
      target_pos_pitch += static_cast<float>(globals->rc->right_y()) * 0.0000005 +
                          static_cast<float>(globals->rc->mouse_y() / 32768.0 * 3);
    } else {
      target_pos_yaw += static_cast<float>(globals->rc->right_x()) * 0.000005 +
                        static_cast<float>(globals->custom_client->mouse_x()) * 0.000015;  // ≈0.003/per
      target_pos_pitch += static_cast<float>(globals->rc->right_y()) * 0.0000005 +
                          static_cast<float>(globals->custom_client->mouse_y()) * 0.000015;  // 0.00033/per
    }

    aimbot_state_flag = 0;
  }

  // target_pos_yaw
  // yaw限位
  // if (target_pos_yaw < -1.85) {
  //   target_pos_yaw = -1.85;
  // } else if (target_pos_yaw > 1.64) {
  //   target_pos_yaw = 1.64;
  // }
  // pitch限位
  if (target_pos_pitch < 0) {
    target_pos_pitch = 0;
  } else if (target_pos_pitch > 0.6644) {
    target_pos_pitch = 0.6644;
  }

  // PID计算
  // if (globals->rc->switch_r() == rm::device::DR16::SwitchPosition::kMid) {
  //   globals->pid_yaw_position->Update(target_pos_yaw, globals->gimbal_motor_yaw->pos(), 0.002);
  //   // globals->pid_yaw_velocity->Update(globals->pid_yaw_position->out(), globals->gimbal_motor_yaw->vel(),
  //   0.002);
  //   // globals->pid_pitch_velocity->Update(globals->ahrs.euler_angle().pitch, 0.002);
  //   globals->gimbal_motor_yaw->SetPosition(0, 0, globals->pid_yaw_position->out(), 0, 0);
  //   HAL_Delay(0);
  // } else {
  //   globals->gimbal_motor_yaw->SetPosition(0, 0, 0, 0, 0);
  // }
  /*---------*/

  // PID计算（使用IMU）

  // yawPID计算（双环）
  globals->pid_yaw_position->SetCircular(true).SetCircularCycle(3.141593 * 2);
  globals->pid_yaw_position->Update(target_pos_yaw, -globals->ahrs.euler_angle().yaw - 0.005, 0.001);
  globals->pid_yaw_velocity->Update(globals->pid_yaw_position->out(), -globals->imu->gyro_z(), 0.001);
  // pitchPID计算
  globals->pid_pitch_position->Update(target_pos_pitch, -globals->ahrs.euler_angle().pitch, 1);
  globals->pid_pitch_velocity->Update(globals->pid_pitch_position->out(), -globals->imu->gyro_y(), 0.001);

  // 发送CAN
  globals->gimbal_motor_yaw->SetMitCommand(
      0, 0, (0.98 - 0.5 * eulerangle_pitch / 0.6644) * globals->pid_yaw_velocity->out(), 0, 0);
  // 爬坡模式
  if (r_switch_position_now == rm::device::DR16::SwitchPosition::kUp &&
      l_switch_position_now != device::DR16::SwitchPosition::kUp &&
      (globals->rc->dial() < 500 && globals->rc->dial() > -500)) {
    globals->gimbal_motor_pitch->SetCurrent(0);
  } else {
    globals->gimbal_motor_pitch->SetCurrent(
        static_cast<int16_t>((1.7 + 1.3 * globals->ahrs.euler_angle().pitch) * pitch_ff) +
        (0.5 + 0.5 * eulerangle_pitch / 0.6644) * (globals->pid_pitch_velocity->out()) /*+out_feedforward*/);
  }

  // HAL_Delay(0);

  // 监测pid
  pitch_out = globals->pid_pitch_velocity->out();
  yaw_out = globals->gimbal_motor_yaw->pos();
  error = globals->pid_yaw_position->error()[1];
  yaw_state = globals->gimbal_motor_yaw->status();
}

/*------------*/

/*-----------*/

void ChassisPower() {
  // 失能
  if (r_switch_position_now == rm::device::DR16::SwitchPosition::kDown ||
      r_switch_position_now == rm::device::DR16::SwitchPosition::kUnknown ||
      (!globals->ref.data().robot_status.power_management_chassis_output)) {
    for (int i = 0; i < 4; i++) {
      globals->chassis_motor[i]->SetCurrent(0);
    }
    return;
  }
  if (globals->tc->data().keyboard_key & static_cast<int16_t>(VT03::KeyboardKey::kC)) {
    follow_state = false;
  } else {
    follow_state = true;
  }

  if (follow_state) {
    globals->pid_chassis_follow_pos->SetCircular(true).SetCircularCycle(3.141593 * 2);
    globals->pid_chassis_follow_pos->Update(0.49, globals->gimbal_motor_yaw->pos(),
                                            0.0011);  // 云台正位为电机编码器的+90°//逆时针旋转为增大
    globals->pid_chassis_follow_vel->Update(globals->pid_chassis_follow_pos->out(), globals->gimbal_motor_yaw->vel(),
                                            0.0011);
    Vw = static_cast<rm::i16>(globals->pid_chassis_follow_vel->out()) * (1 - eulerangle_pitch / 0.6644 * 0.7);
  } else {
    Vw = 0;
  }
  pos_target = 1.54;
  pos_real = globals->gimbal_motor_yaw->pos();
  vel_target = globals->pid_chassis_follow_pos->out();
  vel_real = globals->gimbal_motor_yaw->vel();
  // follow = Vw;

  // 遥控器输入底盘速度
  if (l_switch_position_now == device::DR16::SwitchPosition::kUp ||
      l_switch_position_now == device::DR16::SwitchPosition::kMid) {
    if (globals->rc->key(rm::device::DR16::Key::kW) ||
        (globals->tc->data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kW) &&
         globals->tc->offline_count < 93) ||
        globals->custom_client->key(rm::device::DR16::Key::kW)) {
      Vy += 30;
      if (Vy >= 8400) {
        Vy = 8500;
      }
    } else if (globals->rc->key(rm::device::DR16::Key::kS) ||
               (globals->tc->data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kS) &&
                globals->tc->offline_count < 93) ||
               globals->custom_client->key(rm::device::DR16::Key::kS)) {
      Vy -= 30;
      if (Vy <= -8400) {
        Vy = -8500;
      }
    } else {
      Vy = 0;
    }
    if (globals->rc->key(rm::device::DR16::Key::kD) ||
        (globals->tc->data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kD) &&
         globals->tc->offline_count < 93) ||
        globals->custom_client->key(rm::device::DR16::Key::kD)) {
      Vx += 30;
      if (Vx >= 8400) {
        Vx = 7000;
      }
    } else if (globals->rc->key(rm::device::DR16::Key::kA) ||
               (globals->tc->data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kA) &&
                globals->tc->offline_count < 93) ||
               globals->custom_client->key(rm::device::DR16::Key::kA)) {
      Vx -= 30;
      if (Vx <= -8400) {
        Vx = -7000;
      }
    } else {
      Vx = 0;
    }
  } else {
    Vx = globals->rc->left_x() * 7000 / 660;
    Vy = globals->rc->left_y() * 8500 / 660;
  }

  rm::i16 V_wheel[4];
  V_wheel[0] = -Vy + 1.5 * Vx + 1.5 * Vw;
  V_wheel[1] = Vy + 1.5 * Vx + 1.5 * Vw;
  V_wheel[2] = Vy /*- 1.5 * Vx */ + 1 * Vw;
  V_wheel[3] = -Vy /*- 1.5 * Vx*/ + 1 * Vw;

  for (int i = 0; i < 4; i++) {
    globals->velocity_pids[i]->Update(V_wheel[i], globals->chassis_motor[i]->rpm());
    initial_currents[i] = globals->velocity_pids[i]->out();
    // 构建电机状态
    (*globals->motor_states)[i].speed_rpm = globals->chassis_motor[i]->rpm();
    (*globals->motor_states)[i].give_current = initial_currents[i];
    (*globals->motor_states)[i].measured_current = globals->chassis_motor[i]->current();
  }
  float buffer_energy = globals->ref.data().buff.remaining_energy;
  overpower = false;
  if (r_switch_position_now == DR16::SwitchPosition::kUp) {
    overpower = true;
  } else if (globals->tc->data().keyboard_key & static_cast<int16_t>(VT03::KeyboardKey::kShift) ||
             globals->rc->key(DR16::Key::kShift)) {
    overpower = true;
  } else {
    overpower = false;
  }
  if (overpower) {
    // 超功率
    power_limit = 130;  // 随便给的
  } else {
    power_limit = globals->ref.data().robot_status.chassis_power_limit == 0
                      ? 50
                      : static_cast<float>(globals->ref.data().robot_status.chassis_power_limit);
  }

  power_model.DistributePower<4>(*globals->motor_states, initial_currents, power_limit, output_currents);

  // 电容离线或电压过低则不允许超功率
  if (globals->cms->cms_v < 15) {
    overpower = false;
  }
  if (overpower) {
    if (globals->ref.data().power_heat_data.buffer_energy <= 30) {
      for (int i = 0; i < 4; i++)
        globals->chassis_motor[i]->SetCurrent(
            static_cast<int16_t>(output_currents[i] * (globals->ref.data().power_heat_data.buffer_energy) / 60));
    } else {
      for (int i = 0; i < 4; i++) globals->chassis_motor[i]->SetCurrent(static_cast<int16_t>(output_currents[i]));
    }
  } else {
    if (globals->ref.data().power_heat_data.buffer_energy >= 50) {
      for (int i = 0; i < 4; i++) globals->chassis_motor[i]->SetCurrent(static_cast<int16_t>(output_currents[i]));
    } else {
      for (int i = 0; i < 4; i++)
        globals->chassis_motor[i]->SetCurrent(
            static_cast<int16_t>(output_currents[i] * (globals->ref.data().power_heat_data.buffer_energy) / 60));
    }
  }

  rm::device::DjiMotorBase::SendCommand();

  // 监测值
  P_chassis_1 = output_currents[0];
  P_chassis_2 = output_currents[1];
  P_chassis_3 = output_currents[2];
  P_chassis_4 = output_currents[3];
  V_chassis_1 = globals->chassis_motor[0]->rpm();
  V_chassis_2 = globals->chassis_motor[1]->rpm();
  V_chassis_3 = globals->chassis_motor[2]->rpm();
  V_chassis_4 = globals->chassis_motor[3]->rpm();
}

void AutoaimUpdate() {
  aimbot.Prepare();
  aimbot.Send();
  aimbot.Receive(UserRxBuf, UserRxLen);

  // aimbot_state_flag = aimbot.USB_Rx.AimbotState * 5;
}

void CANAutoaimUpdate() {
  if (imu_count >= 10000) {
    imu_count = 0;
  } else {
    imu_count++;
  }

  globals->aimbot_can_communicator->UpdateControl(globals->ahrs.euler_angle().yaw, globals->ahrs.euler_angle().pitch,
                                                  globals->ahrs.euler_angle().roll,
                                                  globals->ref.data().robot_status.robot_id, 1, imu_count, 11.8);
  aimbot_pitch = -globals->aimbot_can_communicator->pitch() / 57.3;
  aimbot_yaw = -globals->aimbot_can_communicator->yaw() / 57.3;
}

void CustomClientUpdate() { globals->custom_client->Unpack(UserRxBuf, UserRxLen); }

Vofa_TxFrame shooter;

void VOFA() {
  float swhell[6];
  // swhell[0]=-globals->aimbot_can_communicator->yaw();
  // swhell[1]=-globals->aimbot_can_communicator->pitch();
  swhell[0] = globals->imu->gyro_z();
  swhell[1] = shooter_2;
  swhell[2] = shooter_3;
  swhell[3] = shooter_4;
  swhell[4] = shooter_5;
  swhell[5] = shooter_6;
  VOFA_Prepare_Package(swhell, shooter, 6);
  VOFA_Send_JustFloat_DMA(&huart1, shooter);
}

// void SuperCupUpdate() {
//   globals->cms->SendCapBuffer(globals->ref.data().power_heat_data.buffer_energy);
//   globals->cms->SendCapPower(globals->ref.data().robot_status.chassis_power_limit);
// }