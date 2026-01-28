#include "main.hpp"

void MagazineControl() {
  // 失能
  if ((l_switch_position_now != rm::device::DR16::SwitchPosition::kMid &&
       l_switch_position_now != rm::device::DR16::SwitchPosition::kUp) ||
      r_switch_position_now == rm::device::DR16::SwitchPosition::kDown ||
      r_switch_position_now == rm::device::DR16::SwitchPosition::kUnknown) {
    globals->magazine_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    // HAL_Delay(0);
    return;
  }
  // 启动
  if (l_switch_position_last == rm::device::DR16::SwitchPosition::kDown) {
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
    globals->magazine_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
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
    if ((globals->rc->dial() >= 500 || globals->rc->dial() < -500) && shooter_4 < -400) {
      // 堵转检测
      if (rm::modules::Wrap(target_magz - globals->magazine_motor->pos(), -3.141593, 3.141593) < -3.141593 / 18) {
        target_magz = globals->magazine_motor->pos() + 3.141593 / 90;
      } else {
        target_magz = next_target_magz;
        next_target_magz -= 3.141593 / 3;
        if (next_target_magz < -3.141593) {
          next_target_magz += 2 * 3.141593;
        }
      }
      // target_magz -= 1.0472 + magz_compensation /*（π/3）*/;
      // if (target_magz <= -3.141593 /*（π）*/) {
      //   target_magz += 3.141593 * 2;
      // }
      counter = 500;
      // magz_compensation = 0;
    }
  } else {
    counter--;
  }

  // 拨盘电机串级PID（开循环）
  globals->pid_magz_position->SetCircular(true).SetCircularCycle(3.141593 * 2);
  globals->pid_magz_position->Update(target_magz, globals->magazine_motor->pos(), 0.001);
  // target_velocity = globals->pid_magz_position->out();
  // globals->pid_magz_velocity->Update(target_velocity, globals->magazine_motor->vel(), 0.002);
  globals->magazine_motor->SetMitCommand(0, 0, globals->pid_magz_position->out(), 0, 0);
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
  if ((l_switch_position_now != rm::device::DR16::SwitchPosition::kMid &&
       l_switch_position_now != rm::device::DR16::SwitchPosition::kUp) ||
      r_switch_position_now == rm::device::DR16::SwitchPosition::kDown ||
      r_switch_position_now == rm::device::DR16::SwitchPosition::kUnknown) {
    globals->pid_shooter_1->Update(0, globals->shooter_motor_1->rpm());
    // 给shooter电机发送指令
    globals->shooter_motor_1->SetCurrent(globals->pid_shooter_1->out());
    globals->shooter_motor_2->SetCurrent(0);
    globals->shooter_motor_3->SetCurrent(0);
    globals->shooter_motor_4->SetCurrent(0);
    globals->shooter_motor_5->SetCurrent(0);
    globals->shooter_motor_6->SetCurrent(0);
    rm::device::DjiMotor<>::SendCommand();

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

  // 目标速度PID
  globals->pid_shooter_1->Update(V_shooter_1, globals->shooter_motor_1->rpm());
  globals->pid_shooter_2->Update(V_shooter_1, globals->shooter_motor_2->rpm());
  globals->pid_shooter_3->Update(V_shooter_1, globals->shooter_motor_3->rpm());
  globals->pid_shooter_4->Update(V_shooter_2, globals->shooter_motor_4->rpm());
  globals->pid_shooter_5->Update(V_shooter_2, globals->shooter_motor_5->rpm());
  globals->pid_shooter_6->Update(V_shooter_2, globals->shooter_motor_6->rpm());

  // 给shooter电机发送指令
  globals->shooter_motor_1->SetCurrent(static_cast<int16_t>(globals->pid_shooter_1->out()));
  globals->shooter_motor_2->SetCurrent(static_cast<int16_t>(globals->pid_shooter_2->out()));
  globals->shooter_motor_3->SetCurrent(static_cast<int16_t>(globals->pid_shooter_3->out()));
  globals->shooter_motor_4->SetCurrent(static_cast<int16_t>(globals->pid_shooter_4->out()));
  globals->shooter_motor_5->SetCurrent(static_cast<int16_t>(globals->pid_shooter_5->out()));
  globals->shooter_motor_6->SetCurrent(static_cast<int16_t>(globals->pid_shooter_6->out()));

  rm::device::DjiMotor<>::SendCommand();
}

/*----------------------------------------------------*/
void ChassisControl() {
  chassis_1 = globals->chassis_motor_1->rpm();
  chassis_2 = globals->chassis_motor_2->rpm();
  chassis_3 = globals->chassis_motor_3->rpm();
  chassis_4 = globals->chassis_motor_4->rpm();
  // 失能
  if (r_switch_position_now == rm::device::DR16::SwitchPosition::kDown ||
      r_switch_position_now == rm::device::DR16::SwitchPosition::kUnknown) {
    // // 给底盘电机发送指令
    // globals->pid_chassis_1->Update(0, globals->chassis_motor_1->rpm());
    // globals->pid_chassis_2->Update(0, globals->chassis_motor_2->rpm());
    // globals->pid_chassis_3->Update(0, globals->chassis_motor_3->rpm());
    // globals->pid_chassis_4->Update(0, globals->chassis_motor_4->rpm());
    // 给chassis电机发送指令
    globals->chassis_motor_1->SetCurrent(static_cast<int16_t>(0));
    globals->chassis_motor_2->SetCurrent(static_cast<int16_t>(0));
    globals->chassis_motor_3->SetCurrent(static_cast<int16_t>(0));
    globals->chassis_motor_4->SetCurrent(static_cast<int16_t>(0));
    return;
  }
  // 底盘随动
  if (globals->rc->switch_l() == rm::device::DR16::SwitchPosition::kUp) {
    globals->pid_chassis_follow->SetCircular(true).SetCircularCycle(3.141593 * 2);
    globals->pid_chassis_follow->Update(1.5708, globals->gimbal_motor_yaw->pos(),
                                        0.001);  // 云台正位为电机编码器的-90°
    Vw = globals->pid_chassis_follow->out();
  } else {
    Vw = 0;
  }

  // 遥控器输入底盘速度
  Vx = globals->rc->left_x() * 10000 / 660;
  Vy = globals->rc->left_y() * 10000 / 660;

  rm::i16 V_wheel_1 = -Vy + Vx + 0.8 * Vw;
  rm::i16 V_wheel_2 = Vy + Vx + 0.8 * Vw;
  rm::i16 V_wheel_3 = Vy - 0.6 * Vx + 0.3 * Vw;
  rm::i16 V_wheel_4 = -Vy - 0.6 * Vx + 0.3 * Vw;

  // 目标速度PID
  globals->pid_chassis_1->Update(V_wheel_1, globals->chassis_motor_1->rpm());
  globals->pid_chassis_2->Update(V_wheel_2, globals->chassis_motor_2->rpm());
  globals->pid_chassis_3->Update(V_wheel_3, globals->chassis_motor_3->rpm());
  globals->pid_chassis_4->Update(V_wheel_4, globals->chassis_motor_4->rpm());

  // 给chassis电机发送指令
  globals->chassis_motor_1->SetCurrent(static_cast<int16_t>(globals->pid_chassis_1->out()));
  globals->chassis_motor_2->SetCurrent(static_cast<int16_t>(globals->pid_chassis_2->out()));
  globals->chassis_motor_3->SetCurrent(static_cast<int16_t>(globals->pid_chassis_3->out()));
  globals->chassis_motor_4->SetCurrent(static_cast<int16_t>(globals->pid_chassis_4->out()));
  // 发送CAN信号
  rm::device::DjiMotor<>::SendCommand();
}

/*----------------------------------------------------*/
void GimbalControl() {
  // IMU解算
  globals->imu->Update();
  globals->ahrs.Update(rm::modules::ImuData6Dof{
      globals->imu->gyro_x(), globals->imu->gyro_y(), globals->imu->gyro_z() + static_cast<float>(0.000088),
      globals->imu->accel_x(), globals->imu->accel_y(), globals->imu->accel_z()});
  eulerangle_yaw = -globals->ahrs.euler_angle().yaw;
  eulerangle_pitch = -globals->ahrs.euler_angle().pitch;
  eulerangle_roll = -globals->ahrs.euler_angle().roll;
  // 监测imu
  Gy = globals->imu->gyro_y();
  Gz = globals->imu->gyro_z();
  Gx = globals->imu->gyro_x();
  if (r_switch_position_now == rm::device::DR16::SwitchPosition::kDown ||
      r_switch_position_now == rm::device::DR16::SwitchPosition::kUnknown) {
    globals->gimbal_motor_yaw->SetMitCommand(0, 0, 0, 0, 0);
    globals->gimbal_motor_pitch->SetCurrent(0);
    globals->gimbal_motor_yaw->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    target_pos_yaw = -globals->ahrs.euler_angle().yaw;
    // HAL_Delay(0);
    return;
  }

  // 云台电机逻辑

  // 启动云台
  if (r_switch_position_last == rm::device::DR16::SwitchPosition::kDown) {
    globals->gimbal_motor_yaw->SendInstruction(rm::device::DmMotorInstructions::kClearError);
    globals->gimbal_motor_yaw->SendInstruction(rm::device::DmMotorInstructions::kEnable);
  }

  // 遥控器输入云台角度
  target_pos_yaw += static_cast<float>(globals->rc->right_x()) * 0.00001;  //
  target_pos_pitch += static_cast<float>(globals->rc->right_y()) * 0.0000005;
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
  globals->pid_yaw_velocity->Update(globals->pid_yaw_velocity->out(), -globals->imu->gyro_z(), 0.002);
  // pitchPID计算
  globals->pid_pitch_position->Update(target_pos_pitch, -globals->ahrs.euler_angle().pitch, 1);
  globals->pid_pitch_velocity->Update(globals->pid_pitch_position->out(), -globals->imu->gyro_y(), 0.001);

  // 发送CAN
  globals->gimbal_motor_yaw->SetMitCommand(0, 0, globals->pid_yaw_position->out(), 0, 0);
  globals->gimbal_motor_pitch->SetCurrent(
      static_cast<int16_t>(globals->pid_pitch_velocity->out()) /*+out_feedforward*/);
  // HAL_Delay(0);

  // 监测pid
  pitch_out = globals->pid_pitch_velocity->out();
  yaw_out = globals->gimbal_motor_yaw->pos();
  error = globals->pid_yaw_position->error()[1];
  yaw_state = globals->gimbal_motor_yaw->status();
}

/*------------*/
// void VOFA() {
//   rm::modules::VofaPlotter
// }