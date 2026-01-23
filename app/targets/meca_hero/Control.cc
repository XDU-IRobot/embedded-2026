#include "main.hpp"

void MagazineControl() {
  // 拨盘电机逻辑
  l_switch_position_last = l_switch_position_now;
  l_switch_position_now = globals->rc->switch_l();
  if (l_switch_position_now == rm::device::DR16::SwitchPosition::kMid) {
    // 初始化射击模式
    if (l_switch_position_last != rm::device::DR16::SwitchPosition::kMid) {
      globals->magazine_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
      // globals->magazine_motor->SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
      // target_magz = 0.0f;
    }
    // 按下扳机(延时1s)
    if (counter == 0) {
      if (globals->rc->dial() >= 500 || globals->rc->dial() < -500) {
        // 增加60°
        target_magz -= 1.0472 /*（π/3）*/;
        if (target_magz <= -3.141593 /*（π）*/) {
          target_magz += 3.141593 * 2;
        }
        counter = 250;
      }
    } else {
      counter--;
    }

    // 拨盘电机串级PID（开循环）
    globals->pid_magz_position->SetCircular(true).SetCircularCycle(3.141593 * 2);
    globals->pid_magz_position->Update(target_magz, globals->magazine_motor->pos(), 0.002);
    // target_velocity = globals->pid_magz_position->out();
    // globals->pid_magz_velocity->Update(target_velocity, globals->magazine_motor->vel(), 0.002);
    // 发送CAN
    if (globals->rc->switch_r() == rm::device::DR16::SwitchPosition::kDown) {
      globals->magazine_motor->SetPosition(0, 0, 0, 0, 0);
    } else {
      globals->magazine_motor->SetPosition(0, 0, globals->pid_magz_position->out(), 0, 0);
    }
  }
  // 失能
  if (l_switch_position_last == rm::device::DR16::SwitchPosition::kMid &&
      l_switch_position_now != rm::device::DR16::SwitchPosition::kMid) {
    HAL_Delay(0);
    globals->magazine_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    HAL_Delay(0);
  }
}

/*----------------------------------------------------*/
// 摩擦轮逻辑
void ShooterControl() {
  // 摩擦轮逻辑
  if (globals->rc->switch_r() == rm::device::DR16::SwitchPosition::kDown) {
    // 给摩擦轮电机发送指令
    globals->pid_shooter_1->Update(0, globals->shooter_motor_1->rpm());
    globals->pid_shooter_2->Update(0, globals->shooter_motor_2->rpm());
    globals->pid_shooter_3->Update(0, globals->shooter_motor_3->rpm());
    globals->pid_shooter_4->Update(0, globals->shooter_motor_4->rpm());
    globals->pid_shooter_5->Update(0, globals->shooter_motor_5->rpm());
    globals->pid_shooter_6->Update(0, globals->shooter_motor_6->rpm());

    // 给shooter电机发送指令
    globals->shooter_motor_1->SetCurrent(static_cast<int16_t>(0));
    globals->shooter_motor_2->SetCurrent(static_cast<int16_t>(0));
    globals->shooter_motor_3->SetCurrent(static_cast<int16_t>(0));
    globals->shooter_motor_4->SetCurrent(static_cast<int16_t>(0));
    globals->shooter_motor_5->SetCurrent(static_cast<int16_t>(0));
    globals->shooter_motor_6->SetCurrent(static_cast<int16_t>(0));
  } else if (globals->rc->switch_r() == rm::device::DR16::SwitchPosition::kMid) {
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
  }
  rm::device::DjiMotor<>::SendCommand();
}

/*----------------------------------------------------*/
void ChassisControl() {
  // 遥控器输入底盘速度
  Vx = globals->rc->left_x() * 10000 / 660;
  Vy = globals->rc->left_y() * 10000 / 660;
  // Vw = globals->rc->dial() * 30000 / 660;

  // 移动逻辑
  // if (Vy < -100) {
  //   Vx = -Vx;
  // }
  rm::i16 V_wheel_1 = -Vy + Vx;
  rm::i16 V_wheel_2 = Vy + Vx;
  rm::i16 V_wheel_3 = Vy + 0.5 * Vx;
  rm::i16 V_wheel_4 = -Vy + 0.5 * Vx;
  if (globals->rc->switch_r() == rm::device::DR16::SwitchPosition::kDown) {
    // 给底盘电机发送指令
    globals->pid_chassis_1->Update(0, globals->chassis_motor_1->rpm());
    globals->pid_chassis_2->Update(0, globals->chassis_motor_2->rpm());
    globals->pid_chassis_3->Update(0, globals->chassis_motor_3->rpm());
    globals->pid_chassis_4->Update(0, globals->chassis_motor_4->rpm());
    // 给chassis电机发送指令
    globals->chassis_motor_1->SetCurrent(static_cast<int16_t>(globals->pid_chassis_1->out()));
    globals->chassis_motor_2->SetCurrent(static_cast<int16_t>(globals->pid_chassis_2->out()));
    globals->chassis_motor_3->SetCurrent(static_cast<int16_t>(globals->pid_chassis_3->out()));
    globals->chassis_motor_4->SetCurrent(static_cast<int16_t>(globals->pid_chassis_4->out()));
  } else {
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
  }
  rm::device::DjiMotor<>::SendCommand();
}

/*----------------------------------------------------*/
void GimbalControl() {
  //IMU解算
  globals->imu->Update();
  globals->ahrs.Update(rm::modules::ImuData6Dof{-globals->imu->gyro_y(),
                                                -globals->imu->gyro_x(),
                                                -globals->imu->gyro_z(),
                                                -globals->imu->accel_y(),
                                                -globals->imu->accel_x(),
                                                -globals->imu->accel_z()});
  // // 云台电机逻辑
  // l_switch_position_last = l_switch_position_now;
  // l_switch_position_now = globals->rc->switch_l();
  if (l_switch_position_now == rm::device::DR16::SwitchPosition::kMid || l_switch_position_now ==
      rm::device::DR16::SwitchPosition::kUp) {
    // 初始化云台模式
    if (l_switch_position_last != rm::device::DR16::SwitchPosition::kMid &&
        l_switch_position_last != rm::device::DR16::SwitchPosition::kUp) {
      globals->gimbal_motor_yaw->SendInstruction(rm::device::DmMotorInstructions::kEnable);
      // globals->gimbal_motor_yaw->SendInstruction(rm::device::DmMotorInstructions::kClearError);
    }
    // 遥控器输入云台角度
    pos_yaw -= static_cast<float>(globals->rc->right_x()) * 0.000005; //45° per s / 660.0
    pos_pitch += globals->rc->right_y();
    //yaw限位
    if (pos_yaw < -1.85) {
      pos_yaw = -1.85;
    } else if (pos_yaw > 1.64) {
      pos_yaw = 1.64;
    }
    //pitch限位
    // if (pos_pitch) {
    // } else if () {
    // }
    // PID计算
    if (globals->rc->switch_r() == rm::device::DR16::SwitchPosition::kMid) {
      globals->pid_yaw_position->Update(pos_yaw, globals->gimbal_motor_yaw->pos(), 0.002);
      // globals->pid_yaw_velocity->Update(globals->pid_yaw_position->out(), globals->gimbal_motor_yaw->vel(), 0.002);

      // globals->pid_pitch_velocity->Update(globals->ahrs.euler_angle().pitch, 0.002);
      HAL_Delay(0);
      globals->gimbal_motor_yaw->SetPosition(0, 0, globals->pid_yaw_position->out(), 0, 0);
      HAL_Delay(0);
    } else {

      globals->gimbal_motor_yaw->SetPosition(0, 0, 0, 0, 0);
    }
    /*---------*/
    //PID计算（使用IMU）
     // if (globals->rc->switch_r() == rm::device::DR16::SwitchPosition::kMid) {
     //   globals->pid_yaw_position->Update(pos,globals->ahrs.euler_angle().yaw,0.002);
     //   globals->pid_yaw_velocity->Update(globals->pid_yaw_position->out(), globals->ahrs.euler_angle().yaw, 0.002);
     //   globals->pid_pitch_position->Update(globals->ahrs.euler_angle().pitch,0.002);
     }


  if (l_switch_position_last == rm::device::DR16::SwitchPosition::kMid &&
      l_switch_position_now != rm::device::DR16::SwitchPosition::kMid) {
    HAL_Delay(0);
    globals->gimbal_motor_yaw->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    HAL_Delay(0);
  }
}