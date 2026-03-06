#include "Gimbal.hpp"

void Gimbal::GimbalInit() {
  gimbal->gimbal_yaw_target_ = globals->ahrs.euler_angle().yaw;
  gimbal->gimbal_pitch_target_ = globals->ahrs.euler_angle().pitch;
  // gimbal->gimbal_yaw_target_ = globals->hipnuc_imu->yaw();
  // gimbal->gimbal_pitch_target_ = globals->hipnuc_imu->roll();
}

void Gimbal::GimbalTask() { gimbal->GimbalStateUpdate(); }

void Gimbal::GimbalStateUpdate() {
  if (!globals->device_gimbal.all_device_ok()) {
    gimbal->GimbalDisableUpdate();
  } else {
    switch (globals->StateMachine_) {
      case kNoForce:                    // 无力模式下，所有电机失能
        gimbal->GimbalDisableUpdate();  // 云台电机失能计算
        break;

      case kTest:  // 测试模式下，发射系统与拨盘电机失能
        switch (gimbal->GimbalMove_) {
          case kGbRemote:
          case kGbAimbot:
            gimbal->GimbalEnableUpdate();  // 云台电机使能计算
            break;

          default:
            gimbal->GimbalDisableUpdate();  // 云台电机失能计算
            break;
        }
        break;
      case kSineSweepYaw:
        gimbal->GimbalEnableUpdate();  // 云台电机使能计算
        globals->gimbal_controller.output().yaw = static_cast<float>(globals->sine_sweep_yaw->Next());

        break;
      default:                          // 错误状态，所有电机失能
        gimbal->GimbalDisableUpdate();  // 云台电机失能计算
        break;
    }
  }
}
float amp = 2.0f;   // 幅值，比如目标值在±10范围内变化
float freq = 1.0f;  // 频率，1Hz表示1秒完成一个周期
float t = 0.0f;     // 时间变量
float dt = 0.002f;  // 控制循环的时间步长（比如10ms执行一次）
void Gimbal::GimbalRCTargetUpdate() {
  gimbal->gimbal_yaw_target_ -= rm::modules::Map(globals->rc->left_x(), -globals->rc_max_value_, globals->rc_max_value_,
                                                 -gimbal->sensitivity_, gimbal->sensitivity_);  // 上部yaw轴目标值
  gimbal->gimbal_pitch_target_ -= rm::modules::Map(globals->rc->left_y(), -globals->rc_max_value_,  // pitch轴目标值
                                                   globals->rc_max_value_, -gimbal->sensitivity_, gimbal->sensitivity_);
  gimbal->gimbal_yaw_target_ =
      rm::modules::Wrap(gimbal->gimbal_yaw_target_, -static_cast<f32>(M_PI), M_PI);  // yaw轴周期限位
  gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_,    // pitch轴限位
                                                    gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);

  // // 核心：正弦波计算公式
  // gimbal->gimbal_yaw_target_ = amp * sin(2 * M_PI * freq * t);
  // // 时间累加（保证正弦波连续变化）
  // t += dt;
  // // 可选：重置时间防止数值过大（非必须，只是优化）
  // if (t > 1000.0f) t = 0.0f;
}

void Gimbal::GimbalAimbotTargetUpdate() {
  if (globals->can_communicator->aimbot_state() >> 0 & 0x01) {
    gimbal->gimbal_yaw_target_ = globals->can_communicator->yaw();
    gimbal->gimbal_pitch_target_ = globals->can_communicator->pitch();
  } else if (globals->Aimbot.AimbotState >> 0 & 0x01) {
    gimbal->gimbal_yaw_target_ = globals->Aimbot.Yaw;
    gimbal->gimbal_pitch_target_ = globals->Aimbot.Pitch;
  } else {
    // gimbal->gimbal_yaw_target_ -=
    //     rm::modules::Map(globals->rc->left_x(), -globals->rc_max_value_, globals->rc_max_value_,
    //     -gimbal->sensitivity_,
    //                      gimbal->sensitivity_);  // 上部yaw轴目标值
    // gimbal->gimbal_pitch_target_ -=
    //     rm::modules::Map(globals->rc->left_y(), -globals->rc_max_value_,  // pitch轴目标值
    //                      globals->rc_max_value_, -gimbal->sensitivity_, gimbal->sensitivity_);
    gimbal->gimbal_yaw_target_ = globals->ahrs.euler_angle().yaw;
    gimbal->gimbal_pitch_target_ = globals->ahrs.euler_angle().pitch;
  }
  gimbal->gimbal_yaw_target_ =
      rm::modules::Wrap(gimbal->gimbal_yaw_target_, -static_cast<f32>(M_PI), M_PI);  // yaw轴周期限位
  gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_,    // pitch轴限位
                                                    gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);
}

float sign(float input) {
  if (input == 0) return 0;
  return input / abs(input);
}

/********************
//float p_target = 0;
//float p_real = 0;
// float p_error = 0;
// float p_Kp=0;
// float p_Ki=0;
************************/

/************************
// float J=0.027;
// float B=0.2703;
// float C=1;
// float K=5;
// float epson=10;
// float target=0;
// float d_target=0;
// float dd_target = 0;
// float last_target = 0;
// float last_d_target=0;
// float real=0;
// float d_real=0;
// float last_real=0;
// float Ts=0.002;
// float s=0;
// float sat=0;
// float k1=10;
// float k2=10;
// float i_sign_s=0;
*************************/

/*************************
float J=0.027;
float B=0.2703;
float alpha=1;
float beta=1;
float k1=1;
float k2=1;
float target=0;
float last_target=0;
float d_target=0;
float last_d_target=0;
float dd_target=0;
float real=0;
float d_real=0;
float last_real=0;
float err=0;
float dot_e=0;
float last_e=0;
float w_real=0;
float s=0;
float Ts=0.002;
float i_sign_s=0;
float torque=0;
float q=1;
float p=3;
**************************/

float motor_yaw_angle_ = 0;
float motor_pitch_angle_ = 0;
float motor_target_yaw_ = 0;
float motor_target_pitch_ = 0;
float base_yaw_angle_ = 0;
float base_pitch_angle_ = 0;
float imu_yaw_ = 0;
float imu_pitch_ = 0;
float imu_roll_ = 0;
float yaw_motor_zeropoint_ = 0;
float pitch_motor_zeropoint_ = 0;
Eigen::Matrix<float, 3, 3> G_in_W_;
Eigen::Matrix<float, 3, 3> G_in_B_;
Eigen::Matrix<float, 3, 3> B_in_W_;
Eigen::Matrix<float, 3, 1> T_in_W_;
Eigen::Matrix<float, 3, 1> T_in_B_;
float x;
float y;
float z;
float target_yaw = 0;
float target_pitch = 0;

void Gimbal::GimbalMovePIDUpdate() {
  /******************************************************************************
    //变结构pid
    //  p_error = modules::Wrap(p_real - p_target, -M_PI, M_PI);  // 处理过零
    //
    //  p_Kp=18 + 2 * (1 - 1 / exp(80*fabs(p_error)) );
    //  p_Ki=0.03 / exp(fabs(80*p_error));
    //
    //  globals->gimbal_controller.pid().yaw_position.SetKp(p_Kp);
    //  globals->gimbal_controller.pid().yaw_position.SetKi(p_Ki);
    //
    //  w_target=globals->gimbal_controller.pid().yaw_position.out();
    // w_real=globals->yaw_motor->vel();
    // w_real=globals->imu->gyro_z();
    //
    //  w_error=w_real-w_target;
    //
    //  w_Ki=0.03 / exp(60*fabs(w_error));
    //
    //  globals->gimbal_controller.pid().yaw_speed.SetKi(w_Ki);
  ***************************************************************************************/

  /***********************************************************************************************
    // //滑模控制
    // target=gimbal_yaw_target_;
    //
    // d_target=modules::Wrap(target - last_target, -M_PI, M_PI)/Ts;
    // dd_target=(d_target-last_d_target)/Ts;
    // last_target=target;
    // last_d_target=d_target;
    //
    // if (d_target>1600)d_target=1600;
    // if (d_target<-1600)d_target=-1600;
    // if (dd_target>70000)dd_target=70000;
    // if (dd_target<-70000)dd_target=-70000;
    //
    // real=globals->ahrs.euler_angle().yaw;
    // d_real=modules::Wrap(real-last_real, -M_PI, M_PI)/Ts;
    // last_real=real;
    //
    // s=C*(target-real)+(d_target-d_real);
    //
    // i_sign_s+=sign(s)*Ts;
    // i_sign_s=modules::Wrap(i_sign_s,-0.5,0.5);
    //
    // if (s>=2) {
    //   sat=1;
    // }else if (s<=-2) {
    //   s=-1;
    // }else if (s>-2&&s<2) {
    //   sat=s/2;
    // }
    //
    // globals->gimbal_controller.output().yaw = modules::Wrap(J * (C * (d_target - d_real) + dd_target + epson * sat +
  K * s) + B * d_real,-6,6);
    //globals->gimbal_controller.output().yaw = modules::Wrap(J * (C * (d_target - d_real) + dd_target + k1 *
  pow(abs(s),0.5)*sign(s)+k2*i_sign_s) + B * d_real,-6,6);
  ***********************************************************************************************************************/

  /**************************************************************************************************
    //if (globals->StateMachine_==kGbRemote) {
      target=gimbal_yaw_target_;
      d_target=modules::Wrap(target - last_target, -M_PI, M_PI)/Ts;
      dd_target=(d_target-last_d_target)/Ts;
      last_target=target;
      last_d_target=d_target;

      if (d_target>1600)d_target=1600;
      if (d_target<-1600)d_target=-1600;
      if (dd_target>70000)dd_target=70000;
      if (dd_target<-70000)dd_target=-70000;

      real=globals->ahrs.euler_angle().yaw;
      d_real=modules::Wrap(real-last_real, -M_PI, M_PI)/Ts;
      last_real=real;

      err=target-real;
      dot_e=(err-last_e)/Ts;
      last_e=err;

      s=dot_e+alpha*err+sign(err)*beta*pow(abs(err),q/p);

      i_sign_s+=sign(s)*Ts;
      i_sign_s=modules::Wrap(i_sign_s,-1,1);

      if (err!=0) {
        torque=B*d_real+J*(dd_target-k1*pow(abs(s),0.5)*sign(s)-k2*i_sign_s+alpha*dot_e+beta*q/p*sign(err)*pow(abs(err),q/p-1))*dot_e;
      }else {
        torque=0;
      }

      globals->gimbal_controller.output().yaw = modules::Wrap(torque,-6,6);

    // }else if (globals->StateMachine_==kNoForce) {
    //   target=gimbal_yaw_target_;
    //   last_target=gimbal_yaw_target_;
    //   d_target=0;
    //   last_d_target=gimbal_yaw_target_;
    //   dd_target=0;
    //   real=globals->ahrs.euler_angle().yaw;
    //   d_real=0;
    //   last_real=globals->ahrs.euler_angle().yaw;
    //   err=0;
    //   dot_e=0;
    //   last_e=0;
    //   s=0;
    //   Ts=0.002;
    //   i_sign_s=0;
    //   torque=0;
    // }
    //globals->gimbal_controller.output().yaw = 0;
    **************************************************************************************************/

  /*********************************************************************************************************
   *前馈，无roll轴补偿
    globals->yaw_speed_feedforward->Update(gimbal_yaw_target_);
    globals->gimbal_controller.SetTarget(gimbal->gimbal_yaw_target_, gimbal->gimbal_pitch_target_
                                         ,globals->yaw_speed_feedforward->GetYawSpeedFeedforward());

    globals->gimbal_controller.Update(globals->ahrs.euler_angle().yaw, globals->yaw_motor->vel(),
                                      globals->ahrs.euler_angle().pitch, globals->pitch_motor->vel());

    //globals->gimbal_controller.SetTarget(gimbal->gimbal_yaw_target_, gimbal->gimbal_pitch_target_);
      // globals->gimbal_controller.Update(globals->hipnuc_imu->yaw(), globals->yaw_motor->vel(),
      // globals->hipnuc_imu->roll(),
      //                                   globals->pitch_motor->vel());
      // gimbal->gravity_compensation_ = gimbal->k_gravity_compensation_ * std::cos(globals->pitch_motor->pos());
      // gimbal->pitch_torque_ = globals->gimbal_controller.output().pitch + gimbal->gravity_compensation_;
      // gimbal->pitch_torque_ = rm::modules::Clamp(pitch_torque_, -10.0f, 10.0f);
  ********************************************************************************************************/

  /***************************************************************************************************************/
  // 前馈，roll轴补偿
  globals->gimbal_solver_with_roll->Update(globals->ahrs.euler_angle().yaw, globals->ahrs.euler_angle().pitch,
                                           globals->ahrs.euler_angle().roll, globals->yaw_motor->pos(),
                                           globals->pitch_motor->pos(), gimbal_yaw_target_, gimbal_pitch_target_);

  globals->yaw_speed_feedforward->Update(globals->gimbal_solver_with_roll->Get_Motor_Target_Yaw());
  globals->gimbal_controller.SetTarget(globals->gimbal_solver_with_roll->Get_Motor_Target_Yaw(),
                                       globals->gimbal_solver_with_roll->Get_Motor_Target_Pitch()  //);
                                       ,
                                       globals->yaw_speed_feedforward->GetYawSpeedFeedforward());

  globals->gimbal_controller.Update(globals->gimbal_solver_with_roll->Get_Base_Yaw_Angle(), globals->yaw_motor->vel(),
                                    globals->gimbal_solver_with_roll->Get_Base_Pitch_Angle(),
                                    globals->pitch_motor->vel());

  target_pitch = gimbal_pitch_target_;
  target_yaw = gimbal_yaw_target_;
  /************************************************************************************************************/
}

void Gimbal::GimbalEnableUpdate() {
  gimbal->DaMiaoMotorEnable();
  globals->gimbal_controller.Enable(true);
  if (gimbal->GimbalMove_ == kGbRemote) {
    globals->GimbalData.aim_mode = 0x00;
    globals->aim_mode = 0x00;
    gimbal->GimbalRCTargetUpdate();
    gimbal->GimbalMovePIDUpdate();
  } else if (gimbal->GimbalMove_ == kGbAimbot) {
    globals->GimbalData.aim_mode = 0x01;
    globals->aim_mode = 0x01;
    gimbal->GimbalAimbotTargetUpdate();
    gimbal->GimbalMovePIDUpdate();
  } else {
    globals->GimbalData.aim_mode = 0x00;
    globals->aim_mode = 0x00;
    globals->gimbal_controller.Enable(false);
  }
}

void Gimbal::GimbalDisableUpdate() {
  gimbal->DaMiaoMotorDisable();
  globals->gimbal_controller.Enable(false);
  globals->GimbalData.aim_mode = 0x00;
  gimbal->gimbal_yaw_target_ = globals->ahrs.euler_angle().yaw;
  gimbal->gimbal_pitch_target_ = globals->ahrs.euler_angle().pitch;
  // gimbal->gimbal_yaw_target_ = globals->hipnuc_imu->yaw();
  // gimbal->gimbal_pitch_target_ = globals->hipnuc_imu->roll();
  gimbal->GimbalMovePIDUpdate();
}

void Gimbal::DaMiaoMotorEnable() {
  if (gimbal->DM_enable_flag_ == false) {
    // 使达妙电机使能
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    globals->yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    gimbal->DM_enable_flag_ = true;
  }
}

void Gimbal::DaMiaoMotorDisable() {
  if (gimbal->DM_enable_flag_ == true) {
    // 使达妙电机失能
    globals->yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    gimbal->DM_enable_flag_ = false;
  }
}