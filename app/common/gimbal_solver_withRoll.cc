/**
* @file   gimbal_solver_withRoll.hpp
* @brief  本文件通过坐标变换将目标从世界系变换到基座系
* @brief  可以参考https://zhuanlan.zhihu.com/p/1988696935104811632
**/

#include "gimbal_solver_withRoll.hpp"

/**
 * @brief          构造函数
 * @param[in]      yaw_motor_zeropoint    最好将云台朝前读出yaw电机角度，注意正负号，可以调试看base_yaw_angle_是否正确
 * @param[in]      pitch_motor_zeropoint      同上
 * @returns        无
 */
Gimbal_Solver_WithRoll::Gimbal_Solver_WithRoll(float yaw_motor_zeropoint, float pitch_motor_zeropoint) {
  yaw_motor_zeropoint_=yaw_motor_zeropoint;
  pitch_motor_zeropoint_=pitch_motor_zeropoint;
}

/**
 * @brief          Update
 * @param[in]      imu_yaw    云台imu得到的yaw角度
 * @param[in]      imu_pitch  同上
 * @param[in]      imu_roll   同上
 * @param[in]      motor_yaw_angle  此时yaw电机的角度，注意正负号（电机角度增加或减少方向应与imu一致）
 * @param[in]      motor_pitch_angle  此时yaw电机的角度，注意正负号（电机角度增加或减少方向应与imu一致）
 * @param[in]      target_yaw  世界系下的目标yaw角度
 * @param[in]      target_pitch 同上
 * @returns        无
 */
void Gimbal_Solver_WithRoll::Update(float imu_yaw, float imu_pitch, float imu_roll, float motor_yaw_angle,
                                    float motor_pitch_angle, float target_yaw, float target_pitch) {
  motor_yaw_angle_ = motor_yaw_angle;
  motor_pitch_angle_ = motor_pitch_angle;

  imu_target_yaw_ = target_yaw;
  imu_target_pitch_ = target_pitch;

  imu_yaw_ = imu_yaw;
  imu_pitch_ = imu_pitch;
  imu_roll_ = imu_roll;

  base_yaw_angle_ = motor_yaw_angle_ - yaw_motor_zeropoint_ ;
  base_pitch_angle_ = motor_pitch_angle_ - pitch_motor_zeropoint_ ;

  if (base_yaw_angle_>M_PI) {
      base_yaw_angle_ = base_yaw_angle_ - 2*M_PI;
  }

  if (base_yaw_angle_<-M_PI) {
      base_yaw_angle_ = base_yaw_angle_ + 2*M_PI;
  }

  if (base_pitch_angle_>M_PI) {
      base_pitch_angle_ = base_pitch_angle_ - 2*M_PI;
  }

  if (base_pitch_angle_<-M_PI) {
      base_pitch_angle_ = base_pitch_angle_ + 2*M_PI;
  }



  G_in_W_ <<  cos(imu_yaw_)*cos(imu_pitch_),cos(imu_yaw_)*sin(imu_pitch_)*sin(imu_roll_)-sin(imu_yaw_)*cos(imu_roll_),cos(imu_yaw_)*sin(imu_pitch_)*cos(imu_roll_)+sin(imu_yaw_)*sin(imu_roll_),
              sin(imu_yaw_)*cos(imu_pitch_),sin(imu_yaw_)*sin(imu_pitch_)*sin(imu_roll_)+cos(imu_yaw_)*cos(imu_roll_),sin(imu_yaw_)*sin(imu_pitch_)*cos(imu_roll_)-cos(imu_yaw_)*sin(imu_roll_),
              -sin(imu_pitch_)             ,                 cos(imu_pitch_)*sin(imu_roll_)                          ,                       cos(imu_pitch_)*cos(imu_roll_)                    ;

  T_in_W_ <<  cos(imu_target_pitch_)*cos(imu_target_yaw_),
              cos(imu_target_pitch_)*sin(imu_target_yaw_),
                          -sin(imu_target_pitch_)        ;

  G_in_B_ <<  cos(base_yaw_angle_)*cos(base_pitch_angle_),-sin(base_yaw_angle_),cos(base_yaw_angle_)*sin(base_pitch_angle_),
              sin(base_yaw_angle_)*cos(base_pitch_angle_), cos(base_yaw_angle_),sin(base_yaw_angle_)*sin(base_pitch_angle_),
              -sin(base_pitch_angle_)                     ,          0           ,             cos(base_pitch_angle_)      ;

  B_in_W_ = G_in_W_ * G_in_B_.transpose();

  T_in_B_ = B_in_W_.transpose() * T_in_W_;

  x = T_in_B_(0,0); y = T_in_B_(1,0); z = T_in_B_(2,0);

  motor_target_yaw_ = atan2(y,x);
  motor_target_pitch_ = atan2(-z,sqrt(x * x + y * y));
}

/**
 * @brief          Get_Motor_Target_Pitch
 * @param[in]      无
 * @returns        基座系下的目标角度，后续pid将使用该角度作为set，电机反馈值作为ref
 */
float Gimbal_Solver_WithRoll::Get_Motor_Target_Pitch() {
  return motor_target_pitch_;
}

/**
 * @brief          Get_Motor_Target_Yaw
 * @param[in]      无
 * @returns        基座系下的目标角度，后续pid将使用该角度作为set，电机反馈值作为ref
 */
float Gimbal_Solver_WithRoll::Get_Motor_Target_Yaw() {
  return motor_target_yaw_;
}

/**
 * @brief          Get_Base_Yaw_Angle
 * @param[in]      无
 * @returns        可调试确认零点和方向是否正确
 */
float Gimbal_Solver_WithRoll::Get_Base_Yaw_Angle() {
  return base_yaw_angle_;
}

/**
 * @brief          Get_Base_Pitch_Angle
 * @param[in]      无
 * @returns        可调试确认零点和方向是否正确
 */
float Gimbal_Solver_WithRoll::Get_Base_Pitch_Angle() {
  return base_pitch_angle_;
}

