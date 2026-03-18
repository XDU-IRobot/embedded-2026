#include <librm.hpp>

#ifndef BOARDC_GIMBAL_SOLVER_WITHROLL_HPP
#define BOARDC_GIMBAL_SOLVER_WITHROLL_HPP

/**
 * @brief 坐标变换求解器的基类
 **/

class Gimbal_Solver_WithRoll {
 public:
  Gimbal_Solver_WithRoll() = default;
  Gimbal_Solver_WithRoll(float yaw_motor_zeropoint, float pitch_motor_zeropoint);

  void Update(float imu_yaw, float imu_pitch, float imu_roll, float motor_yaw_angle, float motor_pitch_angle,
              float target_yaw, float target_pitch);

  float Get_Motor_Target_Yaw();
  float Get_Motor_Target_Pitch();
  float Get_Base_Yaw_Angle();
  float Get_Base_Pitch_Angle();

 private:
  float motor_yaw_angle_ = 0;     //<yaw轴电机的角度
  float motor_pitch_angle_ = 0;   //<pitch轴电机角度
  float motor_target_yaw_ = 0;    //<基座系下的目标角度，即yaw轴电机目标角度
  float motor_target_pitch_ = 0;  //<pitch轴电机目标角度
  float base_yaw_angle_ = 0;      //<基座的yaw轴角度
  float base_pitch_angle_ = 0;    //<基座的pitch轴角度
  float imu_yaw_ = 0;
  float imu_pitch_ = 0;
  float imu_roll_ = 0;
  float yaw_motor_zeropoint_ = 0;
  float pitch_motor_zeropoint_ = 0;
  float imu_target_yaw_ = 0;  //<世界系下的目标角度，即用imu控制时的目标角度
  float imu_target_pitch_ = 0;
  // 旋转矩阵
  Eigen::Matrix<float, 3, 3> G_in_W_;
  Eigen::Matrix<float, 3, 3> G_in_B_;
  Eigen::Matrix<float, 3, 3> B_in_W_;
  Eigen::Matrix<float, 3, 1> T_in_W_;
  Eigen::Matrix<float, 3, 1> T_in_B_;
  float x = 0;
  float y = 0;
  float z = 0;
};

#endif  // BOARDC_GIMBAL_SOLVER_WITHROLL_HPP
