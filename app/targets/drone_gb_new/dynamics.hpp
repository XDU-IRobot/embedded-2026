#pragma once

#include <Eigen/Dense>
#include <cmath>

/**
 * @brief 计算云台前馈力矩 tau = Y * theta
 * @note  坐标系定义：前+X，左+Y，上+Z，q1=0 q2=0时云台平视前方，q1正向为从上向下看逆时针旋转，q2正向为向上旋转
 *
 * @param q1   Yaw 角度 (rad)
 * @param q2   Pitch 角度 (rad)
 * @param dq1  Yaw 角速度 (rad/s)
 * @param dq2  Pitch 角速度 (rad/s)
 * @param ddq1 Yaw 角加速度 (rad/s^2)
 * @param ddq2 Pitch 角加速度 (rad/s^2)
 * @param g    重力加速度向量 [gx, gy, gz]^T (m/s^2)，如果云台基座有imu可以测量实际加速度的话，传[gx, gy,
 * -9.81]可以实现平动加速度补偿，没有条件的话传[0, 0, -9.81]即可
 * @return Eigen::Vector2f 返回力矩向量 [tau_yaw, tau_pitch]^T
 */

class Gimbal2DofDynamics {
 public:
  Eigen::Vector2f ComputeFf(float q1, float q2, float dq1, float dq2, float ddq1, float ddq2,
                            const Eigen::Vector3f& g) const {
    // 预计算三角函数，一般在F4/H7上已经足够快，如果有CMSIS-DSP可以换成arm_sin_f32/arm_cos_f32
    const float sin_q1 = std::sin(q1);
    const float cos_q1 = std::cos(q1);
    const float sin_q2 = std::sin(q2);
    const float cos_q2 = std::cos(q2);
    const float sin_2q2 = 2.0f * sin_q2 * cos_q2;

    const float sin2_q2 = sin_q2 * sin_q2;
    const float cos2_q2 = cos_q2 * cos_q2;

    const float dq1_dq2 = dq1 * dq2;
    const float dq1_sq = dq1 * dq1;

    // 预计算重力场/加速度相关项
    const float gx = g.x();
    const float gy = g.y();
    const float gz = g.z();
    const float gx_sin_gy_cos_q1 = gx * sin_q1 - gy * cos_q1;
    const float gx_cos_gy_sin_q1 = gx * cos_q1 + gy * sin_q1;

    // 构建回归矩阵 Y (2x9)
    Eigen::Matrix<float, 2, 9> Y;
    Y.setZero();

    // Yaw 轴动力学
    Y(0, 0) = sin2_q2 * ddq1 + sin_2q2 * dq1_dq2;
    Y(0, 1) = cos2_q2 * ddq1 - sin_2q2 * dq1_dq2;
    Y(0, 3) = gx_sin_gy_cos_q1 * cos_q2;  // Yaw 水平偏心重力补偿
    Y(0, 4) = gx_sin_gy_cos_q1 * sin_q2;  // Yaw 垂直偏心重力补偿
    Y(0, 5) = dq1;                        // 粘滞摩擦
    Y(0, 6) = std::tanh(dq1);             // 库仑摩擦

    // Pitch 轴动力学
    Y(1, 0) = -0.5f * sin_2q2 * dq1_sq;
    Y(1, 1) = 0.5f * sin_2q2 * dq1_sq;
    Y(1, 2) = ddq2;
    Y(1, 3) = gx_cos_gy_sin_q1 * sin_q2 + gz * cos_q2;   // Pitch 水平偏心重力补偿
    Y(1, 4) = -gx_cos_gy_sin_q1 * cos_q2 + gz * sin_q2;  // Pitch 垂直偏心重力补偿
    Y(1, 7) = dq2;                                       // 粘滞摩擦
    Y(1, 8) = std::tanh(dq2);                            // 库仑摩擦

    // 计算前馈力矩
    // 如果惯量引起的震荡较大，可以在实机调试时将 theta_1, theta_2 清零
    const Eigen::Vector2f tau = Y * THETA;

    return tau;
  }

 private:
  // 使用本次辨识出的 9 个基参数
  const Eigen::Matrix<float, 9, 1> THETA{
      0.020574f,   // theta_1 (I1zz_com) 可根据实机效果清零
      0.022198f,   // theta_2 (I2xx_com) 可根据实机效果清零
      0.043097f,   // theta_3 (I2yy_com)
      -0.043417f,  // theta_4 (m2*l2x 水平偏心)
      -0.108130f,  // theta_5 (m2*l2z 垂直偏心)
      -0.004880f,  // theta_6 (fv1) - Yaw 粘性摩擦
      0.092282f,   // theta_7 (fc1) - Yaw 库仑摩擦
      0.169119f,   // theta_8 (fv2) - Pitch 粘性摩擦
      0.070344f    // theta_9 (fc2) - Pitch 库仑摩擦
  };
};