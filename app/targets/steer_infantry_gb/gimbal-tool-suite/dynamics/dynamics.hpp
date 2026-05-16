#pragma once

#include <Eigen/Dense>
#include <cmath>

/**
 * @brief 前馈力矩分解输出，用于调试和分步验证
 */
struct GimbalFfDecomposed {
  float yaw = 0.0f;
  float pitch = 0.0f;
  float yaw_inertia = 0.0f;
  float yaw_gravity = 0.0f;
  float yaw_friction = 0.0f;
  float pitch_yaw_coupling = 0.0f;
  float pitch_inertia = 0.0f;
  float pitch_gravity = 0.0f;
  float pitch_friction = 0.0f;
};

/**
 * @brief 二轴云台前馈动力学模型
 *
 * 坐标系：前+X，左+Y，上+Z
 * q1=0 q2=0 时云台平视前方
 * q1 正向为从上向下看逆时针旋转
 * q2 正向为向上旋转
 *
 * 支持移动基座的 3D 加速度/重力场补偿：
 *   - 静止基座传 [0, 0, -9.81]
 *   - 有 IMU 加速度计时可传实测值补偿平动加速度
 */
class Gimbal2DofDynamics {
 public:
  /**
   * @brief 设置 9 个基参数
   * @param theta [theta_0..theta_8] — 由 ident.ipynb 辨识得到
   */
  void SetTheta(const Eigen::Matrix<float, 9, 1>& theta) { theta_ = theta; }

  const Eigen::Matrix<float, 9, 1>& GetTheta() const { return theta_; }

  /**
   * @brief 计算前馈力矩 tau = Y * theta
   * @return Eigen::Vector2f [tau_yaw, tau_pitch]^T
   */
  Eigen::Vector2f ComputeFf(float q1, float q2, float dq1, float dq2, float ddq1, float ddq2,
                            const Eigen::Vector3f& g) const {
    return ComputeY(q1, q2, dq1, dq2, ddq1, ddq2, g) * theta_;
  }

  /**
   * @brief 计算带分解的前馈力矩，便于分步验证（重力→惯性→耦合→摩擦）
   */
  GimbalFfDecomposed ComputeFfDecomposed(float q1, float q2, float dq1, float dq2, float ddq1, float ddq2,
                                         const Eigen::Vector3f& g) const {
    const auto Y = ComputeY(q1, q2, dq1, dq2, ddq1, ddq2, g);

    GimbalFfDecomposed out;

    out.yaw_inertia = Y(0, 0) * theta_(0) + Y(0, 1) * theta_(1);
    out.yaw_gravity = Y(0, 3) * theta_(3) + Y(0, 4) * theta_(4);
    out.yaw_friction = Y(0, 5) * theta_(5) + Y(0, 6) * theta_(6);
    out.yaw = out.yaw_inertia + out.yaw_gravity + out.yaw_friction;

    out.pitch_yaw_coupling = Y(1, 0) * theta_(0) + Y(1, 1) * theta_(1);
    out.pitch_inertia = Y(1, 2) * theta_(2);
    out.pitch_gravity = Y(1, 3) * theta_(3) + Y(1, 4) * theta_(4);
    out.pitch_friction = Y(1, 7) * theta_(7) + Y(1, 8) * theta_(8);
    out.pitch = out.pitch_yaw_coupling + out.pitch_inertia + out.pitch_gravity + out.pitch_friction;

    return out;
  }

 private:
  /**
   * @brief 构建回归矩阵 Y (2×9)
   */
  Eigen::Matrix<float, 2, 9> ComputeY(float q1, float q2, float dq1, float dq2, float ddq1, float ddq2,
                                      const Eigen::Vector3f& g) const {
    const float sin_q1 = std::sin(q1);
    const float cos_q1 = std::cos(q1);
    const float sin_q2 = std::sin(q2);
    const float cos_q2 = std::cos(q2);
    const float sin_2q2 = 2.0f * sin_q2 * cos_q2;
    const float sin2_q2 = sin_q2 * sin_q2;
    const float cos2_q2 = cos_q2 * cos_q2;
    const float dq1_dq2 = dq1 * dq2;
    const float dq1_sq = dq1 * dq1;

    const float gx = g.x();
    const float gy = g.y();
    const float gz = g.z();
    const float gx_sin_gy_cos_q1 = gx * sin_q1 - gy * cos_q1;
    const float gx_cos_gy_sin_q1 = gx * cos_q1 + gy * sin_q1;

    Eigen::Matrix<float, 2, 9> Y;
    Y.setZero();

    // Yaw 轴动力学
    Y(0, 0) = sin2_q2 * ddq1 + sin_2q2 * dq1_dq2;   // theta_0: I1zz_com
    Y(0, 1) = cos2_q2 * ddq1 - sin_2q2 * dq1_dq2;   // theta_1: I2xx_com
    Y(0, 3) = gx_sin_gy_cos_q1 * cos_q2;             // theta_3: m2*l2x 水平偏心
    Y(0, 4) = gx_sin_gy_cos_q1 * sin_q2;             // theta_4: m2*l2z 垂直偏心
    Y(0, 5) = dq1;                                   // theta_5: fv1 粘滞摩擦
    Y(0, 6) = std::tanh(dq1);                        // theta_6: fc1 库仑摩擦

    // Pitch 轴动力学
    Y(1, 0) = -0.5f * sin_2q2 * dq1_sq;                                      // theta_0
    Y(1, 1) = 0.5f * sin_2q2 * dq1_sq;                                       // theta_1
    Y(1, 2) = ddq2;                                                            // theta_2: I2yy_com
    Y(1, 3) = gx_cos_gy_sin_q1 * sin_q2 + gz * cos_q2;                       // theta_3
    Y(1, 4) = -gx_cos_gy_sin_q1 * cos_q2 + gz * sin_q2;                      // theta_4
    Y(1, 7) = dq2;                                                             // theta_7: fv2
    Y(1, 8) = std::tanh(dq2);                                                  // theta_8: fc2

    return Y;
  }

  // 9 个基参数，由 ident.ipynb 辨识得到，可通过 SetTheta() 覆盖
  Eigen::Matrix<float, 9, 1> theta_{
      305.078838f,   // theta_0 (I1zz_com)
      316.070913f,   // theta_1 (I2xx_com)
      1.933167f,     // theta_2 (I2yy_com)
      -0.047971f,    // theta_3 (m2*l2x 水平偏心)
      10.138195f,    // theta_4 (m2*l2z 垂直偏心)
      751.739484f,   // theta_5 (fv1) Yaw 粘性摩擦
      1592.974745f,  // theta_6 (fc1) Yaw 库仑摩擦
      42.827081f,    // theta_7 (fv2) Pitch 粘性摩擦
      -3.728933f     // theta_8 (fc2) Pitch 库仑摩擦
  };
};
