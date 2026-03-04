/*
  Copyright (c) 2026 XDU-IRobot

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "librm.hpp"

using namespace rm;
using namespace rm::modules;

// 浮点数比较容差
constexpr float kFloatEpsilon = 1e-5f;

// ============================================================================
// 构造函数与初始化测试
// ============================================================================

TEST(PIDTest, DefaultConstructor) {
  PID pid;
  // 默认构造应该初始化所有参数为0
  EXPECT_FLOAT_EQ(pid.kp(), 0.0f);
  EXPECT_FLOAT_EQ(pid.ki(), 0.0f);
  EXPECT_FLOAT_EQ(pid.kd(), 0.0f);
  EXPECT_FLOAT_EQ(pid.max_out(), 0.0f);
  EXPECT_FLOAT_EQ(pid.max_iout(), 0.0f);
  EXPECT_FLOAT_EQ(pid.out(), 0.0f);
}

TEST(PIDTest, ParameterizedConstructor) {
  PID pid(1.0f, 2.0f, 3.0f, 100.0f, 50.0f);
  EXPECT_FLOAT_EQ(pid.kp(), 1.0f);
  EXPECT_FLOAT_EQ(pid.ki(), 2.0f);
  EXPECT_FLOAT_EQ(pid.kd(), 3.0f);
  EXPECT_FLOAT_EQ(pid.max_out(), 100.0f);
  EXPECT_FLOAT_EQ(pid.max_iout(), 50.0f);
}

TEST(PIDTest, ConstructorWithZeroParams) {
  PID pid(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  pid.Update(100.0f, 0.0f, 0.01f);
  // 所有增益为0，输出应为0
  EXPECT_FLOAT_EQ(pid.out(), 0.0f);
}

TEST(PIDTest, ConstructorWithNegativeParams) {
  // 负增益应该正常工作（反向控制器）
  PID pid(-1.0f, 0.0f, 0.0f, 100.0f, 50.0f);
  pid.Update(10.0f, 0.0f, 0.01f);
  // 正误差 * 负增益 = 负输出
  EXPECT_LT(pid.out(), 0.0f);
}

// ============================================================================
// Setters 链式调用测试
// ============================================================================

TEST(PIDTest, SettersChaining) {
  PID pid;
  pid.SetKp(1.5f)
      .SetKi(0.5f)
      .SetKd(0.1f)
      .SetMaxOut(200.0f)
      .SetMaxIout(100.0f)
      .SetDiffLpfAlpha(0.5f)
      .SetDiffFirst(true)
      .SetDynamicKi(true)
      .SetCircular(true)
      .SetCircularCycle(360.0f)
      .SetFuzzy(false);

  EXPECT_FLOAT_EQ(pid.kp(), 1.5f);
  EXPECT_FLOAT_EQ(pid.ki(), 0.5f);
  EXPECT_FLOAT_EQ(pid.kd(), 0.1f);
  EXPECT_FLOAT_EQ(pid.max_out(), 200.0f);
  EXPECT_FLOAT_EQ(pid.max_iout(), 100.0f);
  EXPECT_FLOAT_EQ(pid.diff_lpf_alpha(), 0.5f);
  EXPECT_TRUE(pid.enable_diff_first());
  EXPECT_TRUE(pid.enable_dynamic_ki());
  EXPECT_TRUE(pid.enable_circular());
  EXPECT_FLOAT_EQ(pid.circular_cycle(), 360.0f);
  EXPECT_FALSE(pid.enable_fuzzy());
}

// ============================================================================
// 基本P控制器测试
// ============================================================================

TEST(PIDTest, ProportionalOnlyPositiveError) {
  PID pid(2.0f, 0.0f, 0.0f, 100.0f, 0.0f);
  pid.Update(10.0f, 0.0f, 0.01f);
  // P输出 = kp * error = 2.0 * 10.0 = 20.0
  EXPECT_FLOAT_EQ(pid.p_out(), 20.0f);
  EXPECT_FLOAT_EQ(pid.out(), 20.0f);
  EXPECT_FLOAT_EQ(pid.error()[0], 10.0f);
}

TEST(PIDTest, ProportionalOnlyNegativeError) {
  PID pid(2.0f, 0.0f, 0.0f, 100.0f, 0.0f);
  pid.Update(0.0f, 10.0f, 0.01f);
  // P输出 = kp * error = 2.0 * (-10.0) = -20.0
  EXPECT_FLOAT_EQ(pid.p_out(), -20.0f);
  EXPECT_FLOAT_EQ(pid.out(), -20.0f);
}

TEST(PIDTest, ProportionalOnlyZeroError) {
  PID pid(2.0f, 0.0f, 0.0f, 100.0f, 0.0f);
  pid.Update(5.0f, 5.0f, 0.01f);
  EXPECT_FLOAT_EQ(pid.error()[0], 0.0f);
  EXPECT_FLOAT_EQ(pid.out(), 0.0f);
}

// ============================================================================
// 基本I控制器测试
// ============================================================================

TEST(PIDTest, IntegralOnlyAccumulation) {
  PID pid(0.0f, 1.0f, 0.0f, 100.0f, 100.0f);

  // 第一次更新
  pid.Update(10.0f, 0.0f, 0.01f);
  // 梯形积分: (e[0] + e[1]) / 2 * dt = (10 + 0) / 2 * 0.01 = 0.05
  // i_out = ki * trapezoid = 1.0 * 0.05 = 0.05
  EXPECT_NEAR(pid.i_out(), 0.05f, kFloatEpsilon);

  // 第二次更新
  pid.Update(10.0f, 0.0f, 0.01f);
  // 梯形积分: (10 + 10) / 2 * 0.01 = 0.1
  // i_out 累加 = 0.05 + 0.1 = 0.15
  EXPECT_NEAR(pid.i_out(), 0.15f, kFloatEpsilon);
}

TEST(PIDTest, IntegralOnlySaturation) {
  PID pid(0.0f, 100.0f, 0.0f, 1000.0f, 10.0f);  // 较小的max_iout

  // 持续输入大误差，积分项应被限制
  for (int i = 0; i < 100; ++i) {
    pid.Update(100.0f, 0.0f, 0.01f);
  }
  // i_out应该被限制在±max_iout范围内
  EXPECT_LE(pid.i_out(), 10.0f);
  EXPECT_GE(pid.i_out(), -10.0f);
}

TEST(PIDTest, IntegralNegativeSaturation) {
  PID pid(0.0f, 100.0f, 0.0f, 1000.0f, 10.0f);

  for (int i = 0; i < 100; ++i) {
    pid.Update(-100.0f, 0.0f, 0.01f);
  }
  EXPECT_FLOAT_EQ(pid.i_out(), -10.0f);
}

TEST(PIDTest, TrapezoidalIntegration) {
  PID pid(0.0f, 1.0f, 0.0f, 100.0f, 100.0f);

  pid.Update(10.0f, 0.0f, 0.01f);  // error = 10
  pid.Update(20.0f, 0.0f, 0.01f);  // error = 20

  // 梯形积分: (10 + 20) / 2 * 0.01 = 0.15
  EXPECT_NEAR(pid.trapezoid(), 0.15f, kFloatEpsilon);
}

// ============================================================================
// 基本D控制器测试
// ============================================================================

TEST(PIDTest, DerivativeOnlyIncreasingError) {
  PID pid(0.0f, 0.0f, 1.0f, 100.0f, 0.0f);

  pid.Update(0.0f, 0.0f, 0.01f);   // error = 0
  pid.Update(10.0f, 0.0f, 0.01f);  // error = 10

  // D输出 = kd * (e[0] - e[1]) / dt = 1.0 * (10 - 0) / 0.01 = 1000
  // 但会被max_out限制
  EXPECT_FLOAT_EQ(pid.out(), 100.0f);  // 被限幅
}

TEST(PIDTest, DerivativeOnlyDecreasingError) {
  PID pid(0.0f, 0.0f, 1.0f, 100.0f, 0.0f);

  pid.Update(10.0f, 0.0f, 0.01f);  // error = 10
  pid.Update(0.0f, 0.0f, 0.01f);   // error = 0

  // D输出 = kd * (0 - 10) / 0.01 = -1000, 限幅后 -100
  EXPECT_FLOAT_EQ(pid.out(), -100.0f);
}

TEST(PIDTest, DerivativeOnlyConstantError) {
  PID pid(0.0f, 0.0f, 1.0f, 100.0f, 0.0f);

  pid.Update(10.0f, 0.0f, 0.01f);
  pid.Update(10.0f, 0.0f, 0.01f);

  // 误差不变，D输出应为0
  EXPECT_FLOAT_EQ(pid.d_out()[0], 0.0f);
}

// ============================================================================
// 输出限幅测试
// ============================================================================

TEST(PIDTest, OutputSaturationPositive) {
  PID pid(10.0f, 0.0f, 0.0f, 50.0f, 0.0f);
  pid.Update(100.0f, 0.0f, 0.01f);
  // P输出 = 1000，但应被限制在50
  EXPECT_FLOAT_EQ(pid.out(), 50.0f);
}

TEST(PIDTest, OutputSaturationNegative) {
  PID pid(10.0f, 0.0f, 0.0f, 50.0f, 0.0f);
  pid.Update(-100.0f, 0.0f, 0.01f);
  EXPECT_FLOAT_EQ(pid.out(), -50.0f);
}

TEST(PIDTest, OutputWithinLimits) {
  PID pid(1.0f, 0.0f, 0.0f, 100.0f, 0.0f);
  pid.Update(10.0f, 0.0f, 0.01f);
  // 输出10，在限幅范围内
  EXPECT_FLOAT_EQ(pid.out(), 10.0f);
}

// ============================================================================
// 完整PID控制器测试
// ============================================================================

TEST(PIDTest, FullPIDResponse) {
  PID pid(1.0f, 0.1f, 0.01f, 100.0f, 50.0f);

  pid.Update(10.0f, 0.0f, 0.01f);

  // 验证各项都在工作
  EXPECT_NE(pid.p_out(), 0.0f);
  EXPECT_NE(pid.i_out(), 0.0f);
  // 第一次更新d_out可能为0（如果没有上次误差）
}

TEST(PIDTest, ConvergenceToTarget) {
  PID pid(0.5f, 0.1f, 0.05f, 100.0f, 50.0f);

  float ref = 0.0f;
  const float set = 10.0f;
  const float dt = 0.01f;

  // 模拟简单的一阶系统响应
  for (int i = 0; i < 1000; ++i) {
    pid.Update(set, ref, dt);
    ref += pid.out() * dt;  // 简单积分模拟系统响应
  }

  // 应该接近目标值（积分项会导致一定的超调）
  EXPECT_NEAR(ref, set, 2.0f);
}

// ============================================================================
// 微分先行测试
// ============================================================================

TEST(PIDTest, DerivativeFirstEnabled) {
  PID pid(0.0f, 0.0f, 1.0f, 100.0f, 0.0f);
  pid.SetDiffFirst(true);

  // 设定值突变，参考值不变
  pid.Update(0.0f, 5.0f, 0.01f);
  pid.Update(100.0f, 5.0f, 0.01f);  // 设定值突变

  // 微分先行：只对ref微分，ref不变所以d_out应该为0
  EXPECT_FLOAT_EQ(pid.d_out()[0], 0.0f);
}

TEST(PIDTest, DerivativeFirstWithRefChange) {
  PID pid(0.0f, 0.0f, 1.0f, 100.0f, 0.0f);
  pid.SetDiffFirst(true);

  pid.Update(0.0f, 0.0f, 0.01f);
  pid.Update(0.0f, 10.0f, 0.01f);  // ref变化

  // 微分先行：d_out = kd * (ref[0] - ref[1]) / dt = 1 * (10 - 0) / 0.01 = 1000
  // 被限幅到100
  EXPECT_FLOAT_EQ(pid.out(), 100.0f);
}

TEST(PIDTest, DerivativeFirstDisabled) {
  PID pid(0.0f, 0.0f, 1.0f, 10000.0f, 0.0f);
  pid.SetDiffFirst(false);

  pid.Update(0.0f, 5.0f, 0.01f);    // error = -5
  pid.Update(100.0f, 5.0f, 0.01f);  // error = 95

  // 不使用微分先行：d_out = kd * (e[0] - e[1]) / dt = 1 * (95 - (-5)) / 0.01 = 10000
  EXPECT_FLOAT_EQ(pid.out(), 10000.0f);
}

// ============================================================================
// 微分项低通滤波测试
// ============================================================================

TEST(PIDTest, DerivativeLowPassFilterEnabled) {
  PID pid(0.0f, 0.0f, 1.0f, 10000.0f, 0.0f);
  pid.SetDiffLpfAlpha(0.5f);

  pid.Update(0.0f, 0.0f, 0.01f);
  pid.Update(10.0f, 0.0f, 0.01f);

  float first_d_out = pid.d_out()[0];

  pid.Update(10.0f, 0.0f, 0.01f);
  float second_d_out = pid.d_out()[0];

  // 第二次误差不变，但由于滤波，d_out应该是上次值的衰减
  EXPECT_NEAR(second_d_out, (1 - 0.5f) * first_d_out, kFloatEpsilon);
}

TEST(PIDTest, DerivativeLowPassFilterDisabled) {
  PID pid(0.0f, 0.0f, 1.0f, 10000.0f, 0.0f);
  pid.SetDiffLpfAlpha(1.0f);  // alpha=1表示不滤波

  pid.Update(0.0f, 0.0f, 0.01f);
  pid.Update(10.0f, 0.0f, 0.01f);
  pid.Update(10.0f, 0.0f, 0.01f);

  // 不滤波，误差不变时d_out应该为0
  EXPECT_FLOAT_EQ(pid.d_out()[0], 0.0f);
}

// ============================================================================
// 变速积分测试
// ============================================================================

TEST(PIDTest, DynamicKiEnabled) {
  PID pid(0.0f, 10.0f, 0.0f, 100.0f, 100.0f);
  pid.SetDynamicKi(true);

  // 变速积分逻辑:
  // calc_ki = enable_dynamic_ki_ ? dynamic_ki_ : ki_  (第一次时 dynamic_ki_=0)
  // dynamic_ki_ = SafeDiv(ki_, 1 + abs(error))        (更新为正确值)
  // 所以第一次调用 calc_ki=0，第二次调用使用上一次计算的 dynamic_ki_

  // 大误差时，dynamic_ki应该变小
  pid.Update(100.0f, 0.0f, 0.01f);
  // dynamic_ki_ = ki / (1 + 100) ≈ 0.099
  EXPECT_LT(pid.dynamic_ki(), 1.0f);
  EXPECT_GT(pid.dynamic_ki(), 0.0f);

  // 第二次调用，积分项使用上一次计算的 dynamic_ki_
  pid.Update(100.0f, 0.0f, 0.01f);
  EXPECT_NE(pid.i_out(), 0.0f);

  // 小误差时，dynamic_ki接近原始ki
  pid.Clear();
  pid.Update(0.01f, 0.0f, 0.01f);
  // dynamic_ki_ = ki / (1 + 0.01) ≈ 9.9
  EXPECT_GT(pid.dynamic_ki(), 9.0f);
}

TEST(PIDTest, DynamicKiZeroError) {
  PID pid(0.0f, 10.0f, 0.0f, 100.0f, 100.0f);
  pid.SetDynamicKi(true);

  // 变速积分逻辑（修复后）: dynamic_ki_ = SafeDiv(ki_, 1 + abs(error))
  // 误差为0时，dynamic_ki = ki / 1 = ki
  pid.Update(0.0f, 0.0f, 0.01f);
  EXPECT_FLOAT_EQ(pid.dynamic_ki(), 10.0f);

  // 再次更新，仍然为ki
  pid.Update(0.0f, 0.0f, 0.01f);
  EXPECT_FLOAT_EQ(pid.dynamic_ki(), 10.0f);
}

// ============================================================================
// 过零点处理测试
// ============================================================================

TEST(PIDTest, CircularEnabled) {
  PID pid(1.0f, 0.0f, 0.0f, 1000.0f, 0.0f);
  pid.SetCircular(true).SetCircularCycle(360.0f);

  // 从350度到10度，实际误差应该是20度，而不是-340度
  pid.Update(10.0f, 350.0f, 0.01f);

  // 误差应该是正方向的20度
  EXPECT_NEAR(pid.error()[0], 20.0f, kFloatEpsilon);
  EXPECT_NEAR(pid.out(), 20.0f, kFloatEpsilon);
}

TEST(PIDTest, CircularNegativeDirection) {
  PID pid(1.0f, 0.0f, 0.0f, 1000.0f, 0.0f);
  pid.SetCircular(true).SetCircularCycle(360.0f);

  // 从10度到350度，误差应该是-20度
  pid.Update(350.0f, 10.0f, 0.01f);
  EXPECT_NEAR(pid.error()[0], -20.0f, kFloatEpsilon);
}

TEST(PIDTest, CircularDisabled) {
  PID pid(1.0f, 0.0f, 0.0f, 1000.0f, 0.0f);
  pid.SetCircular(false);

  pid.Update(10.0f, 350.0f, 0.01f);
  EXPECT_FLOAT_EQ(pid.error()[0], -340.0f);
}

TEST(PIDTest, CircularWithDifferentCycle) {
  PID pid(1.0f, 0.0f, 0.0f, 1000.0f, 0.0f);
  pid.SetCircular(true).SetCircularCycle(2 * M_PI);  // 弧度制

  // 从 -3.0 到 3.0 弧度
  pid.Update(3.0f, -3.0f, 0.01f);

  // 实际误差应该通过过零点计算
  float expected = 6.0f - 2 * M_PI;  // ≈ -0.283
  EXPECT_NEAR(pid.error()[0], expected, 0.01f);
}

// ============================================================================
// Clear功能测试
// ============================================================================

TEST(PIDTest, ClearResetsAllState) {
  PID pid(1.0f, 1.0f, 1.0f, 100.0f, 100.0f);

  // 运行几次累积状态
  for (int i = 0; i < 10; ++i) {
    pid.Update(10.0f, 0.0f, 0.01f);
  }

  pid.Clear();

  EXPECT_FLOAT_EQ(pid.set(), 0.0f);
  EXPECT_FLOAT_EQ(pid.out(), 0.0f);
  EXPECT_FLOAT_EQ(pid.p_out(), 0.0f);
  EXPECT_FLOAT_EQ(pid.i_out(), 0.0f);
  EXPECT_FLOAT_EQ(pid.d_out()[0], 0.0f);
  EXPECT_FLOAT_EQ(pid.d_out()[1], 0.0f);
  EXPECT_FLOAT_EQ(pid.ref()[0], 0.0f);
  EXPECT_FLOAT_EQ(pid.ref()[1], 0.0f);
  EXPECT_FLOAT_EQ(pid.error()[0], 0.0f);
  EXPECT_FLOAT_EQ(pid.error()[1], 0.0f);
}

TEST(PIDTest, ClearAndRestart) {
  PID pid(1.0f, 1.0f, 0.0f, 100.0f, 100.0f);

  pid.Update(10.0f, 0.0f, 0.01f);
  pid.Clear();
  pid.Update(10.0f, 0.0f, 0.01f);

  // 清除后积分项应该重新从0开始
  float expected_i = 0.05f;  // (10 + 0) / 2 * 0.01
  EXPECT_NEAR(pid.i_out(), expected_i, kFloatEpsilon);
}

// ============================================================================
// UpdateExtDiff测试（使用外部微分值）
// ============================================================================

TEST(PIDTest, UpdateExtDiffBasic) {
  PID pid(1.0f, 0.0f, 1.0f, 1000.0f, 0.0f);

  pid.UpdateExtDiff(10.0f, 0.0f, 5.0f, 0.01f);

  // P输出 = 10
  EXPECT_FLOAT_EQ(pid.p_out(), 10.0f);
  // D输出使用外部微分 = kd * external_diff = 1 * 5 = 5
  EXPECT_FLOAT_EQ(pid.d_out()[0], 5.0f);
  // 总输出 = 10 + 5 = 15
  EXPECT_FLOAT_EQ(pid.out(), 15.0f);
}

TEST(PIDTest, UpdateExtDiffWithFilter) {
  PID pid(0.0f, 0.0f, 1.0f, 1000.0f, 0.0f);
  pid.SetDiffLpfAlpha(0.5f);

  pid.UpdateExtDiff(0.0f, 0.0f, 10.0f, 0.01f);
  float first_d = pid.d_out()[0];

  pid.UpdateExtDiff(0.0f, 0.0f, 10.0f, 0.01f);
  float second_d = pid.d_out()[0];

  // 滤波效果：第二次d_out = alpha * 10 + (1-alpha) * first_d
  float expected = 0.5f * 10.0f + 0.5f * first_d;
  EXPECT_NEAR(second_d, expected, kFloatEpsilon);
}

TEST(PIDTest, UpdateExtDiffCircularEnabled) {
  PID pid(1.0f, 0.0f, 0.0f, 1000.0f, 0.0f);
  pid.SetCircular(true).SetCircularCycle(360.0f);

  pid.UpdateExtDiff(10.0f, 350.0f, 0.0f, 0.01f);

  // 过零点处理仍然适用
  EXPECT_NEAR(pid.error()[0], 20.0f, kFloatEpsilon);
}

// ============================================================================
// dt参数边界测试
// ============================================================================

TEST(PIDTest, VerySmallDt) {
  PID pid(1.0f, 1.0f, 1.0f, 1e10f, 1e10f);

  pid.Update(10.0f, 0.0f, 1e-6f);
  pid.Update(20.0f, 0.0f, 1e-6f);

  // 非常小的dt会导致微分项很大
  // 确保不会产生无穷大或NaN
  EXPECT_FALSE(std::isnan(pid.out()));
  EXPECT_FALSE(std::isinf(pid.out()));
}

TEST(PIDTest, LargeDt) {
  PID pid(1.0f, 1.0f, 1.0f, 1000.0f, 1000.0f);

  pid.Update(10.0f, 0.0f, 10.0f);

  EXPECT_FALSE(std::isnan(pid.out()));
  EXPECT_FALSE(std::isinf(pid.out()));
  // dt值记录正确
  EXPECT_FLOAT_EQ(pid.dt(), 10.0f);
}

TEST(PIDTest, DtEqualsOne) {
  PID pid(1.0f, 1.0f, 1.0f, 1000.0f, 1000.0f);

  pid.Update(10.0f, 0.0f);  // dt默认为1

  EXPECT_FLOAT_EQ(pid.dt(), 1.0f);
}

// ============================================================================
// 极端值测试
// ============================================================================

TEST(PIDTest, VeryLargeError) {
  PID pid(1.0f, 0.0f, 0.0f, 100.0f, 0.0f);

  pid.Update(1e6f, 0.0f, 0.01f);
  // 输出应该被限幅
  EXPECT_FLOAT_EQ(pid.out(), 100.0f);
}

TEST(PIDTest, VerySmallError) {
  PID pid(1.0f, 0.0f, 0.0f, 100.0f, 0.0f);

  pid.Update(1e-10f, 0.0f, 0.01f);

  EXPECT_NEAR(pid.out(), 1e-10f, kFloatEpsilon);
}

TEST(PIDTest, VeryLargeGains) {
  PID pid(1e6f, 1e6f, 1e6f, 1e10f, 1e10f);

  pid.Update(1.0f, 0.0f, 0.01f);

  EXPECT_FALSE(std::isnan(pid.out()));
  EXPECT_FALSE(std::isinf(pid.out()));
}

TEST(PIDTest, ZeroMaxOutputDoesNotPreventCalculation) {
  PID pid(1.0f, 1.0f, 1.0f, 0.0f, 0.0f);

  pid.Update(10.0f, 0.0f, 0.01f);

  // max_out = 0，输出被限幅到0
  EXPECT_FLOAT_EQ(pid.out(), 0.0f);
  // 但P项仍然计算
  EXPECT_FLOAT_EQ(pid.p_out(), 10.0f);
}

// ============================================================================
// 模糊PID测试
// ============================================================================

TEST(PIDTest, FuzzyPIDEnabled) {
  PID pid(1.0f, 0.1f, 0.01f, 100.0f, 50.0f);
  pid.SetFuzzy(true).SetFuzzyErrorScale(10.0f).SetFuzzyDErrorScale(100.0f);

  EXPECT_TRUE(pid.enable_fuzzy());

  pid.Update(5.0f, 0.0f, 0.01f);
  pid.Update(10.0f, 0.0f, 0.01f);

  // 模糊PID应该有输出
  EXPECT_NE(pid.out(), 0.0f);
  EXPECT_FALSE(std::isnan(pid.out()));
}

TEST(PIDTest, FuzzyPIDZeroError) {
  PID pid(1.0f, 0.0f, 0.0f, 100.0f, 0.0f);
  pid.SetFuzzy(true).SetFuzzyErrorScale(10.0f).SetFuzzyDErrorScale(100.0f);

  pid.Update(0.0f, 0.0f, 0.01f);
  pid.Update(0.0f, 0.0f, 0.01f);

  // 零误差时输出应该是0
  EXPECT_FLOAT_EQ(pid.out(), 0.0f);
}

TEST(PIDTest, FuzzyInferErrorScaleZero) {
  PID pid(1.0f, 0.0f, 0.0f, 100.0f, 0.0f);
  pid.SetFuzzy(true).SetFuzzyErrorScale(0.0f).SetFuzzyDErrorScale(0.0f);

  pid.Update(10.0f, 0.0f, 0.01f);
  pid.Update(20.0f, 0.0f, 0.01f);

  // 尺度因子为0时，不应该崩溃
  EXPECT_FALSE(std::isnan(pid.out()));
}

TEST(PIDTest, FuzzyInferGetters) {
  PID pid;
  pid.SetFuzzyErrorScale(10.0f).SetFuzzyDErrorScale(20.0f);

  EXPECT_FLOAT_EQ(pid.kp_fuzzy().error_scale(), 10.0f);
  EXPECT_FLOAT_EQ(pid.kp_fuzzy().d_error_scale(), 20.0f);
  EXPECT_FLOAT_EQ(pid.ki_fuzzy().error_scale(), 10.0f);
  EXPECT_FLOAT_EQ(pid.kd_fuzzy().d_error_scale(), 20.0f);
}

TEST(PIDTest, FuzzyRuleTableAccess) {
  PID pid;
  const auto& kp_rules = pid.kp_fuzzy().rule_table();

  // 验证规则表大小
  EXPECT_EQ(kp_rules.size(), 7);
  EXPECT_EQ(kp_rules[0].size(), 7);
}

TEST(PIDTest, FuzzyWithLargeInputs) {
  PID pid(1.0f, 0.1f, 0.01f, 1000.0f, 500.0f);
  pid.SetFuzzy(true).SetFuzzyErrorScale(10.0f).SetFuzzyDErrorScale(100.0f);

  // 误差远超尺度因子
  pid.Update(0.0f, 0.0f, 0.01f);
  pid.Update(1000.0f, 0.0f, 0.01f);

  EXPECT_FALSE(std::isnan(pid.out()));
  EXPECT_FALSE(std::isinf(pid.out()));
}

// ============================================================================
// 状态历史测试
// ============================================================================

TEST(PIDTest, ErrorHistoryTracking) {
  PID pid(1.0f, 0.0f, 0.0f, 100.0f, 0.0f);

  pid.Update(5.0f, 0.0f, 0.01f);  // error = 5
  EXPECT_FLOAT_EQ(pid.error()[0], 5.0f);

  pid.Update(10.0f, 0.0f, 0.01f);  // error = 10
  EXPECT_FLOAT_EQ(pid.error()[0], 10.0f);
  EXPECT_FLOAT_EQ(pid.error()[1], 5.0f);
}

TEST(PIDTest, RefHistoryTracking) {
  PID pid(1.0f, 0.0f, 0.0f, 100.0f, 0.0f);

  pid.Update(0.0f, 5.0f, 0.01f);
  EXPECT_FLOAT_EQ(pid.ref()[0], 5.0f);

  pid.Update(0.0f, 10.0f, 0.01f);
  EXPECT_FLOAT_EQ(pid.ref()[0], 10.0f);
  EXPECT_FLOAT_EQ(pid.ref()[1], 5.0f);
}

TEST(PIDTest, DOutHistoryTracking) {
  PID pid(0.0f, 0.0f, 1.0f, 10000.0f, 0.0f);
  pid.SetDiffLpfAlpha(1.0f);  // 无滤波

  pid.Update(0.0f, 0.0f, 0.01f);
  pid.Update(10.0f, 0.0f, 0.01f);
  float d1 = pid.d_out()[0];

  pid.Update(30.0f, 0.0f, 0.01f);
  EXPECT_FLOAT_EQ(pid.d_out()[1], d1);
}

// ============================================================================
// 连续运行稳定性测试
// ============================================================================

TEST(PIDTest, LongTermStability) {
  PID pid(1.0f, 0.1f, 0.01f, 100.0f, 50.0f);

  for (int i = 0; i < 10000; ++i) {
    float set = std::sin(i * 0.01f) * 50.0f;
    float ref = std::cos(i * 0.01f) * 50.0f;
    pid.Update(set, ref, 0.001f);

    EXPECT_FALSE(std::isnan(pid.out()));
    EXPECT_FALSE(std::isinf(pid.out()));
    EXPECT_LE(std::abs(pid.out()), 100.0f);
  }
}

TEST(PIDTest, RapidSetpointChanges) {
  PID pid(2.0f, 0.5f, 0.1f, 100.0f, 50.0f);

  for (int i = 0; i < 1000; ++i) {
    float set = (i % 2 == 0) ? 100.0f : -100.0f;
    pid.Update(set, 0.0f, 0.01f);

    EXPECT_FALSE(std::isnan(pid.out()));
    EXPECT_LE(std::abs(pid.out()), 100.0f);
  }
}

// ============================================================================
// 组合功能测试
// ============================================================================

TEST(PIDTest, CircularWithFuzzy) {
  PID pid(1.0f, 0.1f, 0.01f, 100.0f, 50.0f);
  pid.SetCircular(true)
      .SetCircularCycle(360.0f)
      .SetFuzzy(true)
      .SetFuzzyErrorScale(180.0f)
      .SetFuzzyDErrorScale(100.0f);

  pid.Update(10.0f, 350.0f, 0.01f);
  pid.Update(20.0f, 355.0f, 0.01f);

  EXPECT_FALSE(std::isnan(pid.out()));
  // 误差应该被正确处理（过零点）
  EXPECT_NEAR(pid.error()[0], 25.0f, kFloatEpsilon);
}

TEST(PIDTest, DiffFirstWithLowPass) {
  PID pid(0.0f, 0.0f, 1.0f, 1000.0f, 0.0f);
  pid.SetDiffFirst(true).SetDiffLpfAlpha(0.3f);

  pid.Update(0.0f, 0.0f, 0.01f);
  pid.Update(100.0f, 10.0f, 0.01f);  // set突变，ref也变
  pid.Update(100.0f, 10.0f, 0.01f);

  // 微分先行 + 滤波
  EXPECT_FALSE(std::isnan(pid.out()));
}

TEST(PIDTest, DynamicKiWithFuzzy) {
  PID pid(1.0f, 1.0f, 0.0f, 100.0f, 50.0f);
  pid.SetDynamicKi(true)
      .SetFuzzy(true)
      .SetFuzzyErrorScale(10.0f)
      .SetFuzzyDErrorScale(100.0f);

  // 现在可以直接启用变速积分，不需要预热
  pid.Update(5.0f, 0.0f, 0.01f);
  pid.Update(10.0f, 0.0f, 0.01f);
  pid.Update(15.0f, 0.0f, 0.01f);

  EXPECT_FALSE(std::isnan(pid.out()));
  // 积分项现在应该有值
  EXPECT_NE(pid.i_out(), 0.0f);
}

TEST(PIDTest, AllFeaturesEnabled) {
  PID pid(1.0f, 0.5f, 0.1f, 100.0f, 50.0f);
  pid.SetDiffFirst(true)
      .SetDiffLpfAlpha(0.5f)
      .SetDynamicKi(true)
      .SetCircular(true)
      .SetCircularCycle(360.0f)
      .SetFuzzy(true)
      .SetFuzzyErrorScale(180.0f)
      .SetFuzzyDErrorScale(100.0f);

  for (int i = 0; i < 100; ++i) {
    pid.Update(float(i % 360), float((i + 180) % 360), 0.01f);
    EXPECT_FALSE(std::isnan(pid.out()));
    EXPECT_FALSE(std::isinf(pid.out()));
  }
}

// ============================================================================
// 边界条件测试
// ============================================================================

TEST(PIDTest, MaxFloatValues) {
  PID pid(1.0f, 0.0f, 0.0f, std::numeric_limits<float>::max(), 0.0f);

  pid.Update(std::numeric_limits<float>::max() / 2, 0.0f, 0.01f);

  EXPECT_FALSE(std::isnan(pid.out()));
}

TEST(PIDTest, NegativeZeroHandling) {
  PID pid(1.0f, 1.0f, 1.0f, 100.0f, 50.0f);

  pid.Update(-0.0f, 0.0f, 0.01f);

  EXPECT_FLOAT_EQ(pid.error()[0], 0.0f);
  EXPECT_FLOAT_EQ(pid.out(), 0.0f);
}

TEST(PIDTest, SubnormalNumbers) {
  PID pid(1.0f, 0.0f, 0.0f, 100.0f, 0.0f);

  float subnormal = std::numeric_limits<float>::denorm_min();
  pid.Update(subnormal, 0.0f, 0.01f);

  EXPECT_FALSE(std::isnan(pid.out()));
}
