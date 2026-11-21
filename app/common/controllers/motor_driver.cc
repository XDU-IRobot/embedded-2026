/**
****************************************************************************************************
 * @file        motor_driver.cc
 * @author      ForTomorrow
 * @version     V1.0
 * @date        2026-11-16
 * @brief       简易 PID
 ****************************************************************************************************
 * @attention
 ****************************************************************************************************
 * @log  v1.0  去掉了冗杂的轨迹规划器，smc 可以实现在线的轨迹规划效果更优
 ****************************************************************************************************
 */

#include "motor_driver.hpp"

/**
 * @brief SMC 初始化
 */
void MotorDriver::SMCInit(float C, float K, float ref, float error_eps, float q, float p, float u_max, float J,
                          float epsilon, float dt) {
  smc_ = SMC(C, K, ref, error_eps, q, p, u_max, J, epsilon, dt);
}

/**
 * @brief 更新一步
 * @param raw_position 原始位置
 * @param raw_speed 原始速度
 * @param dt 时间间隔，单位 s
 * @note  保证输入单位一致
 */
void MotorDriver::Update(const float raw_position, const float raw_speed, const float dt) {
  // 计算当前位置和速度
  state_.position = raw_position;
  state_.speed = raw_speed;

  // 速度滤波
  // constexpr float kSpeedLpfAlpha = 0.15f;
  // static float filtered_speed = 0.0f;
  // filtered_speed = kSpeedLpfAlpha * raw_speed + (1.0f - kSpeedLpfAlpha) * filtered_speed;
  // state_.speed = filtered_speed;

  if (!is_enable_) {  // 无力，控制量设0直接返回
    output_ = 0.f;
    return;
  }

  // // 机械位置限幅(无 imu + enc 直接使用位置限位即可)
  // float safe_target_position = ApplyPositionLimits(target_.position, state_.position, raw_position);

  CalcOutput(target_.position, dt);

  // 输出限幅
  if (is_output_limits_) output_ = std::clamp(output_, limits_.min_out, limits_.max_out);
}

/**
 * @brief 更新一步
 * @param raw_imu_position IMU 位置
 * @param raw_enc_position 编码器位置
 * @param input_speed 希望输入的速度（可以是编码器也可以是 imu）
 * @param dt 时间间隔，单位 s
 * @note  imu用于pid，编码器用于安全限位
 * @note  编码器为绝对编码器，imu上电初始位置为 0
 * @note  注意保证编码器方向和 imu 方向一致，否则可能出现暴走!!!
 * @note  保证输入单位一致
 */
void MotorDriver::UpdateWithImu(const float raw_imu_position, const float raw_enc_position, const float input_speed, const float dt) {
  // 计算当前位置和速度
  state_.position = raw_imu_position;
  state_.speed = input_speed;

  if (!is_enable_) {  // 无力，控制量设0直接返回
    output_ = 0.f;
    return;
  }

  // 机械位置限幅
  CalcOutput(ApplyPositionLimits(target_.position, state_.position, raw_enc_position), dt);

  // 输出限幅
  if (is_output_limits_) output_ = std::clamp(output_, limits_.min_out, limits_.max_out);
}

void MotorDriver::CalcOutput(float target_position, const float dt) {
  switch (state_.mode) {
    case Mode::kPos: {  // 单位置环
      pid_.position.Update(target_position, state_.position, dt);
      output_ = pid_.position.out() + target_.output_ff;
      break;
    }
    case Mode::kSpeed: {  // 单速度环
      pid_.speed.Update(target_.speed, state_.speed, dt);
      output_ = pid_.speed.out() + target_.output_ff;
      break;
    }
    case Mode::kPosToSpeed: {  // 位置->速度（双环）
      pid_.position.Update(target_position, state_.position, dt);
      // 速度限幅
      const float speed_limit = std::abs(target_.speed);
      const float speed_target = std::clamp(pid_.position.out() + target_.speed_ff, -speed_limit, speed_limit);
      pid_.speed.Update(speed_target, state_.speed, dt);
      output_ = pid_.speed.out() + target_.output_ff;
      break;
    }
    case Mode::kSMC: {   // 滑膜（注：速度设置对滑膜无效果）
      smc_.Update(target_position, state_.position, state_.speed);   // smc 使用独立的 dt
      output_ = smc_.u + target_.output_ff;
    }
    default:
      break;
  }
}

/**
 * @brief 设置模式
 * @param mode 模式
 * @note 设置模式时会清除目标位置和速度
 */
void MotorDriver::SetMode(const Mode mode) {
  state_.mode = mode;
  // target_.speed = 0;
  // target_.position = 0;
  if (state_.mode == Mode::kSpeed) {
    target_.speed = 0;
  } else {
    target_.speed = std::numeric_limits<float>::infinity();  // 当切换为非速度模式时，不做速度限幅
  }
}

/**
 * @brief 设置目标速度和前馈量
 * @param target_speed 目标速度
 * @param speed_feedforward 速度前馈量
 * @note 仅在模式为 kSpeed 时有效
 */
void MotorDriver::SetSpeed(float target_speed, float speed_feedforward) {
  if (state_.mode == Mode::kSpeed || state_.mode == Mode::kPosToSpeed) {
    target_.speed = target_speed;
    target_.speed_ff = speed_feedforward;
  }
}

/**
 * @brief 设置目标位置、速度和前馈量
 * @param target_position    目标位置
 * @note kSpeed 模式下无效
 */
void MotorDriver::SetPosition(float target_position) {
  if (state_.mode != Mode::kSpeed) {
    // 目标限幅
    if (is_target_limits_) {
      target_position = std::clamp(target_position, limits_.min_target, limits_.max_target);
    }

    target_.position = target_position;
  }
}

/**
 * @brief 设置目标位置、速度和前馈量
 * @param speed_feedforward  速度前馈量
 * @param output_feedforward 输出前馈量
 * @note kSpeed 模式下无效
 */
void MotorDriver::SetFeedforward(float speed_feedforward, float output_feedforward) {
  target_.speed_ff = speed_feedforward;
  target_.output_ff = output_feedforward;
}

/**
 * @brief 设置输出限幅
 * @param min  输出下限
 * @param max  输出上限
 * @param is_enabled  是否启用
 */
void MotorDriver::SetOutputLimits(const float min, const float max, const bool is_enabled) {
  is_output_limits_ = is_enabled;
  limits_.min_out = min;
  limits_.max_out = max;
}

/**
 * @brief 设置位置限幅
 * @param min  位置下限，单位 rad
 * @param max  位置上限，单位 rad
 * @param is_enabled  是否启用
 */
void MotorDriver::SetPositionLimits(const float min, const float max, const bool is_enabled) {
  is_position_limits_ = is_enabled;
  limits_.min_position = min;
  limits_.max_position = max;
}

/**
 * @brief 设置目标限幅
 * @param min  目标下限，单位 rad
 * @param max  目标上限，单位 rad
 * @param is_enabled  是否启用
 */
void MotorDriver::SetTargetLimits(const float min, const float max, const bool is_enabled) {
  is_target_limits_ = is_enabled;
  limits_.min_target = min;
  limits_.max_target = max;
}

/**
 * @brief 执行位置限幅
 * @param target_world_pos  目标位置（世界坐标）
 * @param current_world_pos 当前实际位置（世界坐标）
 * @param encoder_pos       编码器位置（机械坐标）
 * @return 限幅后的目标位置（世界坐标）
 * @note 仅在 is_position_limits_ 为 true 时生效
 * @note 注意保证编码器方向和世界坐标方向一致，否则可能出现错误的限幅
 */
float MotorDriver::ApplyPositionLimits(float target_world_pos, float current_world_pos, float encoder_pos) const {
  if (!is_position_limits_) return target_world_pos;
  // 把世界目标映射到编码器坐标系
  const float enc_target = encoder_pos + (target_world_pos - current_world_pos);
  // 在编码器坐标系下 clamp
  const float enc_clamped = std::clamp(enc_target, limits_.min_position, limits_.max_position);
  // 计算 clamp 量，等价地施加到世界目标上
  return target_world_pos + (enc_clamped - enc_target);
}
