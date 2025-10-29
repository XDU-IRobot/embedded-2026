#pragma once

#include <librm.hpp>

/**
 * @brief 三摩擦轮发射机构控制器
 */
class Shoot3Fric {
 public:
  explicit Shoot3Fric(int bullets_per_drum) : bullets_per_drum_(bullets_per_drum) {
    pid_.loader_position.SetCircular(true).SetCircularCycle(2.f * M_PI);  // 拨盘位置过零点处理
  }

  /**
   * @brief     更新一步，角度单位均为弧度，速度单位均为弧度每秒
   * @warning   注意处理电机减速比！这里传进来的数字应该是电机输出轴的实际位置和速度，而不是电机屁股的
   */
  void Update(float fric_1_speed, float fric_2_speed, float fric_3_speed, float loader_position, float loader_speed,
              float dt = 1.f) {
    state_.fric_1_speed = fric_1_speed;
    state_.fric_2_speed = fric_2_speed;
    state_.fric_3_speed = fric_3_speed;
    state_.loader_position = loader_position;
    state_.loader_speed = loader_speed;

    if (!enabled_) {  // 无力，控制量设0直接返回
      output_.fric_1 = 0.f;
      output_.fric_2 = 0.f;
      output_.fric_3 = 0.f;
      output_.loader = 0.f;
      return;
    }

    // 摩擦轮PID
    if (armed_) {
      pid_.fric_1_speed.Update(target_.fric_speed, state_.fric_1_speed, dt);
      output_.fric_1 = pid_.fric_1_speed.out();
      pid_.fric_2_speed.Update(target_.fric_speed, state_.fric_2_speed, dt);
      output_.fric_2 = pid_.fric_2_speed.out();
      pid_.fric_3_speed.Update(target_.fric_speed, state_.fric_3_speed, dt);
      output_.fric_3 = pid_.fric_3_speed.out();
    } else {
      output_.fric_1 = 0.f;
      output_.fric_2 = 0.f;
      output_.fric_3 = 0.f;
    }

    // 拨盘PID
    if (mode_ == kFullAuto) {
      // 全自动模式，速度环
      pid_.loader_speed.Update(target_.loader_speed, state_.loader_speed, dt);
      output_.loader = pid_.loader_speed.out();
    } else {
      // 单发模式，位置-速度串级PID
      pid_.loader_position.Update(target_.loader_position, state_.loader_position, dt);
      float loader_target_speed = pid_.loader_position.out();
      pid_.loader_speed.Update(loader_target_speed, state_.loader_speed, dt);
      output_.loader = pid_.loader_speed.out();
    }
  }

  /**
   * @brief 设置射频
   * @param frequency 射频，单位为发每秒
   */
  void SetShootFrequency(float frequency) {
    // 计算拨盘目标速度
    calculated_target_loader_speed_ = frequency * (2.f * M_PI / bullets_per_drum_);  // rad/s
  }

  /**
   * @brief 摩擦轮解锁/上锁
   */
  void Arm(bool enable) { armed_ = enable; }

  enum Mode {
    kFullAuto,    ///< 全自动模式
    kSingleShot,  ///< 单发模式
  };
  /**
   * @brief 设置射击模式（单发/全自动）
   */
  void SetMode(Mode mode) {
    CeaseFire();                                       // 停火
    target_.loader_position = state_.loader_position;  // 防止位置跳变
    mode_ = mode;
  }

  /**
   * @brief 启用或禁用控制器（切换有力无力）
   */
  void Enable(bool enable) { enabled_ = enable; }

  /**
   * @brief 开火，如果是单发模式则发射一发，如果是全自动模式则开始持续发射
   */
  void Fire() {
    if (!armed_) {  // 如果摩擦轮没有转（没有解锁），不开火
      return;
    }
    if (mode_ == kSingleShot) {
      // 单发模式，拨盘转动一个子弹间距
      target_.loader_position =
          rm::modules::Wrap(target_.loader_position + (2.f * M_PI / bullets_per_drum_), 0.f, M_PI * 2.f);
    } else {
      // 全自动模式，拨盘持续以计算得到的目标速度转动
      target_.loader_speed = calculated_target_loader_speed_;
    }
  }

  /**
   * @brief 停火，单发模式下调用无效，全自动模式下停止持续发射
   */
  void CeaseFire() {
    if (mode_ == kFullAuto) {
      target_.loader_speed = 0.f;
    }
  }

  // getters
  auto &pid() { return pid_; }
  auto &state() { return state_; }
  auto &target() { return target_; }
  auto &output() { return output_; }

 private:
  bool enabled_{false};                        ///< 有力/无力？
  bool armed_{true};                           ///< 摩擦轮转/不转？
  const int bullets_per_drum_;                 ///< 拨盘每圈子弹数
  float calculated_target_loader_speed_{0.f};  ///< 由射频计算得到的拨盘目标速度
  Mode mode_{kFullAuto};                       ///< 射击模式
  struct {
    rm::modules::PID fric_1_speed,  ///< 摩擦轮1速度环
        fric_2_speed,               ///< 摩擦轮2速度环
        fric_3_speed,               ///< 摩擦轮3速度环
        loader_position,            ///< 拨盘位置环，单发控制用
        loader_speed;               ///< 拨盘速度环
  } pid_;
  struct {
    float fric_1_speed;     ///< 摩擦轮1速度
    float fric_2_speed;     ///< 摩擦轮2速度
    float fric_3_speed;     ///< 摩擦轮3速度
    float loader_position;  ///< 拨盘位置
    float loader_speed;     ///< 拨盘速度
  } state_{};               ///< 当前状态
  struct {
    float fric_speed;       ///< 摩擦轮目标速度
    float loader_speed;     ///< 拨盘目标速度
    float loader_position;  ///< 拨盘目标位置
  } target_{};              ///< 目标状态
  struct {
    float fric_1;
    float fric_2;
    float fric_3;
    float loader;
  } output_{};  ///< 控制输出
};