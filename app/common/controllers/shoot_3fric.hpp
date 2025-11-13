#pragma once

#include <librm.hpp>

/**
 * @brief 三摩擦轮发射机构控制器
 */
class Shoot3Fric {
public:
    explicit Shoot3Fric(int bullets_per_drum, float reduction_ratio)
        : bullets_per_drum_(bullets_per_drum), loader_reduction_ratio_(reduction_ratio) {
        pid_.loader_position.SetCircular(true).SetCircularCycle(2.0f * M_PI); // 拨盘位置过零点处理
    }

    /**
     * @brief     更新一步，角度单位均为弧度，速度单位均为弧度每秒
     * @warning   注意处理电机减速比！这里传进来的数字应该是电机输出轴的实际位置和速度，而不是电机屁股的
     */
    void Update(float fric_1_speed, float fric_2_speed, float fric_3_speed, float loader_position, float loader_speed,
                float dt = 1.0f) {
        state_.fric_1_speed = fric_1_speed;
        state_.fric_2_speed = fric_2_speed;
        state_.fric_3_speed = fric_3_speed;
        state_.loader_speed = loader_speed;

        state_.loader_position = loader_position;
        if (loader_position >= target_.loader_position - 2000.0f) { single_shoot_complete_ = true; }

        if (!enabled_) {
            // 无力，控制量设0直接返回
            output_.fric_1 = 0.0f;
            output_.fric_2 = 0.0f;
            output_.fric_3 = 0.0f;
            output_.loader = 0.0f;
            return;
        }

        // 摩擦轮PID
        if (!armed_) {
            target_.fric_speed = 0.0f;
        }
        pid_.fric_1_speed.Update(target_.fric_speed, state_.fric_1_speed, dt);
        output_.fric_1 = pid_.fric_1_speed.out();
        pid_.fric_2_speed.Update(-target_.fric_speed, state_.fric_2_speed, dt);
        output_.fric_2 = pid_.fric_2_speed.out();
        pid_.fric_3_speed.Update(target_.fric_speed, state_.fric_3_speed, dt);
        output_.fric_3 = pid_.fric_3_speed.out();

        // 拨盘PID
        if (single_shoot_complete_ == false) {
            // 单发模式，位置-速度串级PID
            pid_.loader_position.Update(target_.loader_position, state_.loader_position, dt);
            pid_.loader_speed.Update(-7000.0f, state_.loader_speed, dt);
            output_.loader = pid_.loader_speed.out();
        } else {
            // 全自动模式，速度环
            target_.loader_position = state_.loader_position;
            pid_.loader_speed.Update(target_.loader_speed, state_.loader_speed, dt);
            output_.loader = pid_.loader_speed.out();
        }
    }

    /**
     * @brief 开火，如果是单发模式则发射一发，如果是全自动模式则开始持续发射
     */
    void Fire() {
        if (!armed_) {
            // 如果摩擦轮没有转（没有解锁），不开火
            return;
        }
        if (mode_ == kSingleShot && single_shoot_complete_) {
            // 单发模式，拨盘转动一个子弹间距
            target_.loader_position = state_.loader_position + loader_reduction_ratio_ /
                                      static_cast<float>(bullets_per_drum_) * 8191.f;
            state_.loader_circle_num_ = 0;
            single_shoot_complete_ = false;
        } else if (mode_ == kFullAuto) {
            // 全自动模式，拨盘持续以计算得到的目标速度转动
            target_.loader_speed = calculated_target_loader_speed_ * loader_reduction_ratio_;
        } else {
            target_.loader_speed = 0.0f;
        }
    }

    enum Mode {
        kStop, ///< 停止模式
        kFullAuto, ///< 全自动模式
        kSingleShot, ///< 单发模式
    };

    /**
     * @brief 设置射击模式（单发/全自动）
     */
    void SetMode(Mode mode) { mode_ = mode; }

    /**
     * @brief 设置射频
     * @param frequency 射频，单位为发每秒
     */
    void SetShootFrequency(float frequency) {
        // 计算拨盘目标速度
        calculated_target_loader_speed_ = frequency / static_cast<float>(bullets_per_drum_) * 60.0f; // rpm
    }

    /**
     * @brief 设置摩擦轮速度
     * @param fric_speed  摩擦轮速度，单位为弧度每秒
     */
    void SetArmSpeed(float fric_speed) { target_.fric_speed = fric_speed; }

    /**
     * @brief 摩擦轮解锁/上锁
     */
    void Arm(bool enable) { armed_ = enable; }

    /**
     * @brief 启用或禁用控制器（切换有力无力）
     */
    void Enable(bool enable) { enabled_ = enable; }

    // getters
    auto &pid() { return pid_; }
    auto &state() { return state_; }
    auto &target() { return target_; }
    auto &output() { return output_; }

    auto &shoot_flag() { return single_shoot_complete_; }

private:
    bool enabled_{false}; ///< 有力/无力？
    bool armed_{true}; ///< 摩擦轮转/不转？
    bool single_shoot_complete_{true}; ///< 单发模式是否发射完成
    const int bullets_per_drum_; ///< 拨盘每圈子弹数
    const float loader_reduction_ratio_; ///< 拨盘减速比
    float calculated_target_loader_speed_{0.0f}; ///< 由射频计算得到的拨盘目标速度
    Mode mode_{kFullAuto}; ///< 射击模式
    struct {
        rm::modules::PID fric_1_speed, ///< 摩擦轮1速度环
                fric_2_speed, ///< 摩擦轮2速度环
                fric_3_speed, ///< 摩擦轮3速度环
                loader_position, ///< 拨盘位置环，单发控制用
                loader_speed; ///< 拨盘速度环
    } pid_;

    struct {
        float fric_1_speed; ///< 摩擦轮1速度
        float fric_2_speed; ///< 摩擦轮2速度
        float fric_3_speed; ///< 摩擦轮3速度
        float loader_speed; ///< 拨盘速度
        float loader_position; ///< 拨盘位置
        int loader_circle_num_{0}; ///< 拨盘圈数
    } state_{}; ///< 当前状态
    struct {
        float fric_speed; ///< 摩擦轮目标速度
        float loader_speed; ///< 拨盘目标速度
        float loader_position; ///< 拨盘目标位置
    } target_{}; ///< 目标状态
    struct {
        float fric_1;
        float fric_2;
        float fric_3;
        float loader;
    } output_{}; ///< 控制输出
};
