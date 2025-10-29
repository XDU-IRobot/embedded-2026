#pragma once

#include <cstdint>

#include <etl/forward_list.h>
#include <etl/delegate.h>

#include "tim.h"

/**
 * @brief 基于 STM32
 * 定时器的定频任务封装。类接管一个定时器，利用定时器中断周期性执行回调函数，达到运行高精度定频任务的目的。
 *
 * 用法示例：
 *  - 在创建 TimerTask 时传入对应的 TIM_HandleTypeDef* 和回调。
 *  - 调用 Start() 启动定时器中断，Stop() 停止。
 *  - 在 HAL 的回调中转发中断：
 *      void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 *        TimerTask::HandleIRQ(htim);
 *      }
 */
class TimerTask {
 public:
  using Callback = etl::delegate<void()>;  // 回调函数类型

  explicit TimerTask(TIM_HandleTypeDef* htim = nullptr, Callback cb = {}) : htim_(htim), cb_(std::move(cb)) {
    Register(this);
    if (htim_) {
      // 注册 HAL 的周期回调，转发到 HandleIRQ
      HAL_TIM_RegisterCallback(htim_, HAL_TIM_PERIOD_ELAPSED_CB_ID, &TimerTask::HALTimerCallback);
    }
  }

  ~TimerTask() { Unregister(this); }

  // Non-copyable
  TimerTask(const TimerTask&) = delete;
  TimerTask& operator=(const TimerTask&) = delete;

  void Attach(TIM_HandleTypeDef* htim) {
    htim_ = htim;
    if (htim_) {
      HAL_TIM_RegisterCallback(htim_, HAL_TIM_PERIOD_ELAPSED_CB_ID, &TimerTask::HALTimerCallback);
    }
  }

  void SetCallback(Callback cb) { cb_ = cb; }  // SetCallback 签名调整

  // 启动定时器中断（基于 HAL）
  bool Start() {
    if (htim_ == nullptr) return false;
    return HAL_TIM_Base_Start_IT(htim_) == HAL_OK;
  }

  // 停止定时器中断
  bool Stop() {
    if (htim_ == nullptr) return false;
    return HAL_TIM_Base_Stop_IT(htim_) == HAL_OK;
  }

  // 直接设置定时器预分频和自动重装载寄存器（ARR/PSC）
  // 注意：修改寄存器后若要立即生效，可能需要先停止定时器再启动。
  void SetPrescalerAndPeriod(uint32_t psc, uint32_t arr) {
    if (htim_ == nullptr) return;
    __HAL_TIM_SET_PRESCALER(htim_, psc);
    __HAL_TIM_SET_AUTORELOAD(htim_, arr);
  }

  // 在中断上下文被调用（由 HandleIRQ 转发）
  void IRQHandler() {
    if (cb_) {
      cb_();  // 调用回调
    }
  }

  // 将 HAL 的周期中断回调转发到匹配的 TimerTask 实例
  static void HandleIRQ(TIM_HandleTypeDef* htim) {
    for (auto it = tasks_.begin(); it != tasks_.end(); ++it) {
      TimerTask* t = *it;
      if (t && t->htim_ == htim) {
        t->IRQHandler();
      }
    }
  }

 private:
  TIM_HandleTypeDef* htim_;
  Callback cb_;

  static etl::forward_list<TimerTask*, 16> tasks_;  ///< 注册的任务指针链表

  static void Register(TimerTask* t) {
    if (t == nullptr) return;
    tasks_.push_front(t);
  }

  static void Unregister(TimerTask* t) {
    if (t == nullptr) return;
    tasks_.remove(t);  // remove all matching entries
  }

  // 用于向 HAL 注册的全局回调，转发到 HandleIRQ
  static void HALTimerCallback(TIM_HandleTypeDef* htim) { HandleIRQ(htim); }
};

// 静态成员定义
inline etl::forward_list<TimerTask*, 16> TimerTask::tasks_;
