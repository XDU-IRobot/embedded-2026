
#pragma once

#include <functional>

#include "tim.h"

/**
 * @brief   简单阻塞式蜂鸣器
 */
class Buzzer {
 public:
  Buzzer(TIM_HandleTypeDef& htim = htim4, uint32_t channel = TIM_CHANNEL_3);

  void Init();

  void Beep(int beeps, int interval_ms = 50);

 private:
  TIM_HandleTypeDef* htim_;
  uint32_t channel_;
};

/**
 * @brief   利用定时器中断实现的异步蜂鸣器
 */
class AsyncBuzzer {
 public:
  AsyncBuzzer(TIM_HandleTypeDef& htim = htim4,   //
              uint32_t channel = TIM_CHANNEL_3,  //
              TIM_HandleTypeDef& htim_delay = htim14);

  void Init();

  void Beep(int beeps, int interval_ms = 50);

 private:
  static pTIM_CallbackTypeDef CallableObjToCallbackFnPtr(std::function<void(TIM_HandleTypeDef*)> func);

  void TimerCallback(TIM_HandleTypeDef* htim);

 private:
  int beep_count_{0};
  int beep_total_{0};
  bool beep_on_{false};

  TIM_HandleTypeDef* htim_;
  uint32_t channel_;
  TIM_HandleTypeDef* htim_delay_;
};