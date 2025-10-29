
#pragma once

#include "rgb_led.hpp"

void LED::Init() {
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
}

void LED::operator()(uint32_t argb) {
  static uint8_t alpha;
  static uint16_t red, green, blue;

  alpha = (argb & 0xFF000000) >> 24;
  red = ((argb & 0x00FF0000) >> 16) * alpha;
  green = ((argb & 0x0000FF00) >> 8) * alpha;
  blue = ((argb & 0x000000FF) >> 0) * alpha;

  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
  __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
}
