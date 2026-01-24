#ifndef BOARDC_WS2812B_HPP
#define BOARDC_WS2812B_HPP

#include "main.h"
#include <math.h>

#define MAX_LED 1
#define USE_BRIGHTNESS 1

void Set_LED(int LEDnum, int Red, int Green, int Blue);

void Set_Brightness(int brightness);

void WS2812_Send();

#endif  // BOARDC_WS2812B_HPP