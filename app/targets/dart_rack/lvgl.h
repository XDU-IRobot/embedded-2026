#include "main.h"

#ifndef LVGL_H
#define LVGL_H
#define GPIOC_PIN_5 PLUS
#define GPIOC_PIN_1 MINUS
#define GPIOA_PIN_4 LEFT
#define GPIOF_PIN_10 RIGHT
#define GPIOI_PIN_9 ENSURE

#ifdef __cplusplus
extern "C" {
#endif

void init_lvgl_demo(void);

#ifdef __cplusplus
}
#endif

#endif  // LVGL_H
