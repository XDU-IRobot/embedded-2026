//
// Created by 12628 on 26-4-3.
//
#ifndef __INIT_H
#define __INIT_H

#include "main.h"
#include <stdint.h>
#define USE_HORIZONTAL 2

#if USE_HORIZONTAL == 0 || USE_HORIZONTAL == 1
#define W 320
#define H 480

#else
#define W 480
#define H 320
#endif

#define SCLK_Clr() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET)
#define SCLK_Set() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET)

#define MOSI_Clr() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET)
#define MOSI_Set() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET)

#define RES_Clr() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define RES_Set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)

#define DC_Clr() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET)
#define DC_Set() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET)

#define CS_Clr() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define CS_Set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)

#define BLK_Clr() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET)
#define BLK_Set() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET)

#define WHITE 0xFFFF
#define BLACK 0x0000
#define BLUE 0x001F
#define BRED 0XF81F
#define GRED 0XFFE0
#define GBLUE 0X07FF
#define RED 0xF800
#define MAGENTA 0xF81F
#define GREEN 0x07E0
#define CYAN 0x7FFF
#define YELLOW 0xFFE0
#define BROWN 0XBC40
#define BRRED 0XFC07
#define GRAY 0X8430
#define DARKBLUE 0X01CF
#define LIGHTBLUE 0X7D7C
#define GRAYBLUE 0X5458
#define LIGHTGREEN 0X841F
#define LGRAY 0XC618
#define LGRAYBLUE 0XA651
#define LBBLUE 0X2B12

#define LCD_CS_PORT GPIOB
#define LCD_DC_PORT GPIOC
#define LCD_RES_PORT GPIOB
#define LCD_CS_PIN GPIO_PIN_0
#define LCD_DC_PIN GPIO_PIN_2
#define LCD_RES_PIN GPIO_PIN_1

void LCD_init(void);
void LCD_DISPLAY();

class LCD {
 public:
  void WriteByte(uint8_t dat, uint8_t cmd);
  void Writ_Bus(uint8_t dat);
  void WR_DATA8(uint8_t dat);
  void WR_DATA(uint16_t dat);
  void WR_REG(uint8_t dat);
  void Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
  void Init(void);

  void Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color);
  void DrawPoint(uint16_t x, uint16_t y, uint16_t color);
  void DrawLine(uint16_t x1a, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
  void DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
  void Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);

  void ShowChinese(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
  void ShowChinese12x12(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
  void ShowChinese16x16(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
  void ShowChinese24x24(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
  void ShowChinese32x32(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);

  void ShowChar(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
  void ShowString(uint16_t x, uint16_t y, const uint8_t *p, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
  uint32_t mypow(uint8_t m, uint8_t n);
  void ShowIntNum(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey);
  void ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey);

  void ShowPicture(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[]);
};
#endif
