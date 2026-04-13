//
// Created by 12628 on 26-4-3.
//

#include "lcd_init.h"
#include "spi.h"
#include "main.h"
#include "LCDFont.h"
#include "../../LVGL/lvgl.h"

LCD display;

void LCD_init(void) { display.Init(); }
void LCD_DISPLAY() {
  static bool has_displayed = false;
  if (!has_displayed) {
    // display.DrawLine(0, 0, W - 1, H - 1, BLUE);
    display.Fill(0, 0, W - 1, H - 1, WHITE);
    has_displayed = true;
  }
}

void LCD::WriteByte(uint8_t dat, uint8_t cmd) {
  if (cmd) {
    DC_Set();
  } else {
    DC_Clr();
  }
  CS_Clr();
  HAL_SPI_Transmit(&hspi4, &dat, 1, 100);
  CS_Set();
}

void LCD::Writ_Bus(uint8_t dat) {
  CS_Clr();
  HAL_SPI_Transmit(&hspi4, &dat, 1, 100);
  CS_Set();
}

void LCD::WR_DATA8(uint8_t dat) {
  DC_Set();
  Writ_Bus(dat);
}

void LCD::WR_DATA(uint16_t dat) {
  uint8_t buf[2];
  buf[0] = dat >> 8;
  buf[1] = dat;
  DC_Set();
  CS_Clr();
  HAL_SPI_Transmit(&hspi4, buf, 2, 100);
  CS_Set();
}

void LCD::WR_REG(uint8_t dat) {
  DC_Clr();  // 命令模式
  Writ_Bus(dat);
  DC_Set();
}

void LCD::Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
  WR_REG(0x2a);
  uint8_t x_buf[4] = {(uint8_t)(x1 >> 8), (uint8_t)x1, (uint8_t)(x2 >> 8), (uint8_t)x2};
  DC_Set();
  CS_Clr();
  HAL_SPI_Transmit(&hspi4, x_buf, 4, 100);
  CS_Set();

  WR_REG(0x2b);
  uint8_t y_buf[4] = {(uint8_t)(y1 >> 8), (uint8_t)y1, (uint8_t)(y2 >> 8), (uint8_t)y2};
  DC_Set();
  CS_Clr();
  HAL_SPI_Transmit(&hspi4, y_buf, 4, 100);
  CS_Set();

  WR_REG(0x2c);
}

void LCD::Init(void) {
  // CS默认高电平（未选中），低电平使能
  CS_Set();
  DC_Set();
  // BLKCLR默认打开（高电平点亮，低电平关闭）
  BLK_Set();
  HAL_Delay(50);

  // RES低电平复位
  RES_Clr();
  HAL_Delay(120);
  RES_Set();
  HAL_Delay(120);

  WR_REG(0x11);
  HAL_Delay(120);
  WR_REG(0Xf0);
  WR_DATA8(0xc3);
  WR_REG(0Xf0);
  WR_DATA8(0x96);
  WR_REG(0x36);  // Memory Access Control
  if (USE_HORIZONTAL == 0)
    WR_DATA8(0x48);
  else if (USE_HORIZONTAL == 1)
    WR_DATA8(0x88);
  else if (USE_HORIZONTAL == 2)
    WR_DATA8(0x28);
  else
    WR_DATA8(0xE8);
  WR_REG(0X3a);
  WR_DATA8(0x05);
  WR_REG(0Xe6);
  WR_DATA8(0x0f);
  WR_DATA8(0xf2);
  WR_DATA8(0x3f);
  WR_DATA8(0x4f);
  WR_DATA8(0x4f);
  WR_DATA8(0x28);
  WR_DATA8(0x0e);
  WR_DATA8(0x00);
  WR_REG(0Xc5);
  WR_DATA8(0x2a);
  WR_REG(0Xe0);
  WR_DATA8(0xf0);
  WR_DATA8(0x03);
  WR_DATA8(0x0a);
  WR_DATA8(0x11);
  WR_DATA8(0x14);
  WR_DATA8(0x1c);
  WR_DATA8(0x3b);
  WR_DATA8(0x55);
  WR_DATA8(0x4a);
  WR_DATA8(0x0a);
  WR_DATA8(0x13);
  WR_DATA8(0x14);
  WR_DATA8(0x1c);
  WR_DATA8(0x1f);
  WR_REG(0Xe1);
  WR_DATA8(0xf0);
  WR_DATA8(0x03);
  WR_DATA8(0x0a);
  WR_DATA8(0x0c);
  WR_DATA8(0x0c);
  WR_DATA8(0x09);
  WR_DATA8(0x36);
  WR_DATA8(0x54);
  WR_DATA8(0x49);
  WR_DATA8(0x0f);
  WR_DATA8(0x1b);
  WR_DATA8(0x18);
  WR_DATA8(0x1b);
  WR_DATA8(0x1f);
  WR_REG(0Xf0);
  WR_DATA8(0x3c);
  WR_REG(0Xf0);
  WR_DATA8(0x69);
  WR_REG(0X29);
}

void LCD::Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color) {
  uint16_t i, j;
  Address_Set(xsta, ysta, xend - 1, yend - 1);
  uint8_t buf[2] = {(uint8_t)(color >> 8), (uint8_t)color};
  DC_Set();
  CS_Clr();
  for (i = ysta; i < yend; i++) {
    for (j = xsta; j < xend; j++) {
      HAL_SPI_Transmit(&hspi4, buf, 2, 100);
    }
  }
  CS_Set();
}

void LCD::DrawPoint(uint16_t x, uint16_t y, uint16_t color) {
  Address_Set(x, y, x, y);
  WR_DATA(color);
}

void LCD::DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
  // 记录执行信息供 FreeMASTER 查看

  uint16_t t;
  int xerr = 0, yerr = 0, delta_x, delta_y, distance;
  int incx, incy, uRow, uCol;
  delta_x = x2 - x1;
  delta_y = y2 - y1;
  uRow = x1;
  uCol = y1;
  if (delta_x > 0)
    incx = 1;
  else if (delta_x == 0)
    incx = 0;
  else {
    incx = -1;
    delta_x = -delta_x;
  }
  if (delta_y > 0)
    incy = 1;
  else if (delta_y == 0)
    incy = 0;
  else {
    incy = -1;
    delta_y = -delta_y;
  }
  if (delta_x > delta_y)
    distance = delta_x;
  else
    distance = delta_y;
  for (t = 0; t < distance + 1; t++) {
    DrawPoint(uRow, uCol, color);
    xerr += delta_x;
    yerr += delta_y;
    if (xerr > distance) {
      xerr -= distance;
      uRow += incx;
    }
    if (yerr > distance) {
      yerr -= distance;
      uCol += incy;
    }
  }
}

void LCD::DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
  DrawLine(x1, y1, x2, y1, color);
  DrawLine(x1, y1, x1, y2, color);
  DrawLine(x1, y2, x2, y2, color);
  DrawLine(x2, y1, x2, y2, color);
}

void LCD::Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color) {
  int a, b;
  a = 0;
  b = r;
  while (a <= b) {
    DrawPoint(x0 - b, y0 - a, color);  // 3
    DrawPoint(x0 + b, y0 - a, color);  // 0
    DrawPoint(x0 - a, y0 + b, color);  // 1
    DrawPoint(x0 - a, y0 - b, color);  // 2
    DrawPoint(x0 + b, y0 + a, color);  // 4
    DrawPoint(x0 + a, y0 - b, color);  // 5
    DrawPoint(x0 + a, y0 + b, color);  // 6
    DrawPoint(x0 - b, y0 + a, color);  // 7
    a++;
    if ((a * a + b * b) > (r * r)) {
      b--;
    }
  }
}

void LCD::ShowChinese(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode) {
  while (*s != 0) {
    if (sizey == 12)
      ShowChinese12x12(x, y, s, fc, bc, sizey, mode);
    else if (sizey == 16)
      ShowChinese16x16(x, y, s, fc, bc, sizey, mode);
    else if (sizey == 24)
      ShowChinese24x24(x, y, s, fc, bc, sizey, mode);
    else if (sizey == 32)
      ShowChinese32x32(x, y, s, fc, bc, sizey, mode);
    else
      return;
    s += 2;
    x += sizey;
  }
}

void LCD::ShowChinese12x12(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode) {
  uint8_t i, j, m = 0;
  uint16_t k;
  uint16_t HZnum;
  uint16_t TypefaceNum;
  uint16_t x0 = x;
  TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;

  HZnum = sizeof(tfont12) / sizeof(typFNT_GB12);
  for (k = 0; k < HZnum; k++) {
    if ((tfont12[k].Index[0] == *(s)) && (tfont12[k].Index[1] == *(s + 1))) {
      Address_Set(x, y, x + sizey - 1, y + sizey - 1);
      for (i = 0; i < TypefaceNum; i++) {
        for (j = 0; j < 8; j++) {
          if (!mode) {
            if (tfont12[k].Msk[i] & (0x01 << j))
              WR_DATA(fc);
            else
              WR_DATA(bc);
            m++;
            if (m % sizey == 0) {
              m = 0;
              break;
            }
          } else {
            if (tfont12[k].Msk[i] & (0x01 << j)) DrawPoint(x, y, fc);
            x++;
            if ((x - x0) == sizey) {
              x = x0;
              y++;
              break;
            }
          }
        }
      }
    }
    continue;
  }
}

void LCD::ShowChinese16x16(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode) {
  uint8_t i, j, m = 0;
  uint16_t k;
  uint16_t HZnum;
  uint16_t TypefaceNum;
  uint16_t x0 = x;
  TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
  HZnum = sizeof(tfont16) / sizeof(typFNT_GB16);
  for (k = 0; k < HZnum; k++) {
    if ((tfont16[k].Index[0] == *(s)) && (tfont16[k].Index[1] == *(s + 1))) {
      Address_Set(x, y, x + sizey - 1, y + sizey - 1);
      for (i = 0; i < TypefaceNum; i++) {
        for (j = 0; j < 8; j++) {
          if (!mode) {
            if (tfont16[k].Msk[i] & (0x01 << j))
              WR_DATA(fc);
            else
              WR_DATA(bc);
            m++;
            if (m % sizey == 0) {
              m = 0;
              break;
            }
          } else {
            if (tfont16[k].Msk[i] & (0x01 << j)) DrawPoint(x, y, fc);  // ��һ����
            x++;
            if ((x - x0) == sizey) {
              x = x0;
              y++;
              break;
            }
          }
        }
      }
    }
    continue;
  }
}

void LCD::ShowChinese24x24(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode) {
  uint8_t i, j, m = 0;
  uint16_t k;
  uint16_t HZnum;
  uint16_t TypefaceNum;
  uint16_t x0 = x;
  TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
  HZnum = sizeof(tfont24) / sizeof(typFNT_GB24);  // ͳ�ƺ�����Ŀ
  for (k = 0; k < HZnum; k++) {
    if ((tfont24[k].Index[0] == *(s)) && (tfont24[k].Index[1] == *(s + 1))) {
      Address_Set(x, y, x + sizey - 1, y + sizey - 1);
      for (i = 0; i < TypefaceNum; i++) {
        for (j = 0; j < 8; j++) {
          if (!mode) {
            if (tfont24[k].Msk[i] & (0x01 << j))
              WR_DATA(fc);
            else
              WR_DATA(bc);
            m++;
            if (m % sizey == 0) {
              m = 0;
              break;
            }
          } else {
            if (tfont24[k].Msk[i] & (0x01 << j)) DrawPoint(x, y, fc);
            x++;
            if ((x - x0) == sizey) {
              x = x0;
              y++;
              break;
            }
          }
        }
      }
    }
    continue;
  }
}

void LCD::ShowChinese32x32(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode) {
  uint8_t i, j, m = 0;
  uint16_t k;
  uint16_t HZnum;
  uint16_t TypefaceNum;
  uint16_t x0 = x;
  TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
  HZnum = sizeof(tfont32) / sizeof(typFNT_GB32);
  for (k = 0; k < HZnum; k++) {
    if ((tfont32[k].Index[0] == *(s)) && (tfont32[k].Index[1] == *(s + 1))) {
      Address_Set(x, y, x + sizey - 1, y + sizey - 1);
      for (i = 0; i < TypefaceNum; i++) {
        for (j = 0; j < 8; j++) {
          if (!mode) {
            if (tfont32[k].Msk[i] & (0x01 << j))
              WR_DATA(fc);
            else
              WR_DATA(bc);
            m++;
            if (m % sizey == 0) {
              m = 0;
              break;
            }
          } else {
            if (tfont32[k].Msk[i] & (0x01 << j)) DrawPoint(x, y, fc);  // ��һ����
            x++;
            if ((x - x0) == sizey) {
              x = x0;
              y++;
              break;
            }
          }
        }
      }
    }
    continue;
  }
}

void LCD::ShowChar(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode) {
  uint8_t temp, sizex, t, m = 0;
  uint16_t i, TypefaceNum;
  uint16_t x0 = x;
  sizex = sizey / 2;
  TypefaceNum = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * sizey;
  num = num - ' ';
  Address_Set(x, y, x + sizex - 1, y + sizey - 1);
  for (i = 0; i < TypefaceNum; i++) {
    if (sizey == 12)
      temp = ascii_1206[num][i];
    else if (sizey == 16)
      temp = ascii_1608[num][i];
    else if (sizey == 24)
      temp = ascii_2412[num][i];
    else if (sizey == 32)
      temp = ascii_3216[num][i];
    else
      return;
    for (t = 0; t < 8; t++) {
      if (!mode) {
        if (temp & (0x01 << t))
          WR_DATA(fc);
        else
          WR_DATA(bc);
        m++;
        if (m % sizex == 0) {
          m = 0;
          break;
        }
      } else {
        if (temp & (0x01 << t)) DrawPoint(x, y, fc);
        x++;
        if ((x - x0) == sizex) {
          x = x0;
          y++;
          break;
        }
      }
    }
  }
}

void LCD::ShowString(uint16_t x, uint16_t y, const uint8_t *p, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode) {
  while (*p != '\0') {
    ShowChar(x, y, *p, fc, bc, sizey, mode);
    x += sizey / 2;
    p++;
  }
}

uint32_t LCD::mypow(uint8_t m, uint8_t n) {
  uint32_t result = 1;
  while (n--) result *= m;
  return result;
}

void LCD::ShowIntNum(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey) {
  uint8_t t, temp;
  uint8_t enshow = 0;
  uint8_t sizex = sizey / 2;
  for (t = 0; t < len; t++) {
    temp = (num / mypow(10, len - t - 1)) % 10;
    if (enshow == 0 && t < (len - 1)) {
      if (temp == 0) {
        ShowChar(x + t * sizex, y, ' ', fc, bc, sizey, 0);
        continue;
      } else
        enshow = 1;
    }
    ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
  }
}

void LCD::ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey) {
  uint8_t t, temp, sizex;
  uint16_t num1;
  sizex = sizey / 2;
  num1 = num * 100;
  for (t = 0; t < len; t++) {
    temp = (num1 / mypow(10, len - t - 1)) % 10;
    if (t == (len - 2)) {
      ShowChar(x + (len - 2) * sizex, y, '.', fc, bc, sizey, 0);
      t++;
      len += 1;
    }
    ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
  }
}

void LCD::ShowPicture(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[]) {
  uint16_t i, j;
  uint32_t k = 0;
  Address_Set(x, y, x + length - 1, y + width - 1);
  for (i = 0; i < length; i++) {
    for (j = 0; j < width; j++) {
      WR_DATA8(pic[k * 2]);
      WR_DATA8(pic[k * 2 + 1]);
      k++;
    }
  }
}

#include "../../LVGL/lvgl.h"

extern "C" void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
  uint16_t x1 = area->x1;
  uint16_t y1 = area->y1;
  uint16_t x2 = area->x2;
  uint16_t y2 = area->y2;

  // 设置区域
  display.Address_Set(x1, y1, x2, y2);

  uint32_t w = x2 - x1 + 1;
  uint32_t h = y2 - y1 + 1;
  uint32_t len = w * h;

  DC_Set();
  CS_Clr();

  // 批量发送提高速度
  uint16_t buf_size = 512;
  static uint16_t tx_buf[512];  // 使用 static 避免发生 Stack Overflow 导致系统卡死
  uint32_t i = 0;

  while (i < len) {
    uint32_t chunk_len = (len - i > buf_size) ? buf_size : (len - i);
    for (uint32_t j = 0; j < chunk_len; j++) {
      uint16_t c = lv_color_to16(color_p[i + j]);
      tx_buf[j] = (c >> 8) | (c << 8);  // 转换字节序
    }
    HAL_SPI_Transmit(&hspi4, (uint8_t *)tx_buf, chunk_len * 2, HAL_MAX_DELAY);
    i += chunk_len;
  }

  CS_Set();

  // 必须高速LVGL：刷屏完成了
  lv_disp_flush_ready(disp_drv);
}
