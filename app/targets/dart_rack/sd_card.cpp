//
// Created by 12628 on 26-4-9.
//

#include "sd_card.h"

#include "sdio.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"
#include "fatfs.h"  // 引用官方 FatFs 的头文件

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int last_sd_error_step = 0;
FRESULT last_sd_error_code = FR_OK;
static bool sd_mounted = false;  // 记录是否已经成功挂载过一次

static bool init_and_mount_sd(void) {
  if (sd_mounted) return true;

  // 开机时先做一次极速的底层探测，如果单片机跟 SD 卡芯片在物理层都不通（如没插卡、线断了）
  // 就直接快速拦截，避免进入 f_mount 引发 FatFs 长达数秒的“没卡硬读”超时等待（导致开机慢/白屏）
  if (HAL_SD_Init(&hsd) != HAL_OK) {
    last_sd_error_code = FR_NOT_READY;
    return false;
  }

  FRESULT res = FR_NOT_READY;
  // 减少重试次数到1次，否则开机如果无卡或挂载出错会导致 FatFs 超时阻塞过久
  for (int retry = 0; retry < 1; retry++) {
    // 在尝试挂载前，先检查与等待底层状态就绪
    int wait_cnt = 5;
    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER && wait_cnt > 0) {
      HAL_Delay(10);
      wait_cnt--;
    }

    res = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1);
    if (res == FR_OK) {
      sd_mounted = true;
      return true;
    }

    // 挂载失败强制重启一次寄存器
    HAL_SD_DeInit(&hsd);
    HAL_Delay(5);
    HAL_SD_Init(&hsd);
    HAL_Delay(5);
  }

  last_sd_error_code = res;
  return false;
}

bool Save_Params_To_SD(float Pitch[4], float Yaw[4]) {
  FRESULT res = FR_OK;
  UINT byteswritten = 0;
  char text[128];
  char path[32];
  snprintf(path, sizeof(path), "%sparams.txt", SDPath);

  char debug_buf[128];
  snprintf(debug_buf, sizeof(debug_buf), "--- SD Save Start ---\r\n");
  HAL_UART_Transmit(&huart6, (uint8_t *)debug_buf, strlen(debug_buf), 100);

  bool success = false;

  // 增加重试循环：如果遇到文件被锁 (16)、或者底层异常 (1)，直接重置系统并重试一次
  for (int retry = 0; retry < 2; retry++) {
    // 调用强化版的尝试挂载函数
    if (!init_and_mount_sd()) {
      last_sd_error_step = 1;  // Mount failed
      return false;            // 挂载失败
    }

    // 预防性关闭，清理可能残留的悬空句柄（防 FR_LOCKED 16 报错）
    f_close(&SDFile);

    // 打开或创建 params.txt 文件，给允许写入的权限
    res = f_open(&SDFile, path, FA_CREATE_ALWAYS | FA_WRITE);

    snprintf(debug_buf, sizeof(debug_buf), "Save f_open res: %d\r\n", res);
    HAL_UART_Transmit(&huart6, (uint8_t *)debug_buf, strlen(debug_buf), 100);

    if (res == FR_OK) {
      // 由于嵌入式 nano libc 默认可能不支持 %f，使用 * 1000 转换为整形处理
      int len =
          snprintf(text, sizeof(text), "%d %d %d %d %d %d %d %d", (int)(Pitch[0] * 1000.0f), (int)(Pitch[1] * 1000.0f),
                   (int)(Pitch[2] * 1000.0f), (int)(Pitch[3] * 1000.0f), (int)(Yaw[0] * 1000.0f),
                   (int)(Yaw[1] * 1000.0f), (int)(Yaw[2] * 1000.0f), (int)(Yaw[3] * 1000.0f));

      // 写入文件
      FRESULT write_res = f_write(&SDFile, text, len, &byteswritten);

      snprintf(debug_buf, sizeof(debug_buf), "Save f_write res: %d, bytes written: %u\r\n", write_res, byteswritten);
      HAL_UART_Transmit(&huart6, (uint8_t *)debug_buf, strlen(debug_buf), 100);

      FRESULT sync_res = f_sync(&SDFile);
      FRESULT close_res = f_close(&SDFile);

      snprintf(debug_buf, sizeof(debug_buf), "sync:%d cls:%d\r\n", sync_res, close_res);
      HAL_UART_Transmit(&huart6, (uint8_t *)debug_buf, strlen(debug_buf), 100);

      if (write_res == FR_OK && byteswritten > 0 && close_res == FR_OK) {
        success = true;
        break;  // 成功即跳出重试
      } else {
        last_sd_error_step = 3;  // Write/Sync/Close failed
        last_sd_error_code = close_res != FR_OK ? close_res : write_res;
        f_mount(NULL, (TCHAR const *)SDPath, 0);
        sd_mounted = false;
      }
    } else {
      last_sd_error_step = 2;  // Open failed
      last_sd_error_code = res;

      // 若写入失败（如卡被拔出硬件错乱 1，或者文件系统锁死 16），强制卸载并重置状态使下次能完全重新初始化
      f_mount(NULL, (TCHAR const *)SDPath, 0);
      sd_mounted = false;
    }
  }

  // 等待 SD 卡状态就绪，确保数据真实落盘
  int wait_cnt = 50;
  while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER && wait_cnt > 0) {
    HAL_Delay(10);
    wait_cnt--;
  }

  return success;
}

void Load_Params_From_SD(float Pitch[4], float Yaw[4]) {
  FRESULT res = FR_OK;
  UINT bytesread;
  char text[128] = {0};
  char path[32];
  snprintf(path, sizeof(path), "%sparams.txt", SDPath);

  char debug_buf[64];
  snprintf(debug_buf, sizeof(debug_buf), "--- SD Load Start ---\r\n");
  HAL_UART_Transmit(&huart6, (uint8_t *)debug_buf, strlen(debug_buf), 100);

  // 强化版的挂载
  if (!init_and_mount_sd()) {
    snprintf(debug_buf, sizeof(debug_buf), "Mount failed: %d\r\n", last_sd_error_code);
    HAL_UART_Transmit(&huart6, (uint8_t *)debug_buf, strlen(debug_buf), 100);
    return;  // 挂载失败
  }

  f_close(&SDFile);

  // 以只读形式打开 params.txt 读取数据
  res = f_open(&SDFile, path, FA_READ);
  snprintf(debug_buf, sizeof(debug_buf), "f_open res: %d\r\n", res);
  HAL_UART_Transmit(&huart6, (uint8_t *)debug_buf, strlen(debug_buf), 100);

  if (res == FR_OK) {
    // 读取文件
    f_read(&SDFile, text, sizeof(text) - 1, &bytesread);

    snprintf(debug_buf, sizeof(debug_buf), "f_read bytes: %u\r\n", bytesread);
    HAL_UART_Transmit(&huart6, (uint8_t *)debug_buf, strlen(debug_buf), 100);

    // 如果文件不为空，解析格式并赋值回变量
    if (bytesread > 0) {
      text[bytesread] = '\0';  // 确保字符串以 \0 结尾

      // 调试用：把读取到的字符串打印出来看看
      char uart_buf[150];
      snprintf(uart_buf, sizeof(uart_buf), "SD Read: %s\r\n", text);
      HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

      int p[4] = {0}, y[4] = {0};

      // 使用更稳健的解析方式，因为部分嵌入式 libc 的 sscanf 可能无法同时支持这么多参数
      char *ptr = text;
      for (int i = 0; i < 4; i++) {
        p[i] = strtol(ptr, &ptr, 10);
      }
      for (int i = 0; i < 4; i++) {
        y[i] = strtol(ptr, &ptr, 10);
      }

      for (int i = 0; i < 4; i++) {
        // 如果解析出来是0且原来不是0（意味着可能解析错误），则加上额外的防线，
        // 但为了简单，直接赋值即可
        if (p[i] != 0 || y[i] != 0) {  // 防止纯0覆盖了默认值，如果有真实0的情况需更严谨判断
          Pitch[i] = p[i] / 1000.0f;
          Yaw[i] = y[i] / 1000.0f;
        }
      }
    }

    // 关闭文件
    f_close(&SDFile);
  } else {
    // 若读取遇到异常，同样重置挂载标记，保证后续的写操作能正常重置恢复
    f_mount(NULL, (TCHAR const *)SDPath, 0);
    sd_mounted = false;
  }
}

void SD_Card_info() {
  HAL_SD_CardInfoTypeDef card_info;
  char uart_buf[256];

  // HAL_OK is 0, so we should check if it equals HAL_OK
  if (HAL_SD_GetCardInfo(&hsd, &card_info) == HAL_OK) {
    snprintf(uart_buf, sizeof(uart_buf),
             "CardType :%lu\r\n"
             "CardVersion :%lu\r\n"
             "Class :%lu\r\n"
             "RelCardAdd :%lu\r\n"
             "BlockNbr :%lu\r\n"
             "BlockSize :%lu\r\n",
             (unsigned long)card_info.CardType, (unsigned long)card_info.CardVersion, (unsigned long)card_info.Class,
             (unsigned long)card_info.RelCardAdd, (unsigned long)card_info.BlockNbr,
             (unsigned long)card_info.BlockSize);
    HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
  } else {
    snprintf(uart_buf, sizeof(uart_buf), "SD Info Error\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
  }
}