#include "VOFA.hpp"

#pragma once

// 接收缓冲区定义
#define RX_BUF_SIZE 64
// extern uint8_t g_rx_buffer[RX_BUF_SIZE];
// extern Vofa_TxFrame g_vofa_tx;

Vofa_TxFrame g_vofa_tx;
// uint8_t g_rx_buffer[RX_BUF_SIZE];

// 假设你的 PID 参数变量
// extern float Kp, Ki, Kd;

/**
 * @brief 解析 VOFA+ 下行的字符串指令
 * 指令格式示例: "p:1.5\n", "i:0.01\n"
 */
// void VOFA_Parse_Command(char* str) {
//   char* value_ptr = strchr(str, ':');
//   if (!value_ptr) return;
//
//   float val = atof(value_ptr + 1);
//
//   if (strncmp(str, "p", 1) == 0) Kp = val;
//   else if (strncmp(str, "i", 1) == 0) Ki = val;
//   else if (strncmp(str, "d", 1) == 0) Kd = val;
// }

/**
 * @brief 准备数据包
 */
void VOFA_Prepare_Package(float x[], Vofa_TxFrame &tx_buffer, int len) {
  for (int i = 0; i < len; i++) {
    tx_buffer.x[i] = x[i];
  }
}

/**
 * @brief 上行发送函数（非阻塞 DMA）
 */
void VOFA_Send_JustFloat_DMA(UART_HandleTypeDef *huart, Vofa_TxFrame &tx_buffer) {
  if (huart->gState == HAL_UART_STATE_READY) {
    HAL_UART_Transmit_DMA(huart, reinterpret_cast<const uint8_t *>(&tx_buffer), sizeof(Vofa_TxFrame));
  }
}

/**
 * @brief 下行接收函数（非阻塞 DMA）
 */
void VOFA_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *rx_buffer) {
  if (huart->gState == HAL_UART_STATE_READY) {
    HAL_UART_Receive_DMA(huart, rx_buffer, sizeof(Vofa_TxFrame));
  }
}