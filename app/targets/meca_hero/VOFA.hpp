#ifndef BOARDC_VOFA_HPP
#define BOARDC_VOFA_HPP

#include"librm.hpp"

#pragma pack(1)
struct Vofa_TxFrame {
  float pid_out;
  uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
};
#pragma pack()

void VOFA_Prepare_Package(const float &pid_out, Vofa_TxFrame &tx_buffer);
void VOFA_Send_JustFloat_DMA(UART_HandleTypeDef *huart, Vofa_TxFrame &tx_buffer);

#endif  // BOARDC_VOFA_HPP