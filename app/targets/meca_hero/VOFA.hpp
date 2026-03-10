#ifndef BOARDC_VOFA_HPP
#define BOARDC_VOFA_HPP

#include "usart.h"

void VOFA_Send_JustFloat_DMA(UART_HandleTypeDef* huart);
struct Vofa_TxFrame;

#endif  // BOARDC_VOFA_HPP