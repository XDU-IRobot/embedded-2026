#include "VOFA.hpp"

extern UART_HandleTypeDef huart1; // CubeMX 生成的句柄




void VOFA_SendPitch_Blocking(uint32_t t_ms,double pitch_rad, double pitch_current) {
    char buf[64];
    // 放大 10000 倍并四舍五入为整数（避免使用 %f）
    int int_pitch_rad = (int)(pitch_rad * 10000.0);
    int int_pitch_current = (int)(pitch_current * 10000.0);

    // 格式化：time_ms,[-]IP.FFFF\r\n
    // int len = snprintf(buf, sizeof(buf), "time:%u,%d,%d\n",(unsigned)t_ms,int_pitch_rad,int_pitch_current);
    int len = snprintf(buf, sizeof(buf), "time:%u,%d\n",(unsigned)t_ms,int_pitch_current);

    if (len > 0 && len < (int)sizeof(buf)) {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 20);
    }
}