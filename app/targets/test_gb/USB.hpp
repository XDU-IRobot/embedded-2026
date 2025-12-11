#ifndef USB_HPP
#define USB_HPP

#include <stdint.h>

typedef struct __attribute__((packed)) {
  // 包头
  uint8_t _SOF;
  uint8_t ID;
  // 云台数据
  uint32_t TimeStamp;
  float q0;
  float q1;
  float q2;
  float q3;
  uint8_t robot_id;
  uint8_t aim_mode;
  // 包尾
  uint8_t _EOF;
} GimbalDataFrame_SCM_t;

typedef struct __attribute__((packed)) {
  // 包头
  uint8_t _SOF;
  uint8_t ID;
  // 自瞄状态
  uint8_t AimbotState;
  uint8_t AimbotTarget;
  // 自瞄数据
  float Pitch;
  float Yaw;
  // 时间戳
  float SystemTimer;
  // 包尾
  uint8_t _EOF;
} AimbotFrame_SCM_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief          Usb接收数据
 * @param[in]      数据地址指针
 * @param[in]      数据长度
 * @retval         none
 */
void USBReceive(uint8_t *rx_data, uint8_t len);

/**
 * @brief          Usb数据发送
 * @param[in]      数据地址指针
 * @param[in]      数据长度
 * @param[in]      数据id
 * @retval         none
 */
void USBSendMessage(uint8_t *address, uint16_t len, uint8_t id);

/**
 * @brief          发送云台电机数据
 */
void GimbalDataSend();

#ifdef __cplusplus
}
#endif

#endif  // USB_HPP