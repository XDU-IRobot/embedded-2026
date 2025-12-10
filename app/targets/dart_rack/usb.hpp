#pragma once

#include "usbd_cdc_if.h"

#define IMU_DATA_SEND_ID 0x1        // 发送数据帧ID 0x1
#define AIMBOT_DATA_RECEIVE_ID 0x2  // 接收数据帧ID 0x2

typedef struct __attribute__((packed)) {
  uint8_t _SOF;     // 包头
  uint8_t ID;       // 接收id
  float Pitch;      // pitch目标
  float Yaw;        // yaw目标
  bool IsValiLock;  // 是否锁定目标
  uint8_t _EOF;     // 包尾
} USBVisionReceive_SCM_t;

typedef struct __attribute__((packed)) {
  uint8_t _SOF;  // 包头
  uint8_t ID;    // 发送id
  uint8_t _EOF;  // 包尾
} USBVisionSend_SCM_t;

#ifdef __cplusplus
extern "C" {
#endif
void UsbReceive(uint8_t* rx_data, uint8_t len);
void UsbSendMessage(uint8_t* address, uint16_t len, uint8_t id);
#ifdef __cplusplus
}
#endif
