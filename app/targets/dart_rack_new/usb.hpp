#pragma once
// cpp
#ifdef __cplusplus
#include <cstdbool>
#else
#include <stdbool.h>
#endif

#include "usbd_cdc_if.h"

#define IMU_DATA_SEND_ID 0x1           // 发送数据帧ID 0x1
#define AIMBOT_DATA_RECEIVE_ID 0x02    // 接收数据帧ID 0x02
#define AIMBOT_DATA_NOT_FOUND_ID 0x01  // 接收数据帧ID 0x01，表示未识别或错误)

#define VISION_LOCK_ERROR 0x00
#define VISION_LOCK_NONE 0x01
#define VISION_LOCK_VALID 0x02

typedef struct __attribute__((packed)) {
  uint8_t _SOF;        // 包头
  uint8_t ID;          // 接收id
  float Pitch;         // pitch目标
  float Yaw;           // yaw目标
  uint8_t IsValiLock;  // 识别状态: 0x00=错误 0x01=未识别 0x02=正确识别
  uint8_t _EOF;        // 包尾
} USBVisionReceive_SCM_t;

typedef struct __attribute__((packed)) {
  uint8_t _SOF;  // 包头
  uint8_t ID;    // 发送id
  uint8_t _EOF;  // 包尾
} USBVisionSend_SCM_t;

// USB 全局变量声明 (FreeMaster)
extern volatile uint32_t g_usb_rx_count;
extern volatile uint32_t g_usb_rx_matched;
extern volatile uint8_t g_vision_id;
extern volatile float g_vision_pitch;
extern volatile float g_vision_yaw;
extern volatile uint8_t g_vision_is_valid;

#ifdef __cplusplus
extern "C" {
#endif
void UsbReceive(uint8_t* rx_data, uint8_t len);
void UsbSendMessage(uint8_t* address, uint16_t len, uint8_t id);
#ifdef __cplusplus
}
#endif
