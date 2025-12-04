#ifndef USB_HPP
#define USB_HPP

#include "usbd_cdc_if.h"

#define IMU_DATA_SEND_ID 0x1        // 发送数据帧ID 0x1
#define AIMBOT_DATA_RECEIVE_ID 0x2  // 接收数据帧ID 0x2

typedef struct __attribute__((packed)) {
  uint8_t _SOF;            // 包头
  uint8_t ID;              // 接收id
  uint8_t AimbotState;     // 自瞄状态
  uint8_t AimbotTarget;    // 自瞄目标
  float PitchRelativeAngle;// pitch角度值
  float YawRelativeAngle;  // yaw角度值
  float TargetPitchSpeed;  // 目标pitch速度
  float TargetYawSpeed;    // 目标yaw速度
  uint32_t SystemTimer;    // 系统时间
  uint8_t _EOF;            // 包尾
} AimbotFrame_SCM_t;

typedef struct __attribute__((packed)) {
  uint8_t _SOF;      // 包头
  uint8_t ID;        // 发送id
  uint32_t TimeStamp;// 时间戳
  float q0;          // 四元数
  float q1;          //
  float q2;          //
  float q3;          //
  uint8_t robot_id;  // 机器人id
  uint8_t mode;      // 自瞄模式
  uint8_t _EOF;      // 包尾
} GimbalImuFrame_SCM_t;

#ifdef __cplusplus
extern "C" {
#endif
void UsbReceive(uint8_t* rx_data, uint8_t len);
void UsbSendMessage(uint8_t* address, uint16_t len, uint8_t id);
void GimbalImuSend(float w, float x, float y, float z);
#ifdef __cplusplus
}
#endif

#endif  // USB_HPP