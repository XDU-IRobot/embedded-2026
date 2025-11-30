#ifndef USB_HPP
#define USB_HPP

#include "usbd_cdc_if.h"

#define IMU_DATA_SEND_ID 0x1        // 发送数据帧ID 0x1
#define AIMBOT_DATA_RECEIVE_ID 0x2  // 接收数据帧ID 0x2

typedef struct __attribute__((packed)) {
    uint8_t _SOF;
    uint8_t ID;
    uint8_t AimbotState;
    uint8_t AimbotTarget;
    float PitchRelativeAngle;
    float YawRelativeAngle;
    float TargetPitchSpeed;
    float TargetYawSpeed;
    uint32_t SystemTimer;
    uint8_t _EOF;
} AimbotFrame_SCM_t;


typedef struct __attribute__((packed)) {
    uint8_t _SOF;
    uint8_t ID;
    uint32_t TimeStamp;
    float q0;
    float q1;
    float q2;
    float q3;
    uint8_t robot_id;
    uint8_t mode;
    uint8_t _EOF;
} GimbalImuFrame_SCM_t;

#ifdef __cplusplus
extern "C" {
#endif
    void UsbReceive(uint8_t* rx_data, uint8_t len);
    void UsbSendMessage(uint8_t* address, uint16_t len, uint8_t id);
    void GimbalImuSend(float w,float x,float y,float z);
#ifdef __cplusplus
}
#endif

#endif //USB_HPP