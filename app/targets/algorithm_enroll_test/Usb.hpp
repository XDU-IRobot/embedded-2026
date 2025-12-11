#ifndef USB_HPP
#define USB_HPP


#include "usbd_cdc_if.h"

#define AIMBOT_DATA_RECEIVE_ID 0x2  // 接收数据帧ID 0x2


typedef struct __attribute__((packed)) {
    uint8_t _SOF;              // 包头         0x55
    uint8_t ID;                // 接收id       0x02
    float PitchRelativeAngle;  // pitch角度值  接收pitch数据
    float YawRelativeAngle;    // yaw角度值    接收yaw数据
    uint8_t _EOF;              // 包尾         0xff
} AimbotFrame_SCM_t;


#define IMU_DATA_SEND_ID 0x1        // 发送数据帧ID 0x1

typedef struct __attribute__((packed)) {
    uint8_t _SOF;        // 包头   0x55
    uint8_t ID;          // 发送id 01
    float q0;            // 四元数
    float q1;            //
    float q2;            //
    float q3;            //
    uint8_t _EOF;        // 包尾   0xff
} GimbalImuFrame_SCM_t;

#ifdef __cplusplus
extern "C" {
#endif
    void UsbReceive(uint8_t* rx_data, uint8_t len);
    void GimbalImuSend(float w, float x, float y, float z);
#ifdef __cplusplus
}
#endif

#endif  // USB_HPP