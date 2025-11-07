#ifndef USB_HPP
#define USB_HPP

#include <stdint.h>
#include <string.h>

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
    // 自瞄目标角速度
    float TargetPitchSpeed;
    float TargetYawSpeed;
    // 时间戳
    float SystemTimer;
    // 包尾
    uint8_t _EOF;
    // 处理后数据
    float PitchRelativeAngle;
    float YawRelativeAngle;

} AimbotFrame_SCM_t;

typedef struct __attribute__((packed)) {
    // 包头
    uint8_t _SOF;
    uint8_t ID;
    // IMU数据
    uint32_t TimeStamp;
    float q0;
    float q1;
    float q2;
    float q3;
    uint8_t robot_id;
    uint8_t mode;
    // 包尾
    uint8_t _EOF;
} GimbalImuFrame_SCM_t;

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
     * @brief          数据处理与发送
     * @retval         none
     */
    void GimbalImuSend();

#ifdef __cplusplus
}
#endif

#endif // USB_HPP