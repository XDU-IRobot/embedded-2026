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
  float motor_yaw_angle;
  uint8_t aim_mode;
  // 包尾
  uint8_t _EOF;
} GimbalDataFrame_SCM_t;

typedef struct __attribute__((packed)) {
  // 包头
  uint8_t _SOF;
  uint8_t ID;
  // 裁判系统数据
  uint8_t robot_id;
  uint16_t current_HP;
  uint16_t maximum_HP;
  float barrel1_speed;
  float barrel2_speed;
  float x;
  float y;
  // 包尾
  uint8_t _EOF;
} RefereeDataFrame_SCM_t;

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
  // float TargetPitchSpeed;
  // float TargetYawSpeed;
  // 时间戳
  float SystemTimer;
  // 包尾
  uint8_t _EOF;
} AimbotFrame_SCM_t;

typedef struct __attribute__((packed)) {
  // 包头
  uint8_t _SOF;
  uint8_t ID;
  // nuc控制
  float vx;
  float vy;
  float w;
  float yaw_speed;
  uint8_t chassis_mode;
  // uint32_t sentry_cmd;     // 哨兵自主决策信息
  // 包尾
  uint8_t _EOF;
} NucControlFrame_SCM_t;

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

/**
 * @brief          发送裁判系统数据
 */
void RefereeDataSend();

#ifdef __cplusplus
}
#endif

#endif  // USB_HPP