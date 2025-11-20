#ifndef USB_HPP
#define USB_HPP

#include <stdint.h>
#include <stdbool.h>

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
  // 裁判系统数据
  uint8_t hurt_armor;  // 受击装甲板 0 1 2 3 4 (0表示未受击)
  uint16_t current_HP;
  // uint16_t maximum_HP;
  uint8_t game_progress;  // 比赛阶段 0 1 2 3 4 5（0未开始比赛，1准备阶段，2裁判自检，3五秒倒计时，4比赛中，5结算阶段）
  uint16_t remain_time;   // 比赛剩余时间（420s）
  uint16_t remain_bullet;  // 允许发弹量
  uint8_t robot_id;        // 0为红，1为蓝
  // float x;
  // float y;
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
  float yaw_speed;
  float vw;
  bool scan_mode;  // 0为不扫描，1为扫描
  // uint8 move_and_shoot_type;  1为移动模式，2为射击模式，3为防御模式
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
