#include "USB.hpp"
#include "main.hpp"

#include "usbd_cdc_if.h"

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief          Usb接收数据
 * @param[in]      数据地址指针
 * @param[in]      数据长度
 * @retval         none
 */
void USBReceive(uint8_t *rx_data, uint8_t len) {
  if (rx_data[0] == 0x55 && rx_data[len - 1] == 0xFF) {
    switch (rx_data[1]) {
      case 0x02:
        memcpy(&globals->Aimbot, rx_data, len);
        break;

      case 0x04:
        memcpy(&globals->NucControl, rx_data, len);
        break;

      default:
        break;
    }
  }
}

/**
 * @brief          Usb数据发送
 * @param[in]      数据地址指针
 * @param[in]      数据长度
 * @param[in]      数据id
 * @retval         none
 */
void USBSendMessage(uint8_t *address, uint16_t len, uint8_t id) {
  address[0] = 0x55;
  address[1] = id;
  address[len - 1] = 0xff;
  CDC_Transmit_FS(address, len);
}

/**
 * @brief          数据处理与发送
 * @retval         none
 */
/**
 * @brief          发送云台电机数据
 */
void GimbalDataSend() {
  globals->GimbalData.TimeStamp = 0;
  globals->GimbalData.q0 = globals->up_yaw_qw;
  globals->GimbalData.q1 = globals->up_yaw_qx;
  globals->GimbalData.q2 = globals->up_yaw_qy;
  globals->GimbalData.q3 = globals->up_yaw_qz;
  globals->GimbalData.robot_id = globals->referee_data_buffer->data().robot_status.robot_id > 100 ? 1 : 0;
  USBSendMessage(reinterpret_cast<uint8_t *>(&globals->GimbalData), sizeof(globals->GimbalData), 0x03);
}

/**
 * @brief          发送自瞄所需裁判系统数据
 */
void RefereeDataSend() {
  globals->RefereeData.hurt_armor = globals->referee_data_buffer->data().hurt_data.armor_id;
  globals->RefereeData.current_HP = globals->referee_data_buffer->data().robot_status.current_HP;
  globals->RefereeData.game_progress = globals->referee_data_buffer->data().game_status.game_progress;
  globals->RefereeData.remain_time = globals->referee_data_buffer->data().game_status.stage_remain_time;
  globals->RefereeData.remain_bullet =
      globals->referee_data_buffer->data().projectile_allowance.projectile_allowance_17mm;
  globals->RefereeData.robot_id = globals->referee_data_buffer->data().robot_status.robot_id > 100 ? 1 : 0;
  USBSendMessage(reinterpret_cast<uint8_t *>(&globals->RefereeData), sizeof(globals->RefereeData), 0x07);
}

#ifdef __cplusplus
}
#endif
