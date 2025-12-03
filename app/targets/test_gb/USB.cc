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
  globals->GimbalData.q0 = globals->ahrs.quaternion().w;
  globals->GimbalData.q1 = globals->ahrs.quaternion().x;
  globals->GimbalData.q2 = globals->ahrs.quaternion().y;
  globals->GimbalData.q3 = globals->ahrs.quaternion().z;
  globals->GimbalData.robot_id = 0;
  USBSendMessage(reinterpret_cast<uint8_t *>(&globals->GimbalData), (uint16_t)sizeof(globals->GimbalData), 0x03);
}

#ifdef __cplusplus
}
#endif
