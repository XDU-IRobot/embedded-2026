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
void GimbalImuSend() {
    globals->Imu.TimeStamp = 0;
    globals->Imu.q0 = globals->ahrs.quaternion().w;
    globals->Imu.q1 = globals->ahrs.quaternion().x;
    globals->Imu.q2 = globals->ahrs.quaternion().y;
    globals->Imu.q3 = globals->ahrs.quaternion().z;
    globals->Imu.robot_id = (globals->referee_data_buffer.data().robot_status.robot_id < 100) ? 1 : 0;
    USBSendMessage((uint8_t *) &globals->Imu, (uint16_t) sizeof(globals->Imu), 0x01);
}
#ifdef __cplusplus
}
#endif
