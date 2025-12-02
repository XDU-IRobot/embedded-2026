#include "Usb.hpp"
#include <algorithm>

#ifdef __cplusplus
extern "C" {
#endif

AimbotFrame_SCM_t Aimbot;
GimbalImuFrame_SCM_t GimbalImu;
uint16_t robot_id = 3;
uint8_t x[50];

// USB接收
void UsbReceive(uint8_t* rx_data, uint8_t len) {
  // C 风格 for → C++ std::copy
  std::copy(rx_data, rx_data + len, x);

  if (rx_data[0] == 0x55 && rx_data[len - 1] == 0xFF) {
    switch (rx_data[1]) {
      case AIMBOT_DATA_RECEIVE_ID:
        memcpy(&Aimbot, rx_data, len);
        break;

      default:
        break;
    }
  }
}

// USB发送
void UsbSendMessage(uint8_t* address, uint16_t len, uint8_t id) {
  address[0] = 0x55;
  address[1] = id;
  address[len - 1] = 0xff;
  CDC_Transmit_FS(address, len);
}

// IMU数据发送
void GimbalImuSend(float w, float x, float y, float z) {
  GimbalImu.TimeStamp = 0;

  GimbalImu.q0 = w;
  GimbalImu.q1 = x;
  GimbalImu.q2 = y;
  GimbalImu.q3 = z;

  GimbalImu.robot_id = robot_id;

  UsbSendMessage(reinterpret_cast<uint8_t*>(&GimbalImu), (uint16_t)sizeof(GimbalImu), IMU_DATA_SEND_ID);
}

#ifdef __cplusplus
}
#endif