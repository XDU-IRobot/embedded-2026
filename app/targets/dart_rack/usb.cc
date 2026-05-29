#include "usb.hpp"
#include "dart_core.hpp"

uint8_t x[50];

volatile uint32_t g_usb_rx_count ;
volatile uint32_t g_usb_rx_matched = 0;
uint8_t g_vision_id = 0;
volatile float g_vision_pitch = 0.0f;
volatile float g_vision_yaw = 0.0f;

#ifdef __cplusplus
extern "C" {
#endif

void UsbReceive(uint8_t* rx_data, uint8_t len) {
  g_usb_rx_count++;
  if (len < sizeof(x)) {
    std::copy(rx_data, rx_data + len, x);
  }

  // 增加长度判断 (至少需要11字节保障不会越界: index 0 ~ 10, 外加包尾)
  if (len >= 12 && rx_data[0] == 0x55 && rx_data[len - 1] == 0xFF) {
    g_usb_rx_matched++;
    if (rx_data[10] == AIMBOT_DATA_NOT_FOUND_ID) {
      dart_rack->vision_data_->ID = 0;
      dart_rack->vision_data_->Pitch = 0.0f;
      dart_rack->vision_data_->Yaw = 0.0f;
      dart_rack->vision_data_->IsValiLock = 0;
      g_vision_id = 0;
      g_vision_pitch = 0.0f;
      g_vision_yaw = 0.0f;
      g_vision_is_valid = 0;
    }
    else if (rx_data[10] == AIMBOT_DATA_RECEIVE_ID) {
      dart_rack->vision_data_->ID = rx_data[1];
      dart_rack->vision_data_->Pitch = *((float*)&rx_data[2]);
      dart_rack->vision_data_->Yaw = *((float*)&rx_data[6]);
      dart_rack->vision_data_->IsValiLock = rx_data[10];
      g_vision_id = rx_data[1];
      g_vision_pitch = dart_rack->vision_data_->Pitch;
      g_vision_yaw = dart_rack->vision_data_->Yaw;
      g_vision_is_valid = dart_rack->vision_data_->IsValiLock;
    }
    else {
      dart_rack->vision_data_->ID = 0;
      dart_rack->vision_data_->Pitch = 0.0f;
      dart_rack->vision_data_->Yaw = 0.0f;
      dart_rack->vision_data_->IsValiLock = 0;
      g_vision_id = 0;
      g_vision_pitch = 0.0f;
      g_vision_yaw = 0.0f;
      g_vision_is_valid = 0;
    }
  }
}

void UsbSendMessage(uint8_t* address, uint16_t len, uint8_t id) {
  address[0] = 0x55;
  address[1] = id;
  address[len - 1] = 0xff;
  CDC_Transmit_FS(address, len);
}

#ifdef __cplusplus
}
#endif
