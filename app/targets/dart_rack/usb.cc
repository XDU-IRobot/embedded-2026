#include "usb.hpp"
#include "dart_core.hpp"

uint8_t x[50];

volatile uint32_t g_usb_rx_count = 0;
volatile uint32_t g_usb_rx_matched = 0;
uint8_t g_vision_id = 0;
volatile float g_vision_pitch = 0.0f;
volatile float g_vision_yaw = 0.0f;

#ifdef __cplusplus
extern "C" {
#endif

void UsbReceive(uint8_t* rx_data, uint8_t len) {
  g_usb_rx_count++;
  std::copy(rx_data, rx_data + len, x);

  if (rx_data[0] == 0x55 && rx_data[len - 1] == 0xFF) {
    g_usb_rx_matched++;
    switch (rx_data[10]) {
      case AIMBOT_DATA_RECEIVE_ID:
        dart_rack->vision_data_->ID = rx_data[1];
        dart_rack->vision_data_->Pitch = *((float*)&rx_data[2]);
        dart_rack->vision_data_->Yaw = *((float*)&rx_data[6]);
        dart_rack->vision_data_->IsValiLock = rx_data[10];
        g_vision_id = rx_data[1];
        g_vision_pitch = dart_rack->vision_data_->Pitch;
        g_vision_yaw = dart_rack->vision_data_->Yaw;
        g_vision_is_valid = dart_rack->vision_data_->IsValiLock;
        break;

      default:
        break;
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
