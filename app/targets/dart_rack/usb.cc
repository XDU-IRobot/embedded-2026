#include "usb.hpp"
#include "dart_core.hpp"
uint8_t x[50];
#ifdef __cplusplus
extern "C" {
#endif
  // USB接收
  void UsbReceive(uint8_t* rx_data, uint8_t len) {
    // C 风格 for → C++ std::copy
    std::copy(rx_data, rx_data + len, x);

    if (rx_data[0] == 0x55 && rx_data[len - 1] == 0xFF) {
      switch (rx_data[1]) {
        case AIMBOT_DATA_RECEIVE_ID:
         dart_rack->vision_data_->ID=rx_data[1];
         dart_rack->vision_data_->Pitch=*((float*)&rx_data[2]);
         dart_rack->vision_data_->Yaw=*((float*)&rx_data[6]);
         dart_rack->vision_data_->IsValiLock=*((bool*)&rx_data[10]);
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



#ifdef __cplusplus
}
#endif