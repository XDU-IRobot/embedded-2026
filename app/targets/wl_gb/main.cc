#include <librm.hpp>

#include "can.h"
#include "usart.h"
#include "spi.h"

#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "timer_task.hpp"

#include "comm_chassis.hpp"

rm::hal::stm32::Uart* imu_uart{nullptr};
rm::device::HipnucImu* imu{nullptr};
rm::hal::ThrottledCan<>* can_to_chassis{nullptr};
rm::hal::Serial* vt03_uart{nullptr};
rm::device::VT03* vt03{nullptr};
GimbalToChassisTxBridge* gb_to_chassis{nullptr};

// for debug
float yaw = 0.f;
float pitch = 0.f;
float roll = 0.f;

void Vt03RxCallback(const std::vector<rm::u8>& data, rm::u16 rx_len) {
  for (rm::u16 i = 0; i < rx_len; i++) {
    *vt03 << data.at(i);
  }
}

void MainLoop() {
  if (gb_to_chassis != nullptr) {
    gb_to_chassis->QueueSend();
  }

  yaw = imu->yaw();
  pitch = imu->pitch();
  roll = imu->roll();
}

extern "C" [[noreturn]] void AppMain(void) {
  can_to_chassis = new rm::hal::ThrottledCan<>{hcan2, 5000.0};
  can_to_chassis->SetFilter(0, 0);
  can_to_chassis->Begin();

  imu_uart = new rm::hal::stm32::Uart(huart1, 518, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma);
  imu = new rm::device::HipnucImu(*imu_uart);

  vt03_uart = new rm::hal::Serial(huart6, 128, rm::hal::stm32::UartMode::kDma, rm::hal::stm32::UartMode::kDma);
  vt03 = new rm::device::VT03;
  vt03_uart->AttachRxCallback(Vt03RxCallback);

  imu->Begin();
  vt03_uart->Begin();

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_1000hz.SetPrescalerAndPeriod(84 - 1, 1000 - 1);  // 84MHz / 84 / 1000 = 1kHz
  mainloop_1000hz.Start();

  for (;;) {
    can_to_chassis->Process();
    // __WFI();
  }
}
