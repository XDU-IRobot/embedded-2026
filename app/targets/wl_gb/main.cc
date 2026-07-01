#include <librm.hpp>

#include "can.h"
#include "usart.h"
#include "spi.h"

#include "rgb_led.hpp"
#include "buzzer.hpp"
#include "timer_task.hpp"

#include "comm_chassis.hpp"
#include "wl_gb/include/shoot_ctrl.hpp"

rm::hal::stm32::Uart* imu_uart{nullptr};
rm::device::HipnucImu* imu{nullptr};
rm::hal::ThrottledCan<>* can_to_chassis{nullptr};
rm::hal::ThrottledCan<>* can_to_shoot{nullptr};
rm::hal::Serial* vt03_uart{nullptr};
rm::device::VT03* vt03{nullptr};
GimbalToChassisTxBridge* gb_to_chassis{nullptr};
ChassisToGimbalRxBridge* chassis_rx{nullptr};

// referee
rm::device::Referee<rm::device::RefereeRevision::kNewV120>* referee{nullptr};
EmyRobotHP robotHP{};

// shoot controller
ShootCtrl shoot_ctrl{};

// 摩擦轮电机转速, 1-6 对应 CAN ID 1-6
int16_t fw_rpm_1 = 0;
int16_t fw_rpm_2 = 0;
int16_t fw_rpm_3 = 0;
int16_t fw_rpm_4 = 0;
int16_t fw_rpm_5 = 0;
int16_t fw_rpm_6 = 0;

// for debug
float yaw = 0.f;
float pitch = 0.f;
float roll = 0.f;
static int sof_count = 0;
static int imu_status = 0;

void Vt03RxCallback(const std::vector<rm::u8>& data, rm::u16 rx_len) {
  for (rm::u16 i = 0; i < rx_len; i++) {
    if (data.at(i) == 0xA5) sof_count++;
    *vt03 << data.at(i);
    *referee << data.at(i);
  }
}

// Z/X 键调速状态
bool z_pressed_prev = false;
bool x_pressed_prev = false;

void MainLoop() {
  // VT03 Z/X 键调速
  if (vt03) {
    const bool ctrl_pressed = (vt03->data().keyboard_key & 0x1000) != 0;  // Ctrl
    const bool z_pressed = (vt03->data().keyboard_key & 0x0002) != 0;     // Z
    const bool x_pressed = (vt03->data().keyboard_key & 0x0004) != 0;     // X

    // Ctrl+Z: 减速
    if (ctrl_pressed && z_pressed && !z_pressed_prev) {
      shoot_ctrl.AdjustSpeed(-kFricSpeedStepRpm);
    }
    z_pressed_prev = z_pressed;

    // Ctrl+X: 升速
    if (ctrl_pressed && x_pressed && !x_pressed_prev) {
      shoot_ctrl.AdjustSpeed(kFricSpeedStepRpm);
    }
    x_pressed_prev = x_pressed;
  }

  // 发射控制：使用底盘发来的 combat_mode
  const bool enter_shoot = (chassis_rx && chassis_rx->combat_mode());
  shoot_ctrl.Update(enter_shoot);

  if (gb_to_chassis != nullptr) {
    if (referee) {
      memcpy(&robotHP, &referee->data().robot_custom_data_3.data, 10);
      gb_to_chassis->UpdateRobotHP(robotHP);
    }
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
    fw_rpm_1 = shoot_ctrl.fw_rpm(0);
    fw_rpm_2 = shoot_ctrl.fw_rpm(1);
    fw_rpm_3 = shoot_ctrl.fw_rpm(2);
    fw_rpm_4 = shoot_ctrl.fw_rpm(3);
    fw_rpm_5 = shoot_ctrl.fw_rpm(4);
    fw_rpm_6 = shoot_ctrl.fw_rpm(5);
    int16_t fric_rpm_sum = fw_rpm_1 + fw_rpm_2 + fw_rpm_3 + fw_rpm_4 + fw_rpm_5 + fw_rpm_6;
    gb_to_chassis->SetChassisFricRpm(fric_rpm_sum / kFrictionWheelCount, 0);
    imu_status = static_cast<int>(imu->online_status());

#else
    gb_to_chassis->SetChassisFricRpm(shoot_ctrl.fric_left_rpm(), shoot_ctrl.fric_right_rpm());
#endif
    gb_to_chassis->QueueSend();
  }

  yaw = imu->yaw();
  pitch = imu->pitch();
  roll = imu->roll();
}

extern "C" [[noreturn]] void AppMain(void) {
  can_to_chassis = new rm::hal::ThrottledCan<>{hcan1, 5000.0};
  can_to_chassis->SetFilter(0, 0);
  can_to_chassis->Begin();

  can_to_shoot = new rm::hal::ThrottledCan<>{hcan2, 5000.0};
  can_to_shoot->SetFilter(0, 0);
  can_to_shoot->Begin();

  imu_uart = new rm::hal::stm32::Uart(huart1, 518, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma);
  imu = new rm::device::HipnucImu(*imu_uart);

  vt03_uart = new rm::hal::Serial(huart6, 256, rm::hal::stm32::UartMode::kDma, rm::hal::stm32::UartMode::kDma);
  vt03 = new rm::device::VT03;

  // referee
  referee = new rm::device::Referee<rm::device::RefereeRevision::kNewV120>;

  vt03_uart->AttachRxCallback(Vt03RxCallback);

  gb_to_chassis = new GimbalToChassisTxBridge(*can_to_chassis, imu, vt03);
  chassis_rx = new ChassisToGimbalRxBridge(*can_to_chassis);

  // 初始化摩擦轮控制（CAN2）
  shoot_ctrl.Init(*can_to_shoot);

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
    can_to_shoot->Process();
  }
}
