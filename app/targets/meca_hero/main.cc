#include "main.hpp"
#include "timer_task.hpp"
#include "tim.h"
#include <librm.hpp>
#include "buzzer_controller.hpp"
rm::f32 pitch;
rm::f32 yaw;

float sum = 0;
int count = 0;
int autoaim_update_count = 0;
uint32_t System_time;
int power_management_gimbal_delay = 0;
int power_management_shooter_delay = 0;

// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
//   if (huart->Instance == USART1) {
//     // 1. 这一步至关重要：手动添加字符串结束符
//     // Size 是硬件告诉你的实际收到的字节数
//     rx_buffer[Size] = '\0';
//
//     // 2. 解析逻辑
//     char type = rx_buffer[0];
//     // 跳过第一个字母，直接解析后面的数字
//     float val = atof(&rx_buffer[1]);
//
//     // 根据首字母更新 PID 参数
//     switch (type) {
//       case 'P': globals->gimbal->pos_pid.kp = val; break;
//       case 'v': globals->gimbal->vel_pid.kp = val; break;
//       case 'i': globals->gimbal->vel_pid.ki = val; break;
//       case 'f': globals->gimbal->pitch_kg = val;   break;
//     }
//
//     // 3. 重点：处理完后必须重新开启接收
//     HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)rx_buffer, 64);
//   }
// }

void G_average() {
  if (globals->gyro_z_filter.apply(globals->imu->gyro_z()) < 0.01 /*- 0.03 * eulerangle_pitch / 0.6644*/ &&
      globals->gyro_z_filter.apply(globals->imu->gyro_z()) > -0.01 /*- 0.03 * eulerangle_pitch / 0.6644*/) {
    sum += globals->imu->gyro_z();
    count++;
  }
  if (count == 100) {
    average1 = sum / 100;
    sum = 0;
    count = 0;
  }
}

// 定频循环
void MainLoop() {
  if (globals->ref.data().robot_status.power_management_gimbal_output == 1 && power_management_gimbal_last == 0) {
    power_management_gimbal_delay++;
    if (power_management_gimbal_delay > 840 * 3) {
      power_management_gimbal_last = 1;
      power_management_gimbal_delay = 0;
    }
  } else {
    power_management_gimbal_last = globals->ref.data().robot_status.power_management_gimbal_output;
  }
  if (globals->ref.data().robot_status.power_management_shooter_output == 1 && power_management_shooter_last == 0) {
    power_management_shooter_delay++;
    if (power_management_shooter_delay > 840 * 3) {
      power_management_shooter_last = 1;
      power_management_shooter_delay = 0;
    }
  }else {
    power_management_shooter_last=globals->ref.data().robot_status.power_management_shooter_output;
  }
  // 遥控器输入值
  l_switch_position_last = l_switch_position_now;
  l_switch_position_now = globals->rc->switch_l();
  r_switch_position_last = r_switch_position_now;
  r_switch_position_now = globals->rc->switch_r();
  // 底盘逻辑
  // ChassisControl();
  ChassisPower();
  // 摩擦轮电机逻辑
  ShooterControl();
  // 拨盘电机逻辑
  MagazineControl();
  G_average();
  // 云台控制逻辑
  GimbalControl();
  // 发送DjiCAN信号
  rm::device::DjiMotorBase::SendCommand();
  if (autoaim_update_count == 1) {
    CANAutoaimUpdate();
    aimbot_target = globals->aimbot_can_communicator->aimbot_target();
    aimbot_state = globals->aimbot_can_communicator->aimbot_state();
    // 改usb中断处字长检查
    // AutoaimUpdate();
    CustomClientUpdate();

    key_w = globals->custom_client->key(rm::device::DR16::Key::kW);
    cc_mouse_l = globals->custom_client->mouse_right();
    cc_mouse_x = globals->rc->mouse_x();
    cc_mouse_y = globals->rc->mouse_y();
    key_a = globals->custom_client->key(rm::device::DR16::Key::kA);
    key_d = globals->custom_client->key(rm::device::DR16::Key::kD);
    key_e = globals->custom_client->key(rm::device::DR16::Key::kE);
    VOFA();
    autoaim_update_count = 0;
  } else {
    autoaim_update_count++;
  }
}

extern "C" [[noreturn]] void AppMain(void) {
  /*启动CAN总线
   *启动遥控器
   */
  globals = new GlobalWarehouse;
  globals->Init();
  // // 启动 DMA 接收到空闲中断
  // // rx_buffer 建议开大一点，比如 64 字节
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)rx_buffer, 64);
  // 启动 DMA 接收
  rm::hal::SerialRxCallbackFunction ref_rx_callback = [&](const std::vector<uint8_t> &data, uint16_t len) {
    for (int i = 0; i < len; i++) {
      globals->ref << data[i];
    }
  };
  globals->uart6->AttachRxCallback(ref_rx_callback);
  globals->uart6->Begin();

  rm::hal::SerialRxCallbackFunction tc_rx_callback = [&](const std::vector<uint8_t> &data, uint16_t len) {
    for (int i = 0; i < len; i++) {
      globals->tc->operator<<(data[i]);
    }
  };
  globals->uart1->AttachRxCallback(tc_rx_callback);
  globals->uart1->Begin();
  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim13,
      etl::delegate<void()>::create<MainLoop>() //
  };
  mainloop_1000hz.SetPrescalerAndPeriod(100 - 1, 1000 - 1); // 84MHz / 100 / 1000 = 840Hz
  mainloop_1000hz.Start(); // 启动定时器
  globals->gyro_z_filter.set_cutoff_frequency(1000.0f, 50.0f);
  for (;;) {
    __WFI();
  }
}