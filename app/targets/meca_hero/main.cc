#include "main.hpp"
#include "timer_task.hpp"
#include "tim.h"
#include <librm.hpp>
#include "buzzer_controller.hpp"
// #include "ui.h"
// #include "ui_g.h"
// #include "UI.h"
#include "UI.hpp"
#include "queue.hpp"

rm::f32 pitch;
rm::f32 yaw;

float sum = 0;
int gyro_count = 0;
uint32_t System_time;
int power_management_gimbal_delay = 0;
int power_management_shooter_delay = 0;
int time_conut = 0;

Queue_t UI_send_buffer[2];
u32 irq;
u32 tmp_send[7];

extern u16 robot_id;
extern u8 len;
extern u8 Info_Arr[128];

// 瞄准参考线
Graph_Data  image_y;

// 飞坡、前进参考线
Graph_Data rfd1, rfd2, rfd3, rfd4;

// IMU
Float_Data Pitch;

// 弹速偏置
Float_Data AmmoSpeed;

void UiRefresh();
void UiSend();
void UI_send(rm::hal::Serial *msg, u8 *data, u8 data_len);
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
    gyro_count++;
  }
  if (gyro_count == 100) {
    average1 = sum / 100;
    sum = 0;
    gyro_count = 0;
  }
}

void SubLoop840hz() {
  heat_limit = globals->ref.data().robot_status.shooter_barrel_heat_limit;
  heat_buffer = globals->ref.data().power_heat_data.shooter_42mm_barrel_heat;
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
  } else {
    power_management_shooter_last = globals->ref.data().robot_status.power_management_shooter_output;
  }
  // 遥控器输入值
  l_switch_position_last = l_switch_position_now;
  l_switch_position_now = globals->rc->switch_l();
  r_switch_position_last = r_switch_position_now;
  r_switch_position_now = globals->rc->switch_r();
  // 底盘逻辑
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
}

void SubLoop420hz() {
  if (time_conut % 2 == 0) {
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
    // VOFA();
  }
}

void SubLoop93hz() {
  if (time_conut % 9 == 0) {
    globals->cms->SendCapBuffer(globals->ref.data().power_heat_data.buffer_energy);
  }
}

void SubLoop40hz() {
  if (time_conut % 21 == 0) {
    UiSend();
  }
}

void SubLoop10hz() {
  if (time_conut % 84 == 0) {
    globals->cms->SendCapPower(globals->ref.data().robot_status.chassis_power_limit);
    cms_v = globals->cms->cms_v;
    cms_i = globals->cms->cms_i;
    UiRefresh();
  }
}

// 定频循环
void MainLoop() {
  time_conut++;
  if (time_conut >= 10000) {
    time_conut = 0;
  }
  SubLoop840hz();
  SubLoop420hz();
  SubLoop93hz();
  SubLoop40hz();
  SubLoop10hz();
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

  // 初始化队列
  QueueInit(&UI_send_buffer[0]);
  QueueInit(&UI_send_buffer[1]);
  // ui_self_id = globals->ref.data().robot_status.robot_id;
  // ui_init_g();
  for (;;) {
    // UI();
    // ui_update_g();
  }
}

void UiRefresh() {
  // 接收机器人ID
  robot_id = globals->ref.data().robot_status.robot_id;
  if (globals->tc->data().keyboard_key & static_cast<int16_t>(device::VT03::KeyboardKey::kR) || globals->rc->key(
          device::DR16::Key::kR)
  )
  {
    Line_Draw(&image_x, "xxx", UI_Graph_ADD, 0, UI_Color_Orange, 2, 918, 515, 978, 515);
    Line_Draw(&image_y, "yyy", UI_Graph_ADD, 0, UI_Color_Orange, 2, 948, 465, 948, 565);

    Float_Draw(&super_cap_energy, "cms", UI_Graph_ADD, 2, UI_Color_Green, 27, 2, 5, 900, 270,
               static_cast<f32>(globals->supercap->voltage()) * 1000.0f);
    Float_Draw(&ammo_speed_jugde, "asj", UI_Graph_ADD, 2, UI_Color_White, 25, 2, 2, 360, 850,
               static_cast<f32>(globals->gimbal_communicator->aim_speed_change()) * 1000.0f);


    irq = (u32)&aimbot;
    EnQueue(&UI_send_buffer[1], (u8 *)&irq, 4);
    irq = (u32)&mode;
    EnQueue(&UI_send_buffer[1], (u8 *)&irq, 4);

    irq = (u32)&image_x;
    EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
    irq = (u32)&image_y;
    EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);

    irq = (u32)&super_cap_energy;
    EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
    irq = (u32)&ammo_speed_jugde;
    EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);

    irq = (u32)&get_target_flag;
    EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
    irq = (u32)&suggest_fire_flag;
    EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
    irq = (u32)&chassis_mode_flag;
    EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
    irq = (u32)&buff_mode_flag;
    EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
    irq = (u32)&speed_mode_flag;
    EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
  }
  else
  {
    // 电容电压
    if (chassis->speed_mode_ == kHighSpeed) {
      Float_Draw(&super_cap_energy, "cms", UI_Graph_Change, 2, UI_Color_Green, 27, 1, 5, 900, 270,
                 static_cast<f32>(globals->supercap->voltage()) * 1000.0f);
    } else {
      Float_Draw(&super_cap_energy, "cms", UI_Graph_Change, 2, UI_Color_Main, 27, 1, 5, 900, 270,
                 static_cast<f32>(globals->supercap->voltage()) * 1000.0f);
    }
    // 弹速调节
    if (globals->gimbal_communicator->aim_speed_change() > 0) {
      Float_Draw(&ammo_speed_jugde, "asj", UI_Graph_Change, 2, UI_Color_Green, 25, 2, 2, 360, 850,
                 static_cast<f32>(globals->gimbal_communicator->aim_speed_change()) * 1000.0f);
    } else if (globals->gimbal_communicator->aim_speed_change() < 0) {
      Float_Draw(&ammo_speed_jugde, "asj", UI_Graph_Change, 2, UI_Color_Pink, 25, 2, 2, 360, 850,
                 static_cast<f32>(globals->gimbal_communicator->aim_speed_change()) * 1000.0f);
    } else {
      Float_Draw(&ammo_speed_jugde, "asj", UI_Graph_Change, 2, UI_Color_White, 25, 2, 2, 360, 850,
                 static_cast<f32>(globals->gimbal_communicator->aim_speed_change()) * 1000.0f);
    }





    irq = (u32)&super_cap_energy;
    EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
    irq = (u32)&ammo_speed_jugde;
    EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
    irq = (u32)&get_target_flag;
    EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
    irq = (u32)&suggest_fire_flag;
    EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
    irq = (u32)&chassis_mode_flag;
    EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
    irq = (u32)&buff_mode_flag;
    EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
    irq = (u32)&speed_mode_flag;
    EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
  }
}

void UiSend() {
  if (!IsEmpty(&UI_send_buffer[0]) && globals->ui_send_choice) {
    if (UI_send_buffer[0].counter / 4 >= 7) {
      for (u8 i = 0; i < 7; i++) {
        UI_Pop(&UI_send_buffer[0], (u8 *)&tmp_send[i]);
      }
      UI_ReFresh(7, *(Graph_Data *)tmp_send[0], *(Graph_Data *)tmp_send[1], *(Graph_Data *)tmp_send[2],
                 *(Graph_Data *)tmp_send[3], *(Graph_Data *)tmp_send[4], *(Graph_Data *)tmp_send[5],
                 *(Graph_Data *)tmp_send[6]);
    } else if (UI_send_buffer[0].counter / 4 >= 5) {
      for (u8 i = 0; i < 5; i++) {
        UI_Pop(&UI_send_buffer[0], (u8 *)&tmp_send[i]);
      }
      UI_ReFresh(5, *(Graph_Data *)tmp_send[0], *(Graph_Data *)tmp_send[1], *(Graph_Data *)tmp_send[2],
                 *(Graph_Data *)tmp_send[3], *(Graph_Data *)tmp_send[4]);
    } else if (UI_send_buffer[0].counter / 4 >= 2) {
      for (u8 i = 0; i < 2; i++) {
        UI_Pop(&UI_send_buffer[0], (u8 *)&tmp_send[i]);
      }
      UI_ReFresh(2, *(Graph_Data *)tmp_send[0], *(Graph_Data *)tmp_send[1]);
    } else {
      UI_Pop(&UI_send_buffer[0], (u8 *)&tmp_send[0]);
      UI_ReFresh(1, *(Graph_Data *)tmp_send[0]);
    }
    UI_send(globals->referee_uart, Info_Arr, len);
  }
  if (!IsEmpty(&UI_send_buffer[1]) && !globals->ui_send_choice) {
    UI_Pop(&UI_send_buffer[1], (u8 *)&tmp_send[0]);
    Char_ReFresh(*(String_Data *)tmp_send[0]);
    UI_send(globals->referee_uart, Info_Arr, len);
  }
  globals->ui_send_choice ^= true;
}

void UI_send(rm::hal::Serial *msg, u8 *data, u8 data_len) { msg->Write(data, data_len); }