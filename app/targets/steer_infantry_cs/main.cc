#include <librm.hpp>

#include "can.h"
#include "usart.h"
#include "spi.h"

#include "timer_task.hpp"

#include "main.hpp"
#include "Chassis.hpp"
#include "queue.hpp"
#include "UI.hpp"

using namespace rm;

Queue_t UI_send_buffer[2];
u32 irq;
u32 tmp_send[7];
// 电容电压
Float_Data super_cap_energy, ammo_speed_jugde, remain_bullet;
// 模式
Graph_Data chassis_mode_flag, buff_mode_flag, speed_mode_flag, get_target_flag, suggest_fire_flag;
String_Data mode, aimbot;
// 瞄准线
Graph_Data image_x, image_y;

extern u16 robot_id;
extern u8 len;
extern u8 Info_Arr[128];

void UI_send(rm::hal::Serial *msg, u8 *data, u8 data_len);

void MainLoop() {
  globals->time++;
  globals->SubLoop500Hz();
  globals->SubLoop250Hz();
  globals->SubLoop100Hz();
  globals->SubLoop50Hz();
  globals->SubLoop10Hz();
}

extern "C" [[noreturn]] void AppMain(void) {
  rm::Sleep(std::chrono::milliseconds(100));  // 等待设备初始化完成
  globals = new GlobalWarehouse;
  chassis = new Chassis;
  globals->Init();
  // 初始化队列
  QueueInit(&UI_send_buffer[0]);
  QueueInit(&UI_send_buffer[1]);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_1000hz.SetPrescalerAndPeriod(168 - 1, 1000 - 1);  // 84MHz / 168 / 1000 = 500Hz
  mainloop_1000hz.Start();

  for (;;) {
    // __WFI();
  }
}

void GlobalWarehouse::Init() {
  buzzer = new Buzzer;
  led = new LED;

  can1 = new rm::hal::Can{hcan1};
  can2 = new rm::hal::Can{hcan2};
  gimbal_communicator = new rm::device::GimbalCommunicator(*can1);
  supercap = new rm::device::SuperCap(*can1);
  imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
  dbus = new rm::hal::Serial{huart3, 18, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
  referee_uart = new rm::hal::Serial{huart6, 128, hal::stm32::UartMode::kNormal, hal::stm32::UartMode::kDma};
  rx_referee = new rm::device::RxReferee{*referee_uart};

  referee_data = new rm::device::Referee<rm::device::RefereeRevision::kNewV110>;

  yaw_motor = new rm::device::GM6020{*can1, 4};

  steer_lf = new rm::device::GM6020{*can2, 1};
  steer_rf = new rm::device::GM6020{*can2, 2};
  steer_lb = new rm::device::GM6020{*can2, 4};
  steer_rb = new rm::device::GM6020{*can2, 3};
  wheel_lf = new rm::device::M3508{*can2, 1};
  wheel_rf = new rm::device::M3508{*can2, 2};
  wheel_lb = new rm::device::M3508{*can2, 4};
  wheel_rb = new rm::device::M3508{*can2, 3};

  device_chassis << steer_lf << steer_rf << steer_lb << steer_rb   // 底盘舵电机
                 << wheel_lf << wheel_rf << wheel_lb << wheel_rb;  // 底盘轮电机

  can1->SetFilter(0, 0);
  can1->Begin();
  can2->SetFilter(0, 0);
  can2->Begin();
  rx_referee->Begin();
  buzzer->Init();
  led->Init();

  led_controller.SetPattern<modules::led_pattern::GreenBreath>();
  buzzer_controller.Play<modules::buzzer_melody::Startup>();

  globals->ChassisPIDInit();
  chassis->ChassisInit();
}

void GlobalWarehouse::ChassisPIDInit() {
  chassis_controller.pid().lf_steer_position.SetKp(1000.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().lf_steer_speed.SetKp(60.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rf_steer_position.SetKp(1000.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rf_steer_speed.SetKp(60.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().lb_steer_position.SetKp(1000.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().lb_steer_speed.SetKp(60.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rb_steer_position.SetKp(1000.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rb_steer_speed.SetKp(60.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().lf_wheel.SetKp(5.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(6000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rf_wheel.SetKp(5.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(6000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().lb_wheel.SetKp(5.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(6000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rb_wheel.SetKp(5.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(6000.0f).SetMaxIout(0.0f);
}

void GlobalWarehouse::SubLoop500Hz() {
  globals->imu->Update();
  globals->ahrs.Update(rm::modules::ImuData6Dof{globals->imu->gyro_y(), globals->imu->gyro_z(),
                                                globals->imu->gyro_x() + 0.0015f, globals->imu->accel_y(),
                                                globals->imu->accel_z(), globals->imu->accel_x()});
  globals->gimbal_communicator->SendGimbalCommand(
      globals->referee_data->data().power_heat_data.shooter_17mm_1_barrel_heat,
      globals->referee_data->data().robot_status.shooter_barrel_heat_limit,
      globals->referee_data->data().robot_status.power_management_gimbal_output |
          globals->referee_data->data().robot_status.power_management_gimbal_output << 1 |
          globals->referee_data->data().robot_status.power_management_gimbal_output << 2,
      globals->referee_data->data().robot_status.robot_id);
  chassis->ChassisTask();
  rm::device::DjiMotorBase::SendCommand(*can2);
}

void GlobalWarehouse::SubLoop250Hz() {
  if (globals->time % 2 == 0) {
  }
}

void GlobalWarehouse::SubLoop100Hz() {
  if (globals->time % 5 == 0) {
    globals->device_chassis.Update();
  }
}

void GlobalWarehouse::SubLoop50Hz() {
  if (globals->time % 10 == 0) {
    const auto &[led_r, led_g, led_b] = globals->led_controller.Update();
    (*globals->led)(0xff000000 | led_r << 16 | led_g << 8 | led_b);
    buzzer->SetFrequency(globals->buzzer_controller.Update().frequency);
  }
}

void GlobalWarehouse::SubLoop10Hz() {
  if (globals->time % 50 == 0) {
    globals->time = 0;
  }
}

void UiSend() {
  // 接收机器人ID
  robot_id = globals->referee_data->data().robot_status.robot_id;
  if (globals->gimbal_communicator->UI_show_flag() == 1) {
    Line_Draw(&image_x, (char *)"x", UI_Graph_ADD, 0, UI_Color_Orange, 2, 918, 515, 978, 515);
    Line_Draw(&image_y, (char *)"y", UI_Graph_ADD, 0, UI_Color_Orange, 2, 948, 465, 948, 565);

    Float_Draw(&super_cap_energy, (char *)"cms", UI_Graph_ADD, 2, UI_Color_Green, 27, 2, 5, 900, 270,
               static_cast<f32>(globals->supercap->voltage()) * 1000.0f);
    Float_Draw(&ammo_speed_jugde, (char *)"asj", UI_Graph_ADD, 2, UI_Color_White, 25, 2, 2, 360, 850,
               static_cast<f32>(globals->gimbal_communicator->UI_show_flag()) * 1000.0f);
    Float_Draw(&remain_bullet, (char *)"rbn", UI_Graph_ADD, 2, UI_Color_White, 25, 2, 2, 1362, 475,
               static_cast<f32>(globals->remain_bullet_number) * 1000.0f);

    Rectangle_Draw(&get_target_flag, (char *)"gtf", UI_Graph_ADD, 2, UI_Color_Purplish_red, 0, 360, 800, 420, 750);
    Rectangle_Draw(&suggest_fire_flag, (char *)"sff", UI_Graph_ADD, 2, UI_Color_Purplish_red, 0, 360, 750, 420, 700);
    Rectangle_Draw(&chassis_mode_flag, (char *)"cmf", UI_Graph_ADD, 2, UI_Color_Purplish_red, 0, 6200, 730, 6350, 700);
    Rectangle_Draw(&buff_mode_flag, (char *)"bmf", UI_Graph_ADD, 2, UI_Color_Purplish_red, 0, 6200, 730, 6350, 700);
    Rectangle_Draw(&speed_mode_flag, (char *)"smf", UI_Graph_ADD, 2, UI_Color_Purplish_red, 0, 6200, 770, 6350, 730);

    Char_Draw(&aimbot, (char *)"aim", UI_Graph_ADD, 1, UI_Color_Green, 25, 22, 2, 360, 800,
              (char *)"GETTARGET\nSUGGESTFIRE");
    Char_Draw(&mode, (char *)"mod", UI_Graph_ADD, 1, UI_Color_Green, 25, 20, 2, 1300, 800,
              (char *)"F R N U D X\nH N S");

    irq = (u32)&aimbot;
    EnQueue(&UI_send_buffer[1], (u8 *)&irq, 4);
    irq = (u32)&mode;
    EnQueue(&UI_send_buffer[1], (u8 *)&irq, 4);
    irq = (u32)&image_x;
    EnQueue(&UI_send_buffer[1], (u8 *)&irq, 4);
    irq = (u32)&image_y;
    EnQueue(&UI_send_buffer[1], (u8 *)&irq, 4);
    irq = (u32)&super_cap_energy;
    EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
    irq = (u32)&ammo_speed_jugde;
    EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
    irq = (u32)&remain_bullet;
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

    while (!IsEmpty(&UI_send_buffer[0])) {
      if (UI_send_buffer[0].counter / 4 >= 7) {
        for (unsigned long &i : tmp_send) {
          UI_Pop(&UI_send_buffer[0], (u8 *)&i);
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

    while (!IsEmpty(&UI_send_buffer[1])) {
      UI_Pop(&UI_send_buffer[1], (u8 *)&tmp_send[0]);
      Char_ReFresh(*(String_Data *)tmp_send[0]);

      UI_send(globals->referee_uart, Info_Arr, len);
    }
  } else {
    // 电容电压
    if (chassis->speed_mode_ == kHighSpeed) {
      Float_Draw(&super_cap_energy, (char *)"cms", UI_Graph_Change, 2, UI_Color_Green, 27, 1, 5, 900, 270,
                 static_cast<f32>(globals->supercap->voltage()) * 1000.0f);
    } else {
      Float_Draw(&super_cap_energy, (char *)"cms", UI_Graph_Change, 2, UI_Color_Main, 27, 1, 5, 900, 270,
                 static_cast<f32>(globals->supercap->voltage()) * 1000.0f);
    }
    // 弹速调节
    if (globals->gimbal_communicator->UI_show_flag() > 0) {
      Float_Draw(&ammo_speed_jugde, (char *)"asj", UI_Graph_Change, 2, UI_Color_Green, 25, 2, 2, 360, 850,
                 static_cast<f32>(globals->gimbal_communicator->UI_show_flag()) * 1000.0f);
    } else if (globals->gimbal_communicator->UI_show_flag() < 0) {
      Float_Draw(&ammo_speed_jugde, (char *)"asj", UI_Graph_Change, 2, UI_Color_Pink, 25, 2, 2, 360, 850,
                 static_cast<f32>(globals->gimbal_communicator->UI_show_flag()) * 1000.0f);
    } else {
      Float_Draw(&ammo_speed_jugde, (char *)"asj", UI_Graph_Change, 2, UI_Color_White, 25, 2, 2, 360, 850,
                 static_cast<f32>(globals->gimbal_communicator->UI_show_flag()) * 1000.0f);
    }
    // 剩余子弹
    if (globals->remain_bullet_number < 0) {
      Float_Draw(&remain_bullet, (char *)"rbn", UI_Graph_Change, 2, UI_Color_Pink, 25, 2, 2, 1362, 475,
                 static_cast<f32>(globals->remain_bullet_number) * 1000.0f);
    } else if (globals->remain_bullet_number < 100) {
      Float_Draw(&remain_bullet, (char *)"rbn", UI_Graph_Change, 2, UI_Color_Orange, 25, 2, 2, 1362, 475,
                 static_cast<f32>(globals->remain_bullet_number) * 1000.0f);
    } else {
      Float_Draw(&remain_bullet, (char *)"rbn", UI_Graph_Change, 2, UI_Color_Green, 25, 2, 2, 1362, 475,
                 static_cast<f32>(globals->remain_bullet_number) * 1000.0f);
    }
    // // 自瞄模式
    if (globals->gimbal_communicator->get_target_flag() == 1) {
      Rectangle_Draw(&get_target_flag, (char *)"gtf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 350, 807, 580, 768);
    } else {
      Rectangle_Draw(&get_target_flag, (char *)"gtf", UI_Graph_Change, 2, UI_Color_Purplish_red, 0, 350, 807, 580, 768);
    }

    if (globals->gimbal_communicator->suggest_fire_flag() == 1) {
      Rectangle_Draw(&suggest_fire_flag, (char *)"sff", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 350, 766, 630,
                     731);
    } else {
      Rectangle_Draw(&suggest_fire_flag, (char *)"sff", UI_Graph_Change, 2, UI_Color_Purplish_red, 0, 350, 766, 630,
                     731);
    }

    // 底盘模式
    if (chassis->ChassisMove_ == kFollow) {
      Rectangle_Draw(&chassis_mode_flag, (char *)"cmf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1292, 808, 1322,
                     770);
    } else if (chassis->ChassisMove_ == kRotate) {
      Rectangle_Draw(&chassis_mode_flag, (char *)"cmf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1342, 808, 1372,
                     770);
    } else if (chassis->ChassisMove_ == kNoForce) {
      Rectangle_Draw(&chassis_mode_flag, (char *)"cmf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1392, 808, 1422,
                     770);
    } else {
      Rectangle_Draw(&chassis_mode_flag, (char *)"cmf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1492, 808, 1522,
                     770);
    }
    if (chassis->buff_state_ == kDaFu) {
      Rectangle_Draw(&buff_mode_flag, (char *)"bmf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1542, 808, 1572,
                     770);
    } else if (chassis->buff_state_ == kXiaoFu) {
      Rectangle_Draw(&buff_mode_flag, (char *)"bmf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1592, 808, 1622,
                     770);
    } else {
      Rectangle_Draw(&buff_mode_flag, (char *)"bmf", UI_Graph_Change, 2, UI_Color_Purplish_red, 0, 1542, 808, 1572,
                     770);
    }

    // 底盘速度模式
    if (chassis->speed_mode_ == kHighSpeed) {
      Rectangle_Draw(&speed_mode_flag, (char *)"smf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1292, 766, 1322,
                     728);
    } else {
      Rectangle_Draw(&speed_mode_flag, (char *)"smf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1342, 766, 1372,
                     728);
    }
    if (globals->referee_data->data().power_heat_data.buffer_energy < 40) {
      Rectangle_Draw(&speed_mode_flag, (char *)"smf", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1392, 766, 1422,
                     728);
    }

    irq = (u32)&super_cap_energy;
    EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
    irq = (u32)&ammo_speed_jugde;
    EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
    irq = (u32)&remain_bullet;
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

    while (!IsEmpty(&UI_send_buffer[0])) {
      if (UI_send_buffer[0].counter / 4 >= 7) {
        for (unsigned long &i : tmp_send) {
          UI_Pop(&UI_send_buffer[0], (u8 *)&i);
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

    while (!IsEmpty(&UI_send_buffer[1])) {
      UI_Pop(&UI_send_buffer[1], (u8 *)&tmp_send[0]);
      Char_ReFresh(*(String_Data *)tmp_send[0]);
      UI_send(globals->referee_uart, Info_Arr, len);
    }
  }
}

void UI_send(rm::hal::Serial *msg, u8 *data, u8 data_len) { msg->Write(data, data_len); }