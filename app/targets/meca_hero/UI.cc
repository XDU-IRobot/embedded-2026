#include "UI.h"
#include "Cilent_UI.h"
#include "MyQueue.h"

#include <cmath>
#include <string>

#include "usart.h"

#include <stdint.h>
#include "librm.hpp"
#include "main.hpp"

using namespace rm::hal;
using namespace rm::hal::stm32;
using namespace rm::device;

Queue_t UI_send[2];
uint32_t irq;
uint32_t tmp_send[7];
static uint8_t i = 0;

extern uint8_t len;
extern uint8_t Info_Arr[128];

// 瞄准参考线
Graph_Data imagex, imagey;
Graph_Data x1, x2, x3, x4, x5, x6, x7, x8, x9, x10;
Graph_Data ry1, ry2, ry3, ry4, ry5, ry6;

// 飞坡、前进参考线
Graph_Data rfd1, rfd2, rfd3, rfd4;

// IMU
Float_Data Pitch;

Graph_Data p1, p2, p3, p4, p5, p6;
Graph_Data op1, op2, op3, op4, op5;

// 底盘状态，弹速偏置
String_Data Mode;
Float_Data AmmoSpeed;
Float_Data AmmoCount;

// 自瞄目标
// String_Data aimbot; // 自瞄状态
Float_Data outpost_w;  // 自瞄前哨站转向

// yaw轴夹角
Float_Data Yaw;
Graph_Data Yaw_G;

// 电容电压
Float_Data CapData;

void UIsend(Serial uartx, uint8_t *data, uint8_t len) { uartx.Write(data, len); }

void UI(void) {
  QueueInit(&UI_send[0]);
  QueueInit(&UI_send[1]);

  while (1) {
    if (globals->tc->data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kZ)) {
      // robot_id = referee.data().robot_status.robot_id;
      // 固定UI
      Line_Draw(&imagey, "yck", UI_Graph_ADD, 0, UI_Color_Green, 1, 960, 900, 960, 200);  // 中心瞄准线

      Line_Draw(&x1, "x01", UI_Graph_ADD, 0, UI_Color_White, 1, 900, 360, 1020, 360);
      Line_Draw(&x2, "x02", UI_Graph_ADD, 0, UI_Color_Orange, 1, 900, 480, 1020, 480);
      Line_Draw(&x3, "x03", UI_Graph_ADD, 0, UI_Color_Cyan, 1, 900, 510, 1020, 510);
      Line_Draw(&x4, "x04", UI_Graph_ADD, 0, UI_Color_Orange, 2, 920, 398, 960, 398);
      Line_Draw(&x5, "x05", UI_Graph_ADD, 0, UI_Color_Purplish_red, 2, 890, 385, 950, 385);  // X轴瞄准线

      Line_Draw(&ry1, "ry1", UI_Graph_ADD, 1, UI_Color_Orange, 2, 934, 388, 934, 414);
      // Line_Draw(&ry2, "ry2", UI_Graph_ADD, 1, UI_Color_Orange, 1, 940, 340, 940, 520);
      Line_Draw(&ry3, "ry3", UI_Graph_ADD, 1, UI_Color_Purplish_red, 2, 920, 360, 920, 440);
      // Line_Draw(&ry4, "ry4", UI_Graph_ADD, 1, UI_Color_Cyan, 1, 900, 380, 900, 480);  // Y轴瞄准线

      Line_Draw(&rfd1, "fd1", UI_Graph_ADD, 1, UI_Color_Cyan, 3, 454, 166, 690, 432);
      Line_Draw(&rfd2, "fd2", UI_Graph_ADD, 1, UI_Color_Cyan, 3, 1451, 149, 1192, 433);  // 车边缘轨道

      // Char_Draw(&aimbot, "aim", UI_Graph_ADD, 1, UI_Color_Green, 25, 15, 2, 360, 800, "AIMBOT\nAUTOFIRE");
      Char_Draw(&Mode, "mod", UI_Graph_ADD, 1, UI_Color_Green, 25, 15, 2, 1300, 800, "R F N U\nL N H P");  // 字符提示

      // Float_Draw(&CapData, "cad", UI_Graph_ADD, 1, UI_Color_Green, 27, 2, 5, 7050, 150, (float)cms.cms_v * 1000);
      Float_Draw(&Pitch, "gbp", UI_Graph_ADD, 1, UI_Color_Green, 27, 1, 3, 7300, 500,
                 (float)-globals->ahrs.euler_angle().pitch * 1000);
      // Float_Draw(&AmmoSpeed, "AmmoSpeed", UI_Graph_ADD, 1, UI_Color_Purplish_red, 27, 1, 3, 7300, 560,
      //            (float)(-globals->ahrs.euler_angle().pitch * 1000));
      Float_Draw(&AmmoCount, "amc", UI_Graph_ADD, 1, UI_Color_Orange, 27, 1, 3, 7300, 620, (float)0 * 1000);
      Arc_Draw(&Yaw_G, "Yaw_G", UI_Graph_ADD, 2, UI_Color_Green, 0, 30, 2, 960, 540, 300, 300);
      Float_Draw(&outpost_w, "outpost_w", UI_Graph_ADD, 2, UI_Color_Orange, 27, 2, 3, 7300, 440, 1);
      // Float_Draw(&Yaw, "yaw", UI_Graph_ADD, 1, UI_Color_Green, 27, 1, 3, 7300, 440,
      //            (float)communication.command_.ui.ui2 * 90 * 1000);

      Rectangle_Draw(&p1, "p01", UI_Graph_ADD, 2, UI_Color_Purplish_red, 0, 360, 800, 420, 750);
      Rectangle_Draw(&p2, "p02", UI_Graph_ADD, 2, UI_Color_Purplish_red, 0, 360, 750, 420, 700);
      Rectangle_Draw(&p3, "p03", UI_Graph_ADD, 2, UI_Color_Purplish_red, 0, 6200, 730, 6350, 700);
      Rectangle_Draw(&p4, "p04", UI_Graph_ADD, 2, UI_Color_Purplish_red, 1, 6200, 770, 6350, 730);

      irq = (uint32_t)&imagey;
      EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
      irq = (uint32_t)&x1;
      EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
      irq = (uint32_t)&x2;
      EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
      irq = (uint32_t)&x4;
      EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
      irq = (uint32_t)&x5;
      EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
      irq = (uint32_t)&rfd1;
      EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
      irq = (uint32_t)&rfd2;
      EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
      irq = (uint32_t)&ry1;
      EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
      // irq = (uint32_t)&ry2;
      // EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
      irq = (uint32_t)&ry3;
      EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
      // irq = (uint32_t)&ry4;
      // EnQueue(&UI_send[0], (uint8_t *)&irq, 4);

      // irq = (uint32_t)&aimbot;

      irq = (uint32_t)&Mode;
      EnQueue(&UI_send[1], (uint8_t *)&irq, 4);

      irq = (uint32_t)&Yaw_G;
      EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
      irq = (uint32_t)&AmmoCount;
      EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
      irq = (uint32_t)&outpost_w;
      EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
      irq = (uint32_t)&AmmoSpeed;
      EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
      irq = (uint32_t)&CapData;
      EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
      irq = (uint32_t)&Pitch;
      EnQueue(&UI_send[0], (uint8_t *)&irq, 4);

      irq = (uint32_t)&p1;
      EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
      irq = (uint32_t)&p2;
      EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
      irq = (uint32_t)&p3;
      EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
      irq = (uint32_t)&p4;
      EnQueue(&UI_send[0], (uint8_t *)&irq, 4);

      while (!IsEmpty(&UI_send[0])) {
        if (UI_send[0].counter / 4 >= 7) {
          for (i = 0; i < 7; i++) {
            UI_Pop(&UI_send[0], (uint8_t *)&tmp_send[i]);
          }
          UI_ReFresh(7, *(Graph_Data *)tmp_send[0], *(Graph_Data *)tmp_send[1], *(Graph_Data *)tmp_send[2],
                     *(Graph_Data *)tmp_send[3], *(Graph_Data *)tmp_send[4], *(Graph_Data *)tmp_send[5],
                     *(Graph_Data *)tmp_send[6]);
        } else if (UI_send[0].counter / 4 >= 5) {
          for (i = 0; i < 5; i++) {
            UI_Pop(&UI_send[0], (uint8_t *)&tmp_send[i]);
          }
          UI_ReFresh(5, *(Graph_Data *)tmp_send[0], *(Graph_Data *)tmp_send[1], *(Graph_Data *)tmp_send[2],
                     *(Graph_Data *)tmp_send[3], *(Graph_Data *)tmp_send[4]);
        } else if (UI_send[0].counter / 4 >= 2) {
          for (i = 0; i < 2; i++) {
            UI_Pop(&UI_send[0], (uint8_t *)&tmp_send[i]);
          }
          UI_ReFresh(2, *(Graph_Data *)tmp_send[0], *(Graph_Data *)tmp_send[1]);
        } else {
          UI_Pop(&UI_send[0], (uint8_t *)&tmp_send[0]);
          UI_ReFresh(1, *(Graph_Data *)tmp_send[0]);
        }

        UIsend(*globals->uart6, Info_Arr, len);

        HAL_Delay(50);
      }

      while (!IsEmpty(&UI_send[1])) {
        UI_Pop(&UI_send[1], (uint8_t *)&tmp_send[0]);
        Char_ReFresh(*(String_Data *)tmp_send[0]);

        UIsend(*globals->uart1, Info_Arr, len);
        HAL_Delay(50);
      }
    }
  }
}

//     else {
//       // 电容电压
//       if (chassis.PowerControl == true) {
//         Float_Draw(&CapData, "cad", UI_Graph_Change, 1, UI_Color_Green, 27, 1, 5, 900, 150, (float)cms.cms_v * 1000);
//       } else {
//         Float_Draw(&CapData, "cad", UI_Graph_Change, 1, UI_Color_Main, 27, 1, 5, 900, 150, (float)cms.cms_v * 1000);
//       }
//
//       // // yaw轴转动偏置
//       // if (0) {
//       //   Float_Draw(&Yaw, "yaw", UI_Graph_Change, 1, UI_Color_Green, 27, 1, 3, 7300, 440, (float)0 * 90 * 1000);
//       // } else {
//       //   Float_Draw(&Yaw, "yaw", UI_Graph_Change, 1, UI_Color_Pink, 27, 1, 3, 7300, 440, (float)0 * 90 * 1000);
//       // }
//
//       // 前哨站转向
//       if ((communication.command_.ui.ui1 >> 4) & 0x01)
//         Float_Draw(&outpost_w, "outpost_w", UI_Graph_Change, 2, UI_Color_Cyan, 27, 2, 3, 7300, 440, 1000);
//       else
//         Float_Draw(&outpost_w, "outpost_w", UI_Graph_Change, 2, UI_Color_Cyan, 27, 2, 3, 7300, 440, 0);
//
//       // pitch轴IMU
//       Float_Draw(&Pitch, "gbp", UI_Graph_Change, 1, UI_Color_Green, 27, 1, 3, 7300, 500,
//                  (float)IntToFloat(communication.command_.ui.ui3, -90, 90, 8) * 1000);
//
//       // 底盘夹角
//       Arc_Draw(&Yaw_G, "Yaw_G", UI_Graph_Change, 2, UI_Color_Green, LoopConstrain(-chassis.del - 15, 0, 360),
//                LoopConstrain(-chassis.del + 15, 0, 360), 2, 960, 540, 300, 300);
//
//       // 弹速增益
//       Float_Draw(&AmmoSpeed, "AmmoSpeed", UI_Graph_Change, 1, UI_Color_Purplish_red, 27, 1, 3, 7300, 560,
//                  (float)((communication.command_.ui.ui2 - 0x1F) * 1000));
//
//       // 总发弹统计
//       Float_Draw(&AmmoCount, "amc", UI_Graph_Change, 1, UI_Color_Orange, 27, 1, 3, 7300, 620, (float)ammo_count *
//       1000);
//
//       // 自瞄模式
//       if ((communication.command_.ui.ui1 >> 1) & 0x01) {
//         Rectangle_Draw(&p1, "p01", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 350, 809, 513, 769);
//       } else {
//         Rectangle_Draw(&p1, "p01", UI_Graph_Change, 2, UI_Color_Purplish_red, 0, 350, 809, 513, 769);
//       }
//
//       if ((communication.command_.ui.ui1 >> 2) & 0x01) {
//         Rectangle_Draw(&p2, "p02", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 350, 765, 555, 730);
//       } else {
//         Rectangle_Draw(&p2, "p02", UI_Graph_Change, 2, UI_Color_Purplish_red, 0, 350, 765, 555, 730);
//       }
//
//       // 底盘模式
//       if (communication.command_.chassis.state == ChassisState::ROTATE) {
//         Rectangle_Draw(&p3, "p03", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1295, 810, 1325, 765);
//       } else if (communication.command_.chassis.state == ChassisState::FOLLOW) {
//         Rectangle_Draw(&p3, "p03", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1340, 810, 1370, 765);
//       } else if (communication.command_.chassis.state == ChassisState::NOMOVE) {
//         Rectangle_Draw(&p3, "p03", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1390, 810, 1420, 765);
//       } else if (communication.command_.chassis.state == ChassisState::UNABLE) {
//         Rectangle_Draw(&p3, "p03", UI_Graph_Change, 2, UI_Color_Purplish_red, 3, 1442, 810, 1472, 765);
//       }
//
//       // 底盘速度
//       if (communication.command_.chassis.speed == ChassisSpeed::SLOW) {
//         Rectangle_Draw(&p4, "p04", UI_Graph_Change, 2, UI_Color_White, 3, 1295, 760, 1325, 710);
//       } else if (communication.command_.chassis.speed == ChassisSpeed::NORMAL) {
//         Rectangle_Draw(&p4, "p04", UI_Graph_Change, 2, UI_Color_White, 3, 1340, 760, 1370, 710);
//       } else if (communication.command_.chassis.speed == ChassisSpeed::FAST) {
//         Rectangle_Draw(&p4, "p04", UI_Graph_Change, 2, UI_Color_White, 3, 1390, 760, 1420, 710);
//       } else if (communication.command_.chassis.speed == ChassisSpeed::HIGHSPEED) {
//         Rectangle_Draw(&p4, "p04", UI_Graph_Change, 2, UI_Color_White, 3, 1442, 760, 1472, 710);
//       }
//
//       irq = (uint32_t)&outpost_w;
//       EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
//       irq = (uint32_t)&Yaw_G;
//       EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
//       irq = (uint32_t)&CapData;
//       UI_EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
//       // irq = (uint32_t)&Yaw;
//       // EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
//       irq = (uint32_t)&Pitch;
//       EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
//       irq = (uint32_t)&AmmoSpeed;
//       EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
//       irq = (uint32_t)&AmmoCount;
//       EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
//
//       irq = (uint32_t)&p1;
//       EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
//       irq = (uint32_t)&p2;
//       EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
//       irq = (uint32_t)&p3;
//       EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
//       irq = (uint32_t)&p4;
//       EnQueue(&UI_send[0], (uint8_t *)&irq, 4);
//
//       while (!IsEmpty(&UI_send[0])) {
//         if (UI_send[0].counter / 4 >= 7) {
//           for (i = 0; i < 7; i++) {
//             UI_Pop(&UI_send[0], (uint8_t *)&tmp_send[i]);
//           }
//           UI_ReFresh(7, *(Graph_Data *)tmp_send[0], *(Graph_Data *)tmp_send[1], *(Graph_Data *)tmp_send[2],
//                      *(Graph_Data *)tmp_send[3], *(Graph_Data *)tmp_send[4], *(Graph_Data *)tmp_send[5],
//                      *(Graph_Data *)tmp_send[6]);
//         } else if (UI_send[0].counter / 4 >= 5) {
//           for (i = 0; i < 5; i++) {
//             UI_Pop(&UI_send[0], (uint8_t *)&tmp_send[i]);
//           }
//           UI_ReFresh(5, *(Graph_Data *)tmp_send[0], *(Graph_Data *)tmp_send[1], *(Graph_Data *)tmp_send[2],
//                      *(Graph_Data *)tmp_send[3], *(Graph_Data *)tmp_send[4]);
//         } else if (UI_send[0].counter / 4 >= 2) {
//           for (i = 0; i < 2; i++) {
//             UI_Pop(&UI_send[0], (uint8_t *)&tmp_send[i]);
//           }
//           UI_ReFresh(2, *(Graph_Data *)tmp_send[0], *(Graph_Data *)tmp_send[1]);
//         } else {
//           UI_Pop(&UI_send[0], (uint8_t *)&tmp_send[0]);
//           UI_ReFresh(1, *(Graph_Data *)tmp_send[0]);
//         }
//         UIsend(refereeuart, Info_Arr, len);
//         HAL_Delay(50);
//       }
//
//       while (!IsEmpty(&UI_send[1])) {
//         UI_Pop(&UI_send[1], (uint8_t *)&tmp_send[0]);
//         Char_ReFresh(*(String_Data *)tmp_send[0]);
//         UIsend(refereeuart, Info_Arr, len);
//         HAL_Delay(50);
//       }
//     }
//   }
// }

// void UIsend(uint8_t *data, uint8_t len) { usart6_tx_dma_enable(data, len); }

// void UI(void const *argument) {
//   portTickType UI_task_pre_tick = 0;
//   while (1) {
//     osDelayUntil(&UI_task_pre_tick, 100);
//   }
// }