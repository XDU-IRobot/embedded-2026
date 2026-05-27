#include "main.hpp"

#include "UI/TaskScheduler.hpp"
#include "UI/UIDrone.hpp"
#include "UI/UIuser1.hpp"

using namespace rm;
using namespace rm::device;

extern u16 robotID;

Gimbal* gimbal = nullptr;
Gimbal2DofDynamics drone_gb;
UITaskScheduler schedule{30};

static auto UIRobotHeaderBlueADD = UITask(UITextHeaderRobotBlue_add);
static auto UIhpBlueADD = UITask(UITextHeaderHPBlue_add);
static auto UIhpBlueEDIT = UITask(UITextHeaderHPBlue_edit, 2);
static auto UIalBlueADD = UITask(UITextHeaderAllowBlue_add);
static auto UIalBlueEDIT = UITask(UITextHeaderAllowBlue_edit, 2);

static auto UIRobotHeaderRedADD = UITask(UITextHeaderRobotRed_add);
static auto UIhpRedADD = UITask(UITextHeaderHPRed_add);
static auto UIhpRedEDIT = UITask(UITextHeaderHPRed_edit, 2);
static auto UIalRedADD = UITask(UITextHeaderAllowRed_add);
static auto UIalRedEDIT = UITask(UITextHeaderAllowRed_edit, 2);

static auto UIDroneHeroADD = UITask(UIDroneHero_add);
static auto UIDroneHeroEDIT = UITask(UIDroneHero_edit, 10);

static auto UId2h = UITask(D2H_func, 10);

void static_UI_add() {
  if (robotID > 100) {
    schedule.addTaskStatic(&UIRobotHeaderBlueADD);
    schedule.addTaskStatic(&UIalBlueADD);
    schedule.addTaskStatic(&UIhpBlueADD);
    schedule.addTask(&UIhpBlueEDIT);
    schedule.addTask(&UIalBlueEDIT);
    schedule.delTask(&UIhpRedEDIT);
    schedule.delTask(&UIalRedEDIT);
  }
  else {
    schedule.addTaskStatic(&UIRobotHeaderRedADD);
    schedule.addTaskStatic(&UIalRedADD);
    schedule.addTaskStatic(&UIhpRedADD);
    schedule.addTask(&UIhpRedEDIT);
    schedule.addTask(&UIalRedEDIT);
    schedule.delTask(&UIhpBlueEDIT);
    schedule.delTask(&UIalBlueEDIT);
  }
  schedule.addTaskStatic(&UIDroneHeroADD);
  schedule.addTask(&UIDroneHeroEDIT);
  schedule.addTask(&UId2h);
}

void GlobalLoop30Hz() {
  if (gimbal->time_ % 17 == 0) {
    schedule.schedule();
    static auto last_UI_state = gimbal->GimbalState_;
    if (last_UI_state != gimbal->GimbalState_) {
      robotID = gimbal->referee_data_buffer.data().robot_status.robot_id;
      static_UI_add();
    }
    last_UI_state = gimbal->GimbalState_;
  }
}

void MainLoop() {
  gimbal->time_++;
  gimbal->SubLoop500Hz();
  gimbal->SubLoop250Hz();
  gimbal->SubLoop100Hz();
  gimbal->SubLoop50Hz();
  GlobalLoop30Hz();
  gimbal->SubLoop10Hz();
}

extern "C" [[noreturn]] void AppMain(void) {
  gimbal = new Gimbal();
  gimbal->GimbalInit();

  // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_1000hz{&htim13, etl::delegate<void()>::create<MainLoop>()};
  mainloop_1000hz.SetPrescalerAndPeriod(168 - 1, 1000 - 1);  // 84MHz / 168 / 1000 = 500Hz
  mainloop_1000hz.Start();

  for (;;) {
    gimbal->can1->Process();
    gimbal->can2->Process();
  }
}