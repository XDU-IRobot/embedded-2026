#include "fsm.hpp"

#include <librm.hpp>

#include "global.hpp"
#include "boardc.hpp"
#include "usart.h"

f32 debug;
f32 left_x_debug;
f32 pitch_debug,yaw_debug,roll_debug;
f32 pitch_staus,yaw_staus;
void Fsm::Transit(State new_mode) {
  // 输入新状态
  if (new_mode != mode_) {
    global.bc->buzzer_controller.Play<modules::buzzer_melody::Success>();
    if (new_mode == State::kNoForce) {
      global.motor->DMDisable();
      global.motor->ShootDisable();
    } else {
      global.motor->DMEnable();
      //global.motor->ShootEnable(true);
    }
  }
  // 替换现有状态
  mode_ = new_mode;
}

// 根据遥控器切换状态
void Fsm::Update_State() {
  left_x_debug = global.bc->rc->left_x();

  if (!global.bc->device_rc.all_device_ok()) {
    Transit(State::kNoForce);
  } else {
    switch (global.bc->rc->switch_r()) {
      case DR16::SwitchPosition::kDown:
        Transit(State::kNoForce);
        break;
      case DR16::SwitchPosition::kMid:
        Transit(State::kTest);
        break;
      case DR16::SwitchPosition::kUp:
        Transit(State::kShoot);
        break;
      default:
        Transit(State::kNoForce);
        break;
    }
  }
}

//  根据状态控制电机
void Fsm::Update_Control() {
  switch (mode_) {
    case State::kNoForce:
      init_count_ = 0;
      break;
    case State::kTest:
      if (init_count_ < 300) {
        init_count_++;
        global.motor->DMInitControl();
      }else {
        global.motor->DMControl();
      }
    case State::kShoot:
      global.motor->DMControl();
      global.motor->ShootControl();
  }
}

// 500HZ任务
void Fsm::Update_500HZ() {
  debug = global.motor->gimbal_controller.output().pitch;
  //imu更新
  global.bc->EulerUpdate();
  pitch_debug = global.bc->pitch;
  yaw_debug = global.bc->yaw;
  roll_debug = global.bc->roll;

  pitch_staus = global.motor->pitch_motor->status();
  yaw_staus = global.motor->yaw_motor->status();

  //状态更新
  Update_State();

  //控制量更新
  Update_Control();

  //发送Dji电机信息
  global.motor->SendDjiCommand();
}

  //250HZ任务
void Fsm::Update_250HZ() {
  if (global.divide_count % 2 ==0) {
    //发送DM电机数据
    global.motor->SendDMCommand();
  }
}

void Fsm::Update_100HZ() {

}


//  25HZ任务
void Fsm::Update_25HZ() {
  if (global.divide_count % 20 == 0) {
    const auto &[led_r, led_g, led_b] = global.bc->led_controller.Update();
    (*global.bc->led)(0xff000000 | led_r << 16 | led_g << 8 | led_b);
    global.bc->buzzer->SetFrequency(global.bc->buzzer_controller.Update().frequency);
  }
}

//  10HZ任务
void Fsm::Update_10HZ() {
  if (global.divide_count % 50 == 0) {
    global.divide_count = 0;
  }
}