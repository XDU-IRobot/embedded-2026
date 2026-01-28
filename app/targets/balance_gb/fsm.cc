#include "fsm.hpp"

#include <librm.hpp>

#include "global.hpp"
#include "boardc.hpp"
#include "usart.h"

f32 left_x_debug;
f32 pitch_debug, yaw_debug, roll_debug;
f32 pitch_staus, yaw_staus;
f32 pitch_aim;
Fsm::State state;
f32 pitch_con, yaw_con, pitch_ecd, yaw_ecd;
u8 init_flag;
f32 yaw_aim;
void Fsm::Transit(State new_mode) {
  // 输入新状态
  if (new_mode != mode_) {
    global.bc->buzzer_controller.Play<modules::buzzer_melody::Success>();
    if (new_mode == State::kNoForce) {
      global.motor->DMDisable();
      global.motor->ShootDisable();
    } else if (new_mode == State::kShoot) {
      global.motor->DMEnable();
      global.motor->ShootEnable();
    } else if (new_mode == State::kAutoaim) {
      global.motor->DMEnable();
      global.motor->ShootDisable();
    } else {
      global.motor->DMEnable();
      global.motor->ShootDisable();
    }
  }
  // 替换现有状态
  mode_ = new_mode;
}

// 根据遥控器切换状态
void Fsm::Update_State() {
  left_x_debug = global.bc->rc->left_x();
  global.bc->device_rc.Update();
  if (!global.bc->device_rc.all_device_ok()) {
    Transit(State::kNoForce);
  } else {
    switch (global.bc->rc->switch_r()) {
      case DR16::SwitchPosition::kDown:
        Transit(State::kNoForce);
        break;
      case DR16::SwitchPosition::kMid:
        if (global.bc->rc->switch_l() == DR16::SwitchPosition::kMid) {
          Transit(State::kHigh);
        } else {
          Transit(State::kTest);
        }
        break;
      case DR16::SwitchPosition::kUp:
        if (global.bc->rc->switch_l() == DR16::SwitchPosition::kMid) {
          Transit(State::kShoot);

        } else if (global.bc->rc->switch_l() == DR16::SwitchPosition::kUp) {
          Transit(State::kAutoaim);
        }
        break;
      default:
        Transit(State::kNoForce);
        break;
    }
  }
}

void Fsm::Update_Test() {
  global.chassis_communicator->request_state.ChassisStateRequest = 0x01;  // 正常起立状态
  global.chassis_communicator->request_state.L0Change = 0x01;             // 正常腿长

  // 控制腿的状态
  if (global.bc->rc->right_x() > 650) {
    global.chassis_communicator->jump_flag = true;
  } else if (global.bc->rc->right_x() == -660) {
    global.chassis_communicator->request_state.L0Change = 0x00;  // 低腿长
    global.chassis_communicator->jump_flag = false;
    global.chassis_communicator->jump_count = 0;
  } else {
    if (global.chassis_communicator->jump_flag && global.bc->rc->right_x() < 650) {
      // 跳跃计时增加
      global.chassis_communicator->jump_count++;
      // 跳跃腿长控制
      if (global.chassis_communicator->jump_count < 24) {
        global.chassis_communicator->request_state.L0Change = 0x02;  // 跳跃时先下蹲
      } else if (global.chassis_communicator->jump_count < 50) {
        global.chassis_communicator->request_state.L0Change = 0x03;  // 伸腿
      } else if (global.chassis_communicator->jump_count < 95) {
        global.chassis_communicator->request_state.L0Change = 0x04;  // 收腿
      } else {
        // 重置状态
        global.chassis_communicator->jump_flag = false;
        global.chassis_communicator->jump_count = 0;
      }
    } else {
      global.chassis_communicator->request_state.L0Change = 0x01;  // 正常腿长
      global.chassis_communicator->jump_flag = false;
      global.chassis_communicator->jump_count = 0;
    }
  }

  // 判断是否小陀螺
  if (global.bc->rc->dial() == 660) {
    global.chassis_communicator->request_state.L0Change = 0x07;  // 小陀螺正转
  } else if (global.bc->rc->dial() == -660) {
    global.chassis_communicator->request_state.L0Change = 0x08;  // 小陀螺反转
  } else {
  }

  // 控制遥控器输入量
  global.chassis_communicator->request_state.ChassisMoveYRequest = global.bc->rc->left_y();
}

void Fsm::Update_Chassis_Request() {
  switch (mode_) {
    case State::kNoForce:
      global.chassis_communicator->request_state.ChassisMoveYRequest = 0;
      global.chassis_communicator->request_state.ChassisStateRequest = 0x00;
      global.chassis_communicator->request_state.L0Change = 0x01;
      break;
    case State::kTest:
      Update_Test();
      break;
    case State::kShoot:
      Update_Test();
      global.chassis_communicator->request_state.ChassisStateRequest = 0x00;  // 底盘无力
      break;
    case State::kHigh:
      Update_Test();
      global.chassis_communicator->request_state.L0Change = 0x06;  // 伸腿
      break;
    case State::kAutoaim:
      global.chassis_communicator->request_state.ChassisStateRequest = 0x00;  // 底盘无力
    default:
      global.chassis_communicator->request_state.ChassisMoveYRequest = 0;
      global.chassis_communicator->request_state.ChassisStateRequest = 0x00;
      global.chassis_communicator->request_state.L0Change = 0x01;
      break;
  }
}

//  根据状态控制电机
void Fsm::Update_Control() {
  state = mode_;
  switch (mode_) {
    case State::kNoForce:
      global.motor->CalcYawPos(global.motor->yaw_motor->pos());
      init_count_ = 0;
      global.motor->reset_yaw_flag = 0;
      break;
    case State::kTest:
      global.motor->CalcYawPos(global.motor->yaw_motor->pos());
      global.motor->Transit_initmode(
          static_cast<Motor::InitFlag>(global.chassis_receive->chassis_data_rx.GimbalInitFlag));
      switch (global.motor->init_mode) {
        case Motor::InitFlag::kNormal:
          global.motor->yaw_init = 0.f;
          if (init_count_ < 300) {
            init_count_++;
            global.motor->DMInitControl();
          } else {
            global.motor->DMControl();
          }
          break;
        case Motor::InitFlag::kOpposite:
          global.motor->yaw_init = -3.1f;
          yaw_aim = global.motor->yaw_init;
          global.motor->DMInitControl();
          init_count_ = 0;
          break;
        default:
          break;
      }
      break;
    case State::kHigh:
      global.motor->DMControl();
      break;
    case State::kShoot:
      global.motor->DMControl();
      global.motor->ShootControl();
      break;
    case State::kAutoaim:
      global.motor->DMAutoControl();
      break;
  }
}

// 500HZ任务
void Fsm::Update_500HZ() {
  yaw_ecd = global.bc->ahrs.euler_angle().yaw;
  yaw_con = global.motor->aimbot_comm->yaw();
  pitch_con = global.motor->aimbot_comm->pitch();
  pitch_ecd = global.bc->ahrs.euler_angle().pitch;
  pitch_aim = global.motor->rc_request_pitch;
  // imu更新
  global.bc->EulerUpdate();
  pitch_debug = global.bc->pitch;
  yaw_debug = global.bc->yaw;
  roll_debug = global.bc->roll;

  pitch_staus = global.motor->pitch_motor->status();
  yaw_staus = global.motor->yaw_motor->status();

  // 自瞄更新
  // global.bc->AimbotUpdate();
  if (1) {
    global.bc->imu_count++;
    global.bc->time_camera++;
    if (global.bc->time_camera == 10) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 65535);
      global.bc->time_camera = 0;
    }
    if (global.bc->time_camera == 5) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    }
  }
  if (global.bc->imu_count >= 10000) {
    global.bc->imu_count = 0;
  }
  global.motor->aimbot_comm->UpdateControl(global.bc->ahrs.quaternion().w, global.bc->ahrs.quaternion().x,
                                           global.bc->ahrs.quaternion().y, global.bc->ahrs.quaternion().z, 1, 0,
                                           global.bc->imu_count, 20.0f);
  // 状态更新
  Update_State();

  // 控制量更新
  Update_Control();

  // 发送Dji电机信息
  global.motor->SendDjiCommand();

  init_flag = global.chassis_receive->chassis_data_rx.GimbalInitFlag;
}

// 250HZ任务
void Fsm::Update_250HZ() {
  if (global.divide_count % 2 == 0) {
    // 发送DM电机数据
    global.motor->SendDMCommand();
  }
}

void Fsm::Update_100HZ() {
  if (global.divide_count % 5 == 0) {
    // 更新向底盘发送的数据
    Update_Chassis_Request();
    global.chassis_communicator->SendChassisCommand();
  }
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