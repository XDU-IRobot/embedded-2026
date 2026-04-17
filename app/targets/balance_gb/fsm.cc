#include "fsm.hpp"

#include <librm.hpp>

#include "global.hpp"
#include "boardc.hpp"
#include "usart.h"

f32 left_x_debug;
f32 pitch_debug, yaw_debug, roll_debug;
u8 pitch_staus, yaw_staus;
f32 pitch_aim;
Fsm::State state;
f32 pitch_con, yaw_con, pitch_ecd, yaw_ecd;
u8 init_flag;
u8 RcKey_KB_flag = 0, RcKey_KB_value = 0, TCRcKey_KB_flag = 0;
u8 RcKey_KQ_flag = 0, RcKey_KQ_value = 0, TCRcKey_KQ_flag = 0;
u8 TCRcKey_KF_flag = 0, TCRcKey_KF_value = 0;
u8 TCRcKey_KG_flag = 0, TCRcKey_KG_value = 0;
u8 TCRcKey_KE_flag = 0, TCRcKey_KE_value = 0;
f32 yaw_aim;
f32 debug_w, debug_x, debug_y, debug_z;
u8 uiflag, robot_id;
bool w;
u8 gimbal_flag;
int enable_count = 0;  // 使能延时
i16 init_count = 0;

extern VT03 tcremote;

void Fsm::Transit(State new_mode) {
  pitch_staus = global.motor->pitch_motor->status();
  yaw_staus = global.motor->yaw_motor->status();
  // 输入新状态
  if (new_mode != mode_) {
    global.bc->buzzer_controller.Play<modules::buzzer_melody::Success>();
    if (new_mode == State::kNoForce) {
      enable_count = 0;
      global.motor->DMDisable();
      global.motor->ShootDisable();
    } else if (new_mode == State::kTest) {
      // global.motor->DMEnable();
      global.motor->ShootDisable();
    } else if (new_mode == State::kShoot) {
      global.motor->DMEnable();
      global.motor->ShootDisable();
    } else if (new_mode == State::kAutoFu) {
      global.motor->DMEnable();
      global.motor->ShootEnable();
    }else if (new_mode == State::kHigh) {
      global.motor->DMEnable();
      global.motor->ShootEnable();
    } else if (new_mode == State::kAutoShoot) {
      // global.motor->DMEnable();
      global.motor->ShootEnable();
    } else {
      global.motor->DMDisable();
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
  if (global.chassis_rx->chassis_data_rx.GimbalOutState == 0) {
    Transit(State::kNoForce);
    // enable_count = 0;
  } else {
    // B键
    if (global.bc->rc->key(DR16::Key::kB) == 1) {  // UI更新
      RcKey_KB_flag = 1;
    }
    if (global.bc->rc->key(DR16::Key::kB) == 0 && RcKey_KB_flag == 1) {
      RcKey_KB_value++;
      RcKey_KB_flag = 0;
    }
    if (tcremote.data().keyboard_key >> 15 == 1) {  // UI更新
      TCRcKey_KB_flag = 1;
    }
    if (tcremote.data().keyboard_key >> 15 == 0 && TCRcKey_KB_flag == 1) {
      RcKey_KB_value++;
      TCRcKey_KB_flag = 0;
    }
    if (RcKey_KB_value % 2 == 1) {
      global.chassis_tx->gimbal_data_tx.ui_flag = 0x00;
    } else {
      global.chassis_tx->gimbal_data_tx.ui_flag = 0x01;
    }

    // Q键
    if (global.bc->rc->key(DR16::Key::kQ) == 1) {  // 高腿长更新
      RcKey_KQ_flag = 1;
    }
    if (global.bc->rc->key(DR16::Key::kQ) == 0 && RcKey_KQ_flag == 1) {
      RcKey_KQ_value++;
      RcKey_KQ_flag = 0;
    }
    if (tcremote.data().keyboard_key >> 6 == 1) {  // 高腿长更新
      TCRcKey_KQ_flag = 1;
    }
    if (tcremote.data().keyboard_key >> 6 == 0 && TCRcKey_KQ_flag == 1) {
      RcKey_KQ_value++;
      TCRcKey_KQ_flag = 0;
    }
    if (RcKey_KQ_value % 2 == 1) {
      high_mode_ = true;
    } else {
      high_mode_ = false;
    }
    uiflag = RcKey_KB_value;
    w = global.bc->rc->key(DR16::Key::kW);
    switch (global.bc->rc->switch_r()) {
      case DR16::SwitchPosition::kDown:
        Transit(State::kNoForce);
        break;
      case DR16::SwitchPosition::kMid:
        if (global.bc->rc->switch_l() == DR16::SwitchPosition::kMid) {
          Transit(State::kShoot);
        } else if (global.bc->rc->switch_l() == DR16::SwitchPosition::kUp) {
          Transit(State::kAutoFu);
        }else {
          Transit(State::kTest);
          if (enable_count == 300) {
            global.motor->yaw_motor->SendInstruction(DmMotorInstructions::kClearError);
            global.motor->pitch_motor->SendInstruction(DmMotorInstructions::kClearError);
          }
          if (enable_count < 500) {
            enable_count++;
          } else {
            global.motor->DMEnable();
          }
        }
        break;
      case DR16::SwitchPosition::kUp:
        if (global.bc->rc->switch_l() == DR16::SwitchPosition::kMid || high_mode_) {
          Transit(State::kHigh);
        } else {
          Transit(State::kAutoShoot);
          if (enable_count < 500) {
            enable_count++;
          } else {
            global.motor->DMEnable();
          }
        }
        break;
      default:
        Transit(State::kNoForce);
        break;
    }

    gimbal_flag = global.chassis_rx->chassis_data_rx.GimbalOutState;
    init_count = init_count_;
  }
}

void Fsm::Update_Test() {
  global.chassis_tx->gimbal_data_tx.ChassisStateRequest = 0x01;  // 正常起立状态
  global.chassis_tx->gimbal_data_tx.L0Change = 0x00;             // 正常腿长

  // 控制腿的状态
  // if (global.bc->rc->right_x() > 650 || global.bc->tcremote.data().keyboard_key >> 9 == 1) {
  //   global.chassis_communicator->jump_flag = true;
  // } else if (  // global.bc->rc->right_x() == -660 ||
  //     global.bc->tcremote.data().keyboard_key >> 10 == 1) {
  //   global.chassis_communicator->gimbal_data_tx.L0Change = 0x00;  // 低腿长
  //   global.chassis_communicator->jump_flag = false;
  //   global.chassis_communicator->jump_count = 0;
  // } else {
  //   if (global.chassis_communicator->jump_flag &&
  //       // global.bc->rc->right_x() < 650
  //       global.bc->tcremote.data().keyboard_key >> 9 == 0) {
  //     // 跳跃计时增加
  //     global.chassis_communicator->jump_count++;
  //     // 跳跃腿长控制
  //     // if (global.chassis_communicator->jump_count < 24) {
  //     //   global.chassis_communicator->gimbal_data_tx.L0Change = 0x02;  // 跳跃时先下蹲
  //     // } else if (global.chassis_communicator->jump_count < 50) {
  //     //   global.chassis_communicator->gimbal_data_tx.L0Change = 0x03;  // 伸腿
  //     // } else if (global.chassis_communicator->jump_count < 95) {
  //     //   global.chassis_communicator->gimbal_data_tx.L0Change = 0x04;  // 收腿
  //     // } else if (global.chassis_communicator->jump_count < 120) {
  //     //   global.chassis_communicator->gimbal_data_tx.L0Change = 0x05;  // 缓冲
  //     // } else {
  //     //   // 重置状态
  //     //   global.chassis_communicator->jump_flag = false;
  //     //   global.chassis_communicator->jump_count = 0;
  //     // }
  //     if (global.chassis_communicator->jump_count < 26) {
  //       global.chassis_communicator->gimbal_data_tx.L0Change = 0x03;  // 跳跃时先下蹲
  //     } else if (global.chassis_communicator->jump_count < 71) {
  //       global.chassis_communicator->gimbal_data_tx.L0Change = 0x04;  // 伸腿
  //     } else if (global.chassis_communicator->jump_count < 96) {
  //       global.chassis_communicator->gimbal_data_tx.L0Change = 0x05;  // 收腿
  //     } else {
  //       // 重置状态
  //       global.chassis_communicator->jump_flag = false;
  //       global.chassis_communicator->jump_count = 0;
  //     }
  //   } else {
  //     global.chassis_communicator->gimbal_data_tx.L0Change = 0x01;  // 正常腿长
  //     global.chassis_communicator->jump_flag = false;
  //     global.chassis_communicator->jump_count = 0;
  //   }
  // }

  // 判断是否小陀螺
  if (global.bc->rc->dial() >= 600 ||
      (global.bc->rc->key(DR16::Key::kShift) == 1 && global.bc->rc->key(DR16::Key::kCtrl) == 1) ||
      tcremote.data().keyboard_key >> 4 == 1) {
    global.chassis_tx->gimbal_data_tx.ChassisStateRequest = 0x02;  // 正常起立状态
    global.motor->yaw_compensation_ = -0.6f;
  } else if (global.bc->rc->dial() == -660 || global.bc->rc->key(DR16::Key::kShift) == 1 ||
             (tcremote.data().keyboard_key >> 4 == 1 && tcremote.data().keyboard_key >> 5 == 1)) {
    global.chassis_tx->gimbal_data_tx.ChassisStateRequest = 0x03;  // 跳跃
    global.motor->yaw_compensation_ = 0.f;
  } else {
    global.motor->yaw_compensation_ = 0.f;
  }

  // 控制遥控器输入量
  if (global.bc->rc->key(DR16::Key::kW) || tcremote.data().keyboard_key >> 0 & 0x01) {
    if (global.bc->kw < 0.6) {
      global.bc->kw += 0.025;
    } else if (global.bc->kw < 1.) {
      global.bc->kw += 0.01;
    } else {
    }
  } else {
    global.bc->kw = 0.f;
  }
  if (global.bc->rc->key(DR16::Key::kS) || tcremote.data().keyboard_key >> 1 & 0x01) {
    if (global.bc->ks < 0.6) {
      global.bc->ks += 0.025;
    } else if (global.bc->ks < 1.) {
      global.bc->ks += 0.01;
    } else {
    }
  } else {
    global.bc->ks = 0.f;
  }

  global.chassis_tx->gimbal_data_tx.ChassisMoveYRequest =
      static_cast<int8_t>(global.bc->rc->left_y() * 127.0f / 660.0f) + global.bc->kw * 660.f - global.bc->ks * 660.f;
  global.chassis_tx->gimbal_data_tx.ChassisMoveXRequest = global.bc->rc->left_x();
}

void Fsm::Update_Chassis_Request() {
  switch (mode_) {
    case State::kNoForce:
      global.chassis_tx->gimbal_data_tx.ChassisMoveYRequest = 0;
      global.chassis_tx->gimbal_data_tx.ChassisStateRequest = 0x00;
      global.chassis_tx->gimbal_data_tx.L0Change = 0x01;
      inited_ = false;
      break;
    case State::kTest:
      if (init_count_ == 300) {
        Update_Test();
      }else {
        global.chassis_tx->gimbal_data_tx.ChassisMoveYRequest = 0;
        global.chassis_tx->gimbal_data_tx.ChassisStateRequest = 0x00;
        global.chassis_tx->gimbal_data_tx.L0Change = 0x01;
      }
      break;
    case State::kShoot:
      Update_Test();
      global.chassis_tx->gimbal_data_tx.L0Change = 0x01;
      break;
    case State::kAutoFu:
      Update_Test();
      global.chassis_tx->gimbal_data_tx.ChassisStateRequest = 0x00;  // 底盘无力
      break;
    case State::kHigh:
      Update_Test();
      //global.chassis_tx->gimbal_data_tx.L0Change = 0x06;  // 伸腿
      break;
    case State::kAutoShoot:
      Update_Test();
      if (tcremote.data().keyboard_key >> 7 == 1) {  // E键飞坡腿长更新
        TCRcKey_KE_flag = 1;
      }
      if (tcremote.data().keyboard_key >> 7 == 0 && TCRcKey_KE_flag == 1) {
        TCRcKey_KE_value ++;
        TCRcKey_KE_flag = 0;
      }
      if (TCRcKey_KE_value % 2 ==1) {
        //global.chassis_tx->gimbal_data_tx.L0Change = 0x09;
      }else{}
      // global.chassis_tx->gimbal_data_tx.ChassisStateRequest = 0x00;  // 底盘无力
      //  global.motor->yaw_compensation_ = 0.f;
      break;
    default:
      global.chassis_tx->gimbal_data_tx.ChassisMoveYRequest = 0;
      global.chassis_tx->gimbal_data_tx.ChassisStateRequest = 0x00;
      global.chassis_tx->gimbal_data_tx.L0Change = 0x01;
      break;
  }
}

//  根据状态控制电机
void Fsm::Update_Control() {
  global.motor->MotorPidInit();
  state = mode_;
  switch (mode_) {
    case State::kNoForce:
      RcKey_KB_flag = 0, RcKey_KB_value = 0, TCRcKey_KB_flag = 0;
      RcKey_KQ_flag = 0, RcKey_KQ_value = 0, TCRcKey_KQ_flag = 0;
      TCRcKey_KF_flag = 0, TCRcKey_KF_value = 0;
      TCRcKey_KG_flag = 0, TCRcKey_KG_value = 0;
      TCRcKey_KE_flag = 0, TCRcKey_KE_value = 0;
      global.motor->CalcYawPos(global.motor->yaw_motor->pos());
      init_count_ = 0;
      global.motor->reset_yaw_flag = 0;
      global.motor->change_yaw_init_flag = false;
      global.motor->aimbot_comm->aimbot_state_ = 0;
      break;
    case State::kTest:
      global.motor->CalcYawPos(global.motor->yaw_motor->pos());
      // global.motor->Transit_initmode(static_cast<Motor::InitFlag>(global.chassis_rx->chassis_data_rx.GimbalInitFlag));
      // global.motor->Transit_initmode((tcremote.data().keyboard_key >> 7) & 0x01 ||
      //(global.bc->rc->key(DR16::Key::kE) == 1));
      if (init_count_ < 300) {
        init_count_++;
        global.motor->DMInitControl();
      } else {
        global.motor->DMAutoControl();
      }
      break;
    case State::kHigh:
      global.motor->DMAutoControl();
      global.motor->ShootAutoControl();
      break;
    case State::kShoot:
      global.motor->DMAimControl();
      global.motor->ShootNormalControl();
      break;
    case State::kAutoFu:
      global.motor->DMAimControl();
      global.motor->ShootAutoFuControl();
      break;
    case State::kAutoShoot:
      //更新自瞄模式
      if (tcremote.data().keyboard_key >> 9 == 1) {  // F键小符更新
        TCRcKey_KF_flag = 1;
      }
      if (tcremote.data().keyboard_key >> 9 == 0 && TCRcKey_KF_flag == 1) {
        TCRcKey_KF_value ++;
        TCRcKey_KF_flag = 0;
      }
      // if (tcremote.data().keyboard_key >> 10 == 1) {  // G键大符更新
      //   TCRcKey_KG_flag = 1;
      // }
      // if (tcremote.data().keyboard_key >> 10 == 0 && TCRcKey_KG_flag == 1) {
      //   TCRcKey_KG_value ++;
      //   TCRcKey_KG_flag = 0;
      // }
      if (TCRcKey_KF_value % 3 == 1) {
        auto_mode_ = 2;
      }else if (TCRcKey_KF_value % 3 == 2) {
        auto_mode_ = 3;
      }else {
        auto_mode_ = 1;
      }

      global.motor->CalcYawPos(global.motor->yaw_motor->pos());
      // if (init_count_ < 300) {
      //   init_count_++;
      //   global.motor->DMInitControl();
      // } else {
        global.motor->DMAutoControl();
     // }
      global.motor->ShootAutoControl();
      break;
    default:
      global.motor->CalcYawPos(global.motor->yaw_motor->pos());
      init_count_ = 0;
      global.motor->reset_yaw_flag = 0;
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
  robot_id = global.chassis_rx->chassis_data_rx.id;
  if (global.chassis_rx->chassis_data_rx.Bulletspeed == 0) {
    global.chassis_rx->chassis_data_rx.Bulletspeed = 21;
  }
  debug_w = global.bc->hipnuc_imu->quat_w();
  debug_x = global.bc->hipnuc_imu->quat_x();
  debug_y = global.bc->hipnuc_imu->quat_y();
  debug_z = global.bc->hipnuc_imu->quat_z();
  // 状态更新
  Update_State();

  // 控制量更新
  Update_Control();

  // 发送Dji电机信息
  global.motor->SendDjiCommand();

  global.motor->aimbot_comm->UpdateControl(global.bc->hipnuc_imu->yaw(), global.bc->hipnuc_imu->pitch(),
                                           -global.bc->hipnuc_imu->roll(), robot_id, auto_mode_, global.bc->imu_count,
                                           global.chassis_rx->chassis_data_rx.Bulletspeed);

  init_flag = global.chassis_rx->chassis_data_rx.GimbalInitFlag;
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
    global.chassis_tx->SendChassisCommand();
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