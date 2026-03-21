#include "Chassis.hpp"

void Chassis::ChassisInit() {
  chassis->chassis_follow_pid_.SetCircular(true).SetCircularCycle(M_PI * 2.0f);
  chassis->chassis_follow_pid_.SetKp(26000.0f).SetKi(0.0f).SetKd(1200000.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
}

void Chassis::ChassisTask() {
  chassis->ChassisStateUpdate();  // 底盘状态机更新
}

void Chassis::ChassisStateUpdate() {
  // if (referee_data_buffer.data().robot_status.power_management_chassis_output == 0) {
  //    chassis->ChassisMove_ = UNABLE;
  // } else {
  if ((globals->gimbal_communicator->chassis_mode() >> 0 & 0x01) == 1) {
    if ((globals->gimbal_communicator->chassis_mode() >> 3 & 0x01) == 1) {
      chassis->high_speed_mode_flag = true;
    } else {
      chassis->high_speed_mode_flag = false;
    }
    if ((globals->gimbal_communicator->chassis_mode() >> 4 & 0x01) == 1) {
      chassis->buff_state_ = kDaFu;
    } else if ((globals->gimbal_communicator->chassis_mode() >> 5 & 0x01) == 1) {
      chassis->buff_state_ = kXiaoFu;
    } else {
      chassis->buff_state_ = kNormal;
    }
    if ((globals->gimbal_communicator->chassis_mode() >> 1 & 0x01) == 1) {
      chassis->ChassisMove_ = kRotate;
      chassis->ChassisEnableUpdate();
    } else if ((globals->gimbal_communicator->chassis_mode() >> 2 & 0x01) == 1) {
      chassis->ChassisMove_ = kReRotate;
      chassis->ChassisEnableUpdate();
    } else {
      chassis->ChassisMove_ = kFollow;
      chassis->ChassisEnableUpdate();
    }
  } else {
    chassis->ChassisMove_ = kNoForce;
    chassis->ChassisDisableUpdate();
  }
  // }
}

void Chassis::ChassisRCDataUpdate() {
  chassis->down_yaw_delta_ =
      chassis->front_down_yaw_angle_ -
      rm::modules::Map(globals->yaw_motor->encoder(), 0.0f, 8192.0f, 0.0f, 2.0f * static_cast<f32>(M_PI));
  chassis->down_yaw_delta_ = rm::modules::Wrap(chassis->down_yaw_delta_, -static_cast<f32>(M_PI), M_PI);
  if (std::abs(globals->gimbal_communicator->remote_speed_x()) > 0.1f ||
      std::abs(globals->gimbal_communicator->remote_speed_y()) > 0.1f) {
    chassis->chassis_receive_x_ = rm::modules::Map(globals->gimbal_communicator->remote_speed_y(), -1.0f, 1.0f,
                                                   -chassis->chassis_sensitivity_xy_, chassis->chassis_sensitivity_xy_);
    chassis->chassis_receive_y_ = rm::modules::Map(globals->gimbal_communicator->remote_speed_x(), -1.0f, 1.0f,
                                                   -chassis->chassis_sensitivity_xy_, chassis->chassis_sensitivity_xy_);
  } else {
    chassis->chassis_receive_x_ = 0.0f;
    chassis->chassis_receive_y_ = 0.0f;
  }
  if (chassis->ChassisMove_ == kRotate) {
    chassis->chassis_target_x_ =
        chassis->chassis_receive_x_ * std::cos(chassis->down_yaw_delta_ + chassis->chassis_move_delta_angle_) -
        chassis->chassis_receive_y_ * std::sin(chassis->down_yaw_delta_ + chassis->chassis_move_delta_angle_);
    chassis->chassis_target_y_ =
        chassis->chassis_receive_y_ * std::cos(chassis->down_yaw_delta_ + chassis->chassis_move_delta_angle_) +
        chassis->chassis_receive_x_ * std::sin(chassis->down_yaw_delta_ + chassis->chassis_move_delta_angle_);
    chassis->chassis_target_w_ = 4000.0f;
  } else if (chassis->ChassisMove_ == kReRotate) {
    chassis->chassis_target_x_ =
        chassis->chassis_receive_x_ * std::cos(chassis->down_yaw_delta_ - chassis->chassis_move_delta_angle_) -
        chassis->chassis_receive_y_ * std::sin(chassis->down_yaw_delta_ - chassis->chassis_move_delta_angle_);
    chassis->chassis_target_y_ =
        chassis->chassis_receive_y_ * std::cos(chassis->down_yaw_delta_ - chassis->chassis_move_delta_angle_) +
        chassis->chassis_receive_x_ * std::sin(chassis->down_yaw_delta_ - chassis->chassis_move_delta_angle_);
    chassis->chassis_target_w_ = -4000.0f;
  } else {
    chassis->chassis_target_x_ = chassis->chassis_receive_x_ * std::cos(chassis->down_yaw_delta_) -
                                 chassis->chassis_receive_y_ * std::sin(chassis->down_yaw_delta_);
    chassis->chassis_target_y_ = chassis->chassis_receive_y_ * std::cos(chassis->down_yaw_delta_) +
                                 chassis->chassis_receive_x_ * std::sin(chassis->down_yaw_delta_);
    chassis->chassis_follow_pid_.Update(0.0f, -chassis->down_yaw_delta_, 1.0f);
    chassis->chassis_target_w_ = chassis->chassis_follow_pid_.out();
  }
  if (std::abs(chassis->chassis_target_w_) > 3000.0f) {
    chassis->chassis_target_x_ *= 0.5;
    chassis->chassis_target_y_ *= 0.5;
  }
  if (std::sqrt(std::pow(chassis->chassis_target_x_, 2.0f) + std::pow(chassis->chassis_target_y_, 2.0f)) >
      chassis->chassis_max_speed_xy_) {
    chassis->chassis_target_x_ /=
        std::sqrt(std::pow(chassis->chassis_target_x_, 2.0f) + std::pow(chassis->chassis_target_y_, 2.0f)) /
        chassis->chassis_max_speed_xy_;
    chassis->chassis_target_y_ /=
        std::sqrt(std::pow(chassis->chassis_target_x_, 2.0f) + std::pow(chassis->chassis_target_y_, 2.0f)) /
        chassis->chassis_max_speed_xy_;
  }
  chassis->chassis_target_x_ =
      rm::modules::Clamp(chassis->chassis_target_x_, -chassis->chassis_max_speed_xy_, chassis->chassis_max_speed_xy_);
  chassis->chassis_target_y_ =
      rm::modules::Clamp(chassis->chassis_target_y_, -chassis->chassis_max_speed_xy_, chassis->chassis_max_speed_xy_);
  chassis->chassis_target_w_ =
      rm::modules::Clamp(chassis->chassis_target_w_, -chassis->chassis_max_speed_w_, chassis->chassis_max_speed_w_);
}

void Chassis::ChassisMovePIDUpdate() {
  globals->chassis_controller.SetTarget(chassis->chassis_target_x_, chassis->chassis_target_y_,
                                        chassis->chassis_target_w_);
  globals->chassis_controller.Update(
      rm::modules::Map(globals->steer_lf->encoder() - chassis->steer_wheel_init_encoder_[0],  //
                       0.0f, 8191.0f, 0.0f, 2.0f * static_cast<f32>(M_PI)),
      rm::modules::Map(globals->steer_rf->encoder() - chassis->steer_wheel_init_encoder_[1],  //
                       0.0f, 8191.0f, 0.0f, 2.0f * static_cast<f32>(M_PI)),
      rm::modules::Map(globals->steer_lb->encoder() - chassis->steer_wheel_init_encoder_[2],  //
                       0.0f, 8191.0f, 0.0f, 2.0f * static_cast<f32>(M_PI)),
      rm::modules::Map(globals->steer_rb->encoder() - chassis->steer_wheel_init_encoder_[3],  //
                       0.0f, 8191.0f, 0.0f, 2.0f * static_cast<f32>(M_PI)),
      globals->steer_lf->rpm(), globals->steer_rf->rpm(), globals->steer_lb->rpm(), globals->steer_rb->rpm(),
      globals->wheel_lf->rpm(), -globals->wheel_rf->rpm(), globals->wheel_lb->rpm(), -globals->wheel_rb->rpm(), 2.0f);
}

void Chassis::ChassisEnableUpdate() {
  globals->chassis_controller.Enable(true);
  if (chassis->ChassisMove_ == kFollow || chassis->ChassisMove_ == kRotate || chassis->ChassisMove_ == kReRotate) {
    chassis->ChassisRCDataUpdate();
    chassis->ChassisMovePIDUpdate();
  } else {
    globals->chassis_controller.Enable(false);
    chassis->ChassisMovePIDUpdate();
  }
  // chassis->PowerLimitLoop();
  chassis->SetMotorCurrent();
}

void Chassis::ChassisDisableUpdate() {
  globals->chassis_controller.Enable(false);
  chassis->ChassisMovePIDUpdate();
  chassis->SetMotorCurrent();
}

void Chassis::SpeedModeChange() {
  // 超级电容是否可开启判断
  if (globals->supercap->voltage() < 16.0f || globals->supercap->voltage() > 35.0f ||
      (globals->supercap->error(rm::device::SuperCapError::kOverVoltage) << 0 |
       globals->supercap->error(rm::device::SuperCapError::kOverCurrent) << 1 |
       globals->supercap->error(rm::device::SuperCapError::kUnderVoltage) << 2 |
       globals->supercap->error(rm::device::SuperCapError::kInputUnderVoltage) << 3 |
       globals->supercap->error(rm::device::SuperCapError::kNoData) << 4) == true ||
      globals->referee_data->data().power_heat_data.buffer_energy < 30) {
    chassis->speed_mode_ = kNormal;
  } else if (chassis->high_speed_mode_flag == true && globals->supercap->voltage() > 18.0f &&
             globals->referee_data->data().power_heat_data.buffer_energy > 30) {
    chassis->speed_mode_ = kHighSpeed;
  } else if (chassis->high_speed_mode_flag == false) {
    chassis->speed_mode_ = kNormalSpeed;
  }
}

void Chassis::PowerLimitLoop() {
  // 缓冲能量过低判断
  if (globals->referee_data->data().power_heat_data.buffer_energy < 10) {
    chassis->k_speed_power_limit_ = 0.0f;
  } else if (globals->referee_data->data().power_heat_data.buffer_energy < 60) {
    chassis->k_speed_power_limit_ =
        static_cast<f32>(pow(static_cast<f32>(globals->referee_data->data().power_heat_data.buffer_energy) / 60.0f, 2));
  } else {
    chassis->k_speed_power_limit_ = 1.0f;
  }
}

void Chassis::SetMotorCurrent() {
  globals->steer_lf->SetCurrent(
      static_cast<i16>(globals->chassis_controller.output().lf_steer * chassis->k_speed_power_limit_));
  globals->steer_rf->SetCurrent(
      static_cast<i16>(globals->chassis_controller.output().rf_steer * chassis->k_speed_power_limit_));
  globals->steer_lb->SetCurrent(
      static_cast<i16>(globals->chassis_controller.output().lb_steer * chassis->k_speed_power_limit_));
  globals->steer_rb->SetCurrent(
      static_cast<i16>(globals->chassis_controller.output().rb_steer * chassis->k_speed_power_limit_));
  globals->wheel_lf->SetCurrent(
      static_cast<i16>(globals->chassis_controller.output().lf_wheel * chassis->k_speed_power_limit_));
  globals->wheel_rf->SetCurrent(
      static_cast<i16>(-globals->chassis_controller.output().rf_wheel * chassis->k_speed_power_limit_));
  globals->wheel_lb->SetCurrent(
      static_cast<i16>(globals->chassis_controller.output().lb_wheel * chassis->k_speed_power_limit_));
  globals->wheel_rb->SetCurrent(
      static_cast<i16>(-globals->chassis_controller.output().rb_wheel * chassis->k_speed_power_limit_));
}
