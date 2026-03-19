#include "Chassis.hpp"

void Chassis::ChassisInit() {
  chassis->chassis_follow_pid_.SetCircular(true).SetCircularCycle(M_PI * 2.0f);
  chassis->chassis_follow_pid_.SetKp(0.0f);
  chassis->chassis_follow_pid_.SetKi(0.0f);
  chassis->chassis_follow_pid_.SetKd(0.0f);
  chassis->chassis_follow_pid_.SetMaxOut(chassis->chassis_max_speed_w_);
  chassis->chassis_follow_pid_.SetMaxIout(0.0f);
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
  if (std::abs(globals->gimbal_communicator->remote_speed_x()) > 0.01f ||
      std::abs(globals->gimbal_communicator->remote_speed_y()) > 0.01f) {
    chassis->chassis_receive_x_ =
        rm::modules::Map(0, -660, 660, -chassis->chassis_sensitivity_xy_, chassis->chassis_sensitivity_xy_);
    chassis->chassis_receive_y_ =
        rm::modules::Map(0, -660, 660, -chassis->chassis_sensitivity_xy_, chassis->chassis_sensitivity_xy_);
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
      globals->steer_lf->encoder(), globals->steer_rf->rpm(), globals->steer_lb->encoder(), globals->steer_rb->rpm(),
      globals->steer_lf->encoder(), globals->steer_rf->rpm(), globals->steer_lb->encoder(), globals->steer_rb->rpm(),
      globals->wheel_lf->rpm(), globals->wheel_rf->rpm(), globals->wheel_lb->rpm(), globals->wheel_rb->rpm());
}

void Chassis::ChassisEnableUpdate() {
  globals->chassis_controller.Enable(true);
  if (chassis->ChassisMove_ == kFollow || chassis->ChassisMove_ == kRotate || chassis->ChassisMove_ == kReRotate) {
    chassis->ChassisRCDataUpdate();
    chassis->ChassisMovePIDUpdate();
  } else {
    globals->chassis_controller.Enable(false);
  }
  chassis->PowerLimitLoop();
  chassis->SetMotorCurrent();
}

void Chassis::ChassisDisableUpdate() {
  globals->chassis_controller.Enable(false);
  chassis->ChassisMovePIDUpdate();
  chassis->SetMotorCurrent();
}

void Chassis::PowerLimitLoop() {
  float initial_currents[4];
  initial_currents[0] = globals->chassis_controller.output().lf_wheel;
  initial_currents[1] = globals->chassis_controller.output().rf_wheel;
  initial_currents[2] = globals->chassis_controller.output().lb_wheel;
  initial_currents[3] = globals->chassis_controller.output().rb_wheel;
  chassis->motor_state_[0].speed_rpm = globals->wheel_lf->rpm();
  chassis->motor_state_[0].give_current = globals->chassis_controller.output().lf_wheel;
  chassis->motor_state_[0].measured_current = globals->wheel_lf->current();
  chassis->motor_state_[1].speed_rpm = globals->wheel_rf->rpm();
  chassis->motor_state_[1].give_current = globals->chassis_controller.output().rf_wheel;
  chassis->motor_state_[1].measured_current = globals->wheel_rf->current();
  chassis->motor_state_[2].speed_rpm = globals->wheel_lb->rpm();
  chassis->motor_state_[2].give_current = globals->chassis_controller.output().lb_wheel;
  chassis->motor_state_[2].measured_current = globals->wheel_lb->current();
  chassis->motor_state_[3].speed_rpm = globals->wheel_rb->rpm();
  chassis->motor_state_[3].give_current = globals->chassis_controller.output().rb_wheel;
  chassis->motor_state_[3].measured_current = globals->wheel_rb->current();
  for (int i = 0; i < 4; i++) {
    chassis->power_info_[i] = chassis->power_model_.CalculatePower(chassis->motor_state_[i]);
  }
  chassis->total_power_ = chassis->power_info_[0].total_power + chassis->power_info_[1].total_power +
                          chassis->power_info_[2].total_power + chassis->power_info_[3].total_power;
  chassis->power_model_.DistributePower<4>(chassis->motor_state_, initial_currents, chassis->chassis_power_limit_,
                                           chassis->output_currents_);

  // 缓冲能量过低判断
  if (globals->referee_data->data().power_heat_data.buffer_energy < 10) {
    chassis->k_speed_power_limit_ = 0.0f;
    chassis->chassis_power_limit_ =
        static_cast<f32>(globals->referee_data->data().robot_status.chassis_power_limit) * 0.6f;
  } else if (globals->referee_data->data().power_heat_data.buffer_energy < 60) {
    chassis->k_speed_power_limit_ =
        static_cast<f32>(pow(static_cast<f32>(globals->referee_data->data().power_heat_data.buffer_energy) / 60.0f, 2));
    chassis->chassis_power_limit_ =
        static_cast<f32>(globals->referee_data->data().robot_status.chassis_power_limit) * 0.8f;
  } else {
    chassis->k_speed_power_limit_ = 1.0f;
    chassis->chassis_power_limit_ = globals->referee_data->data().robot_status.chassis_power_limit;
  }
}

void Chassis::SetMotorCurrent() {
  // globals->wheel_lf->SetCurrent(static_cast<i16>(chassis->output_currents_[0]));
  // globals->wheel_rf->SetCurrent(static_cast<i16>(chassis->output_currents_[1]));
  // globals->wheel_lb->SetCurrent(static_cast<i16>(chassis->output_currents_[2]));
  // globals->wheel_rb->SetCurrent(static_cast<i16>(chassis->output_currents_[3]));

  globals->wheel_lf->SetCurrent(
      static_cast<i16>(globals->chassis_controller.output().lf_wheel * chassis->k_speed_power_limit_));
  globals->wheel_rf->SetCurrent(
      static_cast<i16>(globals->chassis_controller.output().rf_wheel * chassis->k_speed_power_limit_));
  globals->wheel_lb->SetCurrent(
      static_cast<i16>(globals->chassis_controller.output().lb_wheel * chassis->k_speed_power_limit_));
  globals->wheel_rb->SetCurrent(
      static_cast<i16>(globals->chassis_controller.output().rb_wheel * chassis->k_speed_power_limit_));
}
