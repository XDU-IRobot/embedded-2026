#include "Chassis.hpp"

#include "Gimbal.hpp"

void Chassis::ChassisInit() {
  chassis->chassis_follow_pid_.SetCircular(true).SetCircularCycle(M_PI * 2.0f);
  chassis->chassis_follow_pid_.SetKp(9000.0f);
  chassis->chassis_follow_pid_.SetKi(0.0f);
  chassis->chassis_follow_pid_.SetKd(600000.0f);
  chassis->chassis_follow_pid_.SetMaxOut(chassis->chassis_max_speed_w_);
  chassis->chassis_follow_pid_.SetMaxIout(0.0f);
}

void Chassis::ChassisTask() {
  chassis->ChassisStateUpdate();  // 底盘状态机更新
}

void Chassis::ChassisStateUpdate() {
  if ((
          // !globals->referee_data_buffer->data().robot_status.power_management_chassis_output ||
          !globals->device_chassis.all_device_ok() && gimbal->down_yaw_enable_flag_)) {
    chassis->ChassisMove_ = kUnable;
  } else {
    switch (globals->StateMachine_) {
      case kNoForce:
        chassis->ChassisDisableUpdate();  // 底盘电机失能计算
        break;

      case kTest:
        switch (chassis->ChassisMove_) {
          case kCsRemote:
          case kCsNavigate:
            chassis->ChassisEnableUpdate();  // 底盘电机使能计算
            break;

          default:                            // 错误状态，所有电机失能
            chassis->ChassisDisableUpdate();  // 底盘电机失能计算
            break;
        }
        break;

      case kMatch:                      // 比赛模式下，所有电机正常工作
        chassis->ChassisMatchUpdate();  // 底盘电机使能计算
        break;

      default:                            // 错误状态，所有电机失能
        chassis->ChassisDisableUpdate();  // 底盘电机失能计算
        break;
    }
  }
}

void Chassis::ChassisRCDataUpdate() {
  chassis->down_yaw_delta_ = chassis->front_down_yaw_angle_ -
                             rm::modules::Wrap(globals->down_yaw_motor->pos(), -static_cast<f32>(M_PI), M_PI);  //
  // + rm::modules::Map(gimbal->mid_up_yaw_angle_ - static_cast<f32>(globals->up_yaw_motor->encoder()), 0.0f,
  //                    globals->GM6020_encoder_max_, 0.0f, 2.0f * static_cast<f32>(M_PI));
  chassis->down_yaw_delta_ = rm::modules::Wrap(chassis->down_yaw_delta_, -static_cast<f32>(M_PI), M_PI);
  if (std::abs(globals->rc->right_y()) > 20 || std::abs(globals->rc->right_x()) > 20) {
    chassis->chassis_receive_x_ =
        -rm::modules::Map(-static_cast<f32>(globals->rc->right_y()), -660, 660, -chassis->chassis_sensitivity_xy_,
                          chassis->chassis_sensitivity_xy_);
    chassis->chassis_receive_y_ =
        -rm::modules::Map(-static_cast<f32>(globals->rc->right_x()), -660, 660, -chassis->chassis_sensitivity_xy_,
                          chassis->chassis_sensitivity_xy_);
  } else {
    chassis->chassis_receive_x_ = 0.0f;
    chassis->chassis_receive_y_ = 0.0f;
  }
  if (globals->rc->dial() >= 650) {
    chassis->chassis_target_x_ =
        chassis->chassis_receive_x_ * std::cos(chassis->down_yaw_delta_ + chassis->chassis_move_delta_angle_) -
        chassis->chassis_receive_y_ * std::sin(chassis->down_yaw_delta_ + chassis->chassis_move_delta_angle_);
    chassis->chassis_target_y_ =
        chassis->chassis_receive_y_ * std::cos(chassis->down_yaw_delta_ + chassis->chassis_move_delta_angle_) +
        chassis->chassis_receive_x_ * std::sin(chassis->down_yaw_delta_ + chassis->chassis_move_delta_angle_);
    chassis->chassis_target_w_ = 6000.0f;
  } else if (globals->rc->dial() <= -650) {
    chassis->chassis_target_x_ =
        chassis->chassis_receive_x_ * std::cos(chassis->down_yaw_delta_ - chassis->chassis_move_delta_angle_) -
        chassis->chassis_receive_y_ * std::sin(chassis->down_yaw_delta_ - chassis->chassis_move_delta_angle_);
    chassis->chassis_target_y_ =
        chassis->chassis_receive_y_ * std::cos(chassis->down_yaw_delta_ - chassis->chassis_move_delta_angle_) +
        chassis->chassis_receive_x_ * std::sin(chassis->down_yaw_delta_ - chassis->chassis_move_delta_angle_);
    chassis->chassis_target_w_ = -6000.0f;
  } else {
    chassis->chassis_target_x_ = chassis->chassis_receive_x_ * std::cos(chassis->down_yaw_delta_) -
                                 chassis->chassis_receive_y_ * std::sin(chassis->down_yaw_delta_);
    chassis->chassis_target_y_ = chassis->chassis_receive_y_ * std::cos(chassis->down_yaw_delta_) +
                                 chassis->chassis_receive_x_ * std::sin(chassis->down_yaw_delta_);
    chassis->chassis_follow_pid_.Update(0.0f, chassis->down_yaw_delta_, 1.0f);
    chassis->chassis_target_w_ = chassis->chassis_follow_pid_.out();
  }
  if (std::abs(chassis->chassis_target_w_) > 4000.0f) {
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

void Chassis::ChassisNavigateDataUpdate() {
  chassis->down_yaw_delta_ = chassis->front_down_yaw_angle_ - globals->down_yaw_motor->pos();
  chassis->down_yaw_delta_ = rm::modules::Wrap(chassis->down_yaw_delta_, -static_cast<f32>(M_PI), M_PI);
  chassis->chassis_receive_x_ =
      rm::modules::Map(rm::modules::Clamp(globals->NucControl.vx + globals->navigate_communicator->chassis_target_x(),
                                          -chassis_max_navigate_xyw_, chassis_max_navigate_xyw_),
                       -chassis_max_navigate_xyw_, chassis_max_navigate_xyw_,  //
                       -chassis->chassis_sensitivity_xy_, chassis->chassis_sensitivity_xy_);
  chassis->chassis_receive_y_ =
      rm::modules::Map(rm::modules::Clamp(-globals->NucControl.vy - globals->navigate_communicator->chassis_target_y(),
                                          -chassis_max_navigate_xyw_, chassis_max_navigate_xyw_),
                       -chassis_max_navigate_xyw_, chassis_max_navigate_xyw_,  //
                       -chassis->chassis_sensitivity_xy_, chassis->chassis_sensitivity_xy_);
  chassis->chassis_target_w_ =
      rm::modules::Map(rm::modules::Clamp(globals->NucControl.vw + globals->navigate_communicator->chassis_target_w(),
                                          -chassis_max_navigate_xyw_, chassis_max_navigate_xyw_),
                       -chassis->chassis_max_navigate_xyw_, chassis->chassis_max_navigate_xyw_,  //
                       -chassis->chassis_max_speed_w_, chassis->chassis_max_speed_w_);
  if (std::abs(chassis->chassis_target_w_) > 0) {
    chassis->chassis_move_delta_angle_ =
        -0.5f * rm::modules::Clamp(globals->NucControl.vw + globals->navigate_communicator->chassis_target_w(),
                                   -chassis_max_navigate_xyw_, chassis_max_navigate_xyw_);
    chassis->chassis_target_x_ =
        chassis->chassis_receive_x_ * std::cos(chassis->down_yaw_delta_ + chassis->chassis_move_delta_angle_) -
        chassis->chassis_receive_y_ * std::sin(chassis->down_yaw_delta_ + chassis->chassis_move_delta_angle_);
    chassis->chassis_target_y_ =
        chassis->chassis_receive_y_ * std::cos(chassis->down_yaw_delta_ + chassis->chassis_move_delta_angle_) +
        chassis->chassis_receive_x_ * std::sin(chassis->down_yaw_delta_ + chassis->chassis_move_delta_angle_);
  } else {
    chassis->chassis_target_x_ = chassis->chassis_receive_x_ * std::cos(chassis->down_yaw_delta_) -
                                 chassis->chassis_receive_y_ * std::sin(chassis->down_yaw_delta_);
    chassis->chassis_target_y_ = chassis->chassis_receive_y_ * std::cos(chassis->down_yaw_delta_) +
                                 chassis->chassis_receive_x_ * std::sin(chassis->down_yaw_delta_);
  }
  if (std::abs(chassis->chassis_target_w_) > 4000.0f) {
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
  globals->chassis_controller.Update(globals->wheel_lf->rpm(), globals->wheel_rf->rpm(),  // 轮电机当前值
                                     globals->wheel_lb->rpm(), globals->wheel_rb->rpm());
}

void Chassis::ChassisMatchUpdate() {
  chassis->ChassisMove_ = kCsNavigate;
  chassis->ChassisEnableUpdate();
}

void Chassis::ChassisEnableUpdate() {
  globals->chassis_controller.Enable(true);
  if (chassis->ChassisMove_ == kCsRemote) {
    chassis->ChassisRCDataUpdate();
    chassis->ChassisMovePIDUpdate();
  } else if (chassis->ChassisMove_ == kCsNavigate) {
    chassis->ChassisNavigateDataUpdate();
    chassis->ChassisMovePIDUpdate();
  } else {
    globals->chassis_controller.Enable(false);
  }
  // chassis->PowerLimitLoop();
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
  if (globals->referee_data_buffer->data().power_heat_data.buffer_energy < 10) {
    chassis->k_speed_power_limit_ = 0.0f;
    chassis->chassis_power_limit_ =
        static_cast<f32>(globals->referee_data_buffer->data().robot_status.chassis_power_limit) * 0.6f;
  } else if (globals->referee_data_buffer->data().power_heat_data.buffer_energy < 60) {
    chassis->k_speed_power_limit_ = static_cast<f32>(
        pow(static_cast<f32>(globals->referee_data_buffer->data().power_heat_data.buffer_energy) / 60.0f, 2));
    chassis->chassis_power_limit_ =
        static_cast<f32>(globals->referee_data_buffer->data().robot_status.chassis_power_limit) * 0.8f;
  } else {
    chassis->k_speed_power_limit_ = 1.0f;
    chassis->chassis_power_limit_ = globals->referee_data_buffer->data().robot_status.chassis_power_limit;
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
