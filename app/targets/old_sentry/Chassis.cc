#include "Chassis.hpp"

#include "Gimbal.hpp"

void Chassis::ChassisInit() {
  chassis->chassis_follow_pid_.SetCircular(true).SetCircularCycle(M_PI * 2.0f);
  chassis->chassis_follow_pid_.SetKp(15000.0f);
  chassis->chassis_follow_pid_.SetKi(0.0f);
  chassis->chassis_follow_pid_.SetKd(10000.0f);
  chassis->chassis_follow_pid_.SetMaxOut(chassis->chassis_max_speed_w_);
  chassis->chassis_follow_pid_.SetMaxIout(0.0f);
}

void Chassis::ChassisTask() {
  chassis->ChassisStateUpdate();  // 底盘状态机更新
}

void Chassis::ChassisStateUpdate() {
  if (!globals->referee_data_buffer->data().robot_status.power_management_chassis_output ||
      !globals->device_chassis.all_device_ok()) {
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
        -rm::modules::Map(globals->rc->right_y(), -globals->rc_max_value_, globals->rc_max_value_,
                          -chassis->chassis_sensitivity_xy_, chassis->chassis_sensitivity_xy_);
    chassis->chassis_receive_y_ =
        -rm::modules::Map(globals->rc->right_x(), -globals->rc_max_value_, globals->rc_max_value_,
                          -chassis->chassis_sensitivity_xy_, chassis->chassis_sensitivity_xy_);
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
    chassis->chassis_target_w_ = 12000.0f;
  } else {
    chassis->chassis_target_x_ = chassis->chassis_receive_x_ * std::cos(chassis->down_yaw_delta_) -
                                 chassis->chassis_receive_y_ * std::sin(chassis->down_yaw_delta_);
    chassis->chassis_target_y_ = chassis->chassis_receive_y_ * std::cos(chassis->down_yaw_delta_) +
                                 chassis->chassis_receive_x_ * std::sin(chassis->down_yaw_delta_);
    chassis->chassis_follow_pid_.Update(0.0f, -chassis->down_yaw_delta_, 1.0f);
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
      rm::modules::Map(-globals->NucControl.vx, -chassis_max_navigate_xy_, chassis_max_navigate_xy_,
                       -chassis->chassis_sensitivity_xy_, chassis->chassis_sensitivity_xy_);
  chassis->chassis_receive_y_ =
      rm::modules::Map(globals->NucControl.vy, -chassis_max_navigate_xy_, chassis_max_navigate_xy_,
                       -chassis->chassis_sensitivity_xy_, chassis->chassis_sensitivity_xy_);
  chassis->chassis_target_w_ =
      rm::modules::Map(globals->NucControl.vw, -chassis->chassis_max_navigate_w_, chassis->chassis_max_navigate_w_,
                       -chassis->chassis_max_speed_w_, chassis->chassis_max_speed_w_);
  if (std::abs(chassis->chassis_target_w_) > 0) {
    chassis->chassis_move_delta_angle_ = -0.5f * chassis->chassis_target_w_;
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
  f32 steer_delta_angle_lf = static_cast<f32>(globals->steer_lf->encoder() - chassis->steer_init_angle_lf_);
  f32 steer_delta_angle_rf = static_cast<f32>(globals->steer_rf->encoder() - chassis->steer_init_angle_rf_);
  f32 steer_delta_angle_lb = static_cast<f32>(globals->steer_lb->encoder() - chassis->steer_init_angle_lb_);
  f32 steer_delta_angle_rb = static_cast<f32>(globals->steer_rb->encoder() - chassis->steer_init_angle_rb_);
  steer_delta_angle_lf =
      rm::modules::Map(steer_delta_angle_lf, 0.0f, globals->GM6020_encoder_max_, 0.0f, 2.0f * static_cast<f32>(M_PI));
  steer_delta_angle_rf =
      rm::modules::Map(steer_delta_angle_rf, 0.0f, globals->GM6020_encoder_max_, 0.0f, 2.0f * static_cast<f32>(M_PI));
  steer_delta_angle_lb =
      rm::modules::Map(steer_delta_angle_lb, 0.0f, globals->GM6020_encoder_max_, 0.0f, 2.0f * static_cast<f32>(M_PI));
  steer_delta_angle_rb =
      rm::modules::Map(steer_delta_angle_rb, 0.0f, globals->GM6020_encoder_max_, 0.0f, 2.0f * static_cast<f32>(M_PI));
  steer_delta_angle_lf = rm::modules::Wrap(steer_delta_angle_lf, -static_cast<f32>(M_PI), M_PI);
  steer_delta_angle_rf = rm::modules::Wrap(steer_delta_angle_rf, -static_cast<f32>(M_PI), M_PI);
  steer_delta_angle_lb = rm::modules::Wrap(steer_delta_angle_lb, -static_cast<f32>(M_PI), M_PI);
  steer_delta_angle_rb = rm::modules::Wrap(steer_delta_angle_rb, -static_cast<f32>(M_PI), M_PI);
  globals->chassis_controller.Update(steer_delta_angle_lf, globals->steer_lf->rpm(),      // 舵电机当前值
                                     steer_delta_angle_rf, globals->steer_rf->rpm(),      //
                                     steer_delta_angle_lb, globals->steer_lb->rpm(),      //
                                     steer_delta_angle_rb, globals->steer_rb->rpm(),      //
                                     globals->wheel_lf->rpm(), globals->wheel_rf->rpm(),  // 轮电机当前值
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
  chassis->PowerLimitLoop();
  chassis->SetMotorCurrent();
}

void Chassis::ChassisDisableUpdate() {
  globals->chassis_controller.Enable(false);
  chassis->ChassisMovePIDUpdate();
  chassis->SetMotorCurrent();
}

void Chassis::PowerLimitLoop() {
  // 缓冲能量过低判断
  if (globals->referee_data_buffer->data().power_heat_data.buffer_energy < 10) {
    chassis->k_speed_power_limit_ = 0.0f;
  } else if (globals->referee_data_buffer->data().power_heat_data.buffer_energy < 60) {
    chassis->k_speed_power_limit_ = static_cast<f32>(
        pow(static_cast<f32>(globals->referee_data_buffer->data().power_heat_data.buffer_energy) / 60.0f, 2));
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
      static_cast<i16>(globals->chassis_controller.output().rf_wheel * chassis->k_speed_power_limit_));
  globals->wheel_lb->SetCurrent(
      static_cast<i16>(globals->chassis_controller.output().lb_wheel * chassis->k_speed_power_limit_));
  globals->wheel_rb->SetCurrent(
      static_cast<i16>(globals->chassis_controller.output().rb_wheel * chassis->k_speed_power_limit_));
}
