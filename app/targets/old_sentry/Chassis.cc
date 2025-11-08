#include "Chassis.hpp"

void Chassis::ChassisInit() {
}

void Chassis::ChassisTask() {
    chassis->ChassisStateUpdate(); // 底盘状态机更新
}

void Chassis::ChassisStateUpdate() {
    switch (globals->StateMachine_) {
        case NO_FORCE:
            ChassisDisableUpdate(); // 底盘电机失能计算
            break;

        case TEST:
            switch (chassis->ChassisMove_) {
                case CS_REMOTE:
                case CS_NAVIGATE:
                    ChassisEnableUpdate(); // 底盘电机使能计算
                    break;

                default: // 错误状态，所有电机失能
                    ChassisDisableUpdate(); // 底盘电机失能计算
                    break;
            }
            break;

        case MATCH: // 比赛模式下，所有电机正常工作
            ChassisMatchUpdate(); // 底盘电机使能计算
            break;

        default: // 错误状态，所有电机失能
            ChassisDisableUpdate(); // 底盘电机失能计算
            break;
    }
}

void Chassis::ChassisRCDataUpdate() {
    chassis->down_yaw_delta_ = front_down_yaw_angle_ - globals->down_yaw_motor->pos();
    chassis->down_yaw_delta_ = rm::modules::Wrap(chassis->down_yaw_delta_, -static_cast<f32>(M_PI), M_PI);
    chassis->chassis_target_x_ = rm::modules::Map(globals->rc->right_x(), -660.0f, 660.0f, -1.0f, 1.0f);
    chassis->chassis_target_y_ = rm::modules::Map(globals->rc->right_y(), -660.0f, 660.0f, -1.0f, 1.0f);
    // chassis->chassis_target_w_ = rm::modules::Map(chassis->down_yaw_delta_, -static_cast<f32>(M_PI), M_PI, -1.0f, 1.0f);
    // chassis->chassis_target_w_ = 0;
}

void Chassis::ChassisMovePIDUpdate() {
    globals->chassis_controller.SetTarget(chassis->chassis_target_x_, chassis->chassis_target_y_,
                                          chassis->chassis_target_w_);
    auto steer_delta_angle_lf = static_cast<f32>(globals->steer_lf->encoder()) - chassis->steer_init_angle_lf_;
    auto steer_delta_angle_rf = static_cast<f32>(globals->steer_rf->encoder()) - chassis->steer_init_angle_rf_;
    auto steer_delta_angle_lb = static_cast<f32>(globals->steer_lb->encoder()) - chassis->steer_init_angle_lb_;
    auto steer_delta_angle_rb = static_cast<f32>(globals->steer_rb->encoder()) - chassis->steer_init_angle_rb_;
    steer_delta_angle_lf = rm::modules::Map(steer_delta_angle_lf, 0.0f, 8191.0f, 0.0f, 2 * static_cast<f32>(M_PI));
    steer_delta_angle_rf = rm::modules::Map(steer_delta_angle_rf, 0.0f, 8191.0f, 0.0f, 2 * static_cast<f32>(M_PI));
    steer_delta_angle_lb = rm::modules::Map(steer_delta_angle_lb, 0.0f, 8191.0f, 0.0f, 2 * static_cast<f32>(M_PI));
    steer_delta_angle_rb = rm::modules::Map(steer_delta_angle_rb, 0.0f, 8191.0f, 0.0f, 2 * static_cast<f32>(M_PI));
    steer_delta_angle_lf = rm::modules::Wrap(steer_delta_angle_lf, -static_cast<f32>(M_PI), M_PI);
    steer_delta_angle_rf = rm::modules::Wrap(steer_delta_angle_rf, -static_cast<f32>(M_PI), M_PI);
    steer_delta_angle_lb = rm::modules::Wrap(steer_delta_angle_lb, -static_cast<f32>(M_PI), M_PI);
    steer_delta_angle_rb = rm::modules::Wrap(steer_delta_angle_rb, -static_cast<f32>(M_PI), M_PI);
    globals->chassis_controller.Update(steer_delta_angle_lf, globals->steer_lf->rpm(), // 舵电机当前值
                                       steer_delta_angle_rf, globals->steer_rf->rpm(),
                                       steer_delta_angle_lb, globals->steer_lb->rpm(),
                                       steer_delta_angle_rb, globals->steer_rb->rpm(),
                                       globals->wheel_lf->rpm(), globals->wheel_rf->rpm(), // 轮电机当前值
                                       globals->wheel_lb->rpm(), globals->wheel_rb->rpm());
}

void Chassis::ChassisNavigateDataUpdate() {
    chassis->chassis_target_x_ = 0.f;
    chassis->chassis_target_y_ = 0.f;
    chassis->chassis_target_w_ = 0.f;
}

void Chassis::ChassisMatchUpdate() {
}

void Chassis::ChassisEnableUpdate() {
    globals->chassis_controller.Enable(true);
    if (chassis->ChassisMove_ == CS_REMOTE) {
        chassis->ChassisRCDataUpdate();
        chassis->ChassisMovePIDUpdate();
    } else if (chassis->ChassisMove_ == CS_NAVIGATE) {
        chassis->ChassisNavigateDataUpdate();
        chassis->ChassisMovePIDUpdate();
    } else {
        globals->chassis_controller.Enable(false);
    }
    globals->steer_lf->SetCurrent(static_cast<i16>(globals->chassis_controller.output().lf_steer));
    globals->steer_rf->SetCurrent(static_cast<i16>(globals->chassis_controller.output().rf_steer));
    globals->steer_lb->SetCurrent(static_cast<i16>(globals->chassis_controller.output().lb_steer));
    globals->steer_rb->SetCurrent(static_cast<i16>(globals->chassis_controller.output().rb_steer));
    globals->wheel_lf->SetCurrent(static_cast<i16>(globals->chassis_controller.output().lf_wheel));
    globals->wheel_rf->SetCurrent(static_cast<i16>(globals->chassis_controller.output().rf_wheel));
    globals->wheel_lb->SetCurrent(static_cast<i16>(globals->chassis_controller.output().lb_wheel));
    globals->wheel_rb->SetCurrent(static_cast<i16>(globals->chassis_controller.output().rb_wheel));
}

void Chassis::ChassisDisableUpdate() {
    globals->chassis_controller.Enable(false);
    globals->steer_lf->SetCurrent(0);
    globals->steer_rf->SetCurrent(0);
    globals->steer_lb->SetCurrent(0);
    globals->steer_rb->SetCurrent(0);
    globals->wheel_lf->SetCurrent(0);
    globals->wheel_rf->SetCurrent(0);
    globals->wheel_lb->SetCurrent(0);
    globals->wheel_rb->SetCurrent(0);
}
