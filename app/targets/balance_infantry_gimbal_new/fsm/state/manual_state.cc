#include "manual_state.hpp"


namespace fsm {
    namespace state {
        etl::fsm_state_id_t Manual::on_enter_state() {
            globals->pitch_motor.SendInstruction(rm::device::DmMotorInstructions::kClearError);
            globals->yaw_motor.SendInstruction(rm::device::DmMotorInstructions::kClearError);
            HAL_Delay(1);
            globals->pitch_motor.SendInstruction(rm::device::DmMotorInstructions::kEnable);
            globals->yaw_motor.SendInstruction(rm::device::DmMotorInstructions::kEnable);
            globals->gimbal_controller.Enable(1);

            rc_yaw_tar = 0.f;
            rc_pitch_tar = 0.f;

            globals->led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
            globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<2>>();
            return No_State_Change;
        };

        etl::fsm_state_id_t Manual::on_event(const event::ControlLoop &e) {
            rc_yaw_tar -= rm::modules::Map( (globals->rc.left_x() < 10 && globals->rc.left_x() > -10) ? 0 : globals->rc.left_x(), -660, 660, -0.005f, 0.005f);
            rc_yaw_tar = rm::modules::Wrap(rc_yaw_tar, -M_PI, M_PI);
            rc_pitch_tar += rm::modules::Map(globals->rc.left_y(), -660, 660, -0.005f, 0.005f);
            rc_pitch_tar = rm::modules::Clamp(rc_pitch_tar, -0.3, 0.68);

//      rc_left_x = rm::modules::Map(globals->rc.left_x(), -660, 660, -0.005f, 0.005f);
            globals->gimbal_controller.SetTarget(rc_yaw_tar,rc_pitch_tar);
            globals->gimbal_controller.Update(globals->yaw_motor.pos(),globals->yaw_motor.vel(),globals->pitch_motor.pos(),globals->pitch_motor.vel());

            globals->yaw_motor.SetMitCommand(0,0,globals->gimbal_controller.output().yaw,0,0);
            globals->pitch_motor.SetMitCommand(0,0,globals->gimbal_controller.output().pitch+1.3*cos(globals->ahrs.euler_angle().pitch),0,0);

            return No_State_Change;
        };
        etl::fsm_state_id_t Manual::on_event(const event::ForceModeSwitch &e) {
            return e.target_mode;
        }

        etl::fsm_state_id_t Manual::on_event_unknown(const etl::imessage &) {
            return No_State_Change;
        }
    }  // namespace state
}  // namespace fsm
