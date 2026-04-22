
#include "auto_state.hpp"

namespace fsm {
    namespace state {
        etl::fsm_state_id_t Auto::on_enter_state() {
            globals->pitch_motor.SendInstruction(rm::device::DmMotorInstructions::kClearError);
            globals->yaw_motor.SendInstruction(rm::device::DmMotorInstructions::kClearError);
            HAL_Delay(1);
            globals->pitch_motor.SendInstruction(rm::device::DmMotorInstructions::kEnable);
            globals->yaw_motor.SendInstruction(rm::device::DmMotorInstructions::kEnable);
            globals->gimbal_controller.Enable(1);
            auto_pitch_tar = globals->ahrs.euler_angle().pitch;
            auto_yaw_tar = globals->ahrs.euler_angle().yaw;
            globals->led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
            globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<2>>();
            return No_State_Change;
        };

        etl::fsm_state_id_t Auto::on_event(const event::ControlLoop &e) {
            auto_pitch_tar = globals->aimbot_comm.pitch();
            auto_pitch_tar = rm::modules::Clamp(auto_pitch_tar, -0.3f,0.68f);

            auto_yaw_tar =  globals->aimbot_comm.yaw();
            auto_yaw_tar = rm::modules::Wrap(auto_yaw_tar, -M_PI, M_PI);

            globals->gimbal_controller.SetTarget(auto_yaw_tar,auto_pitch_tar);
            globals->gimbal_controller.Update(globals->yaw_motor.pos(),globals->yaw_motor.vel(),globals->pitch_motor.pos(),globals->pitch_motor.vel());
            const auto ff = globals->dynamics_.ComputeFf(auto_yaw_tar, auto_pitch_tar, 0, 0, 0, 0, {0, 0, -9.81});
            yaw_ff_ = ff(0);
            pitch_ff_ = ff(1);
            globals->yaw_motor.SetMitCommand(0,0,globals->gimbal_controller.output().yaw+yaw_ff_,0,0);
            globals->pitch_motor.SetMitCommand(0,0,globals->gimbal_controller.output().pitch+pitch_ff_,0,0);

            return No_State_Change;
        };

        etl::fsm_state_id_t Auto::on_event(const event::ForceModeSwitch &e) {
            return e.target_mode;
        }

        etl::fsm_state_id_t Auto::on_event_unknown(const etl::imessage &) {
            return No_State_Change;
        }
    }  // namespace state
}  // namespace fsm