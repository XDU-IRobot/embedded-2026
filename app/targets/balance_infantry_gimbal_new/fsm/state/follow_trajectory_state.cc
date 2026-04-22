#include "follow_trajectory_state.hpp"
#include "manual_state.hpp"


namespace fsm {
    namespace state {
        etl::fsm_state_id_t FollowTrajectory::on_enter_state() {
//      globals->yaw_motor.SendInstruction(rm::device::DmMotorInstructions::kEnable);
//      globals->pitch_motor.SendInstruction(rm::device::DmMotorInstructions::kEnable);
//      HAL_Delay(200);
//      for (int i = 0; i < 10; i++) {
//          globals->yaw_motor.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
//          globals->pitch_motor.SendInstruction(rm::device::DmMotorInstructions::kSetZeroPosition);
//          HAL_Delay(100);
//      }
//      HAL_Delay(200);
//      globals->yaw_motor.SendInstruction(rm::device::DmMotorInstructions::kDisable);
//      globals->pitch_motor.SendInstruction(rm::device::DmMotorInstructions::kDisable);
            globals->led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
            globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<2>>();
            enter_timestamp_ms_ = HAL_GetTick();
            loop_divisor_ = 0;
            return No_State_Change;
        };

        etl::fsm_state_id_t FollowTrajectory::on_event(const event::ControlLoop &e) {
            const uint32_t elapsed_ms = HAL_GetTick() - enter_timestamp_ms_;
            const auto pitch_traj = traj_pitch_.evaluate(elapsed_ms / 1000.f);
            const auto yaw_traj = traj_yaw_.evaluate(elapsed_ms / 1000.f);

            yaw_target_ = yaw_traj.q * 2.5f;
            pitch_target_ = pitch_traj.q*0.8 + 0.2f;

//     const auto ff = dynamics_.ComputeFf(yaw_target_, pitch_target_, yaw_traj.dq * 2.5f, pitch_traj.dq*0.8, yaw_traj.ddq * 2.5f, pitch_traj.ddq*0.8, {0, 0, -9.81});
            const auto ff = globals->dynamics_.ComputeFf(yaw_target_, pitch_target_, yaw_traj.dq * 2.5f, pitch_traj.dq*0.8, yaw_traj.ddq * 2.5f, pitch_traj.ddq*0.8, {0, 0, -9.81});
            yaw_ff_ = ff(0)*1.1;
            pitch_ff_ = ff(1);
            globals->gimbal_controller.SetTarget(yaw_target_,pitch_target_);
            globals->gimbal_controller.Update(globals->yaw_motor.pos(),globals->yaw_motor.vel(),globals->pitch_motor.pos(),globals->pitch_motor.vel());
//      globals->yaw_motor.SetMitCommand(0,0,0,0,0);
            globals->yaw_motor.SetMitCommand(0,0,globals->gimbal_controller.output().yaw+yaw_ff_,0,0);
            globals->pitch_motor.SetMitCommand(0,0,globals->gimbal_controller.output().pitch +pitch_ff_,0,0);
//      globals->pitch_motor.SetMitCommand(0,0,globals->gimbal_controller.output().pitch+1.18*cos(globals->ahrs.euler_angle().pitch),0,0);

            pitch_error_ = pitch_target_ - globals->pitch_motor.pos();
            loop_divisor_ = (loop_divisor_ + 1) % 50;  // 10hz
            if (loop_divisor_ == 0) {
                ReportData(elapsed_ms, globals->yaw_motor.tau(), globals->yaw_motor.pos(), globals->yaw_motor.vel(),
                           globals->pitch_motor.tau(), globals->pitch_motor.pos(), globals->pitch_motor.vel());
            }
            return No_State_Change;
        };


        etl::fsm_state_id_t FollowTrajectory::on_event(const event::ForceModeSwitch &e) {
            return e.target_mode;
        }

        etl::fsm_state_id_t FollowTrajectory::on_event_unknown(const etl::imessage &) {
            return No_State_Change;
        }
    }  // namespace state
}  // namespace fsm
