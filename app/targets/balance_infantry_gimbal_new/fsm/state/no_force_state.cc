#include "no_force_state.hpp"

namespace fsm {
namespace state {
etl::fsm_state_id_t NoForce::on_enter_state() {
  globals->yaw_motor.SendInstruction(rm::device::DmMotorInstructions::kDisable);
  globals->pitch_motor.SendInstruction(rm::device::DmMotorInstructions::kDisable);
  globals->gimbal_controller.Enable(false);
  globals->led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
  globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
  return No_State_Change;
};

etl::fsm_state_id_t NoForce::on_event(const event::ControlLoop &e) {
  globals->yaw_motor.SendInstruction(rm::device::DmMotorInstructions::kDisable);
  globals->pitch_motor.SendInstruction(rm::device::DmMotorInstructions::kDisable);
  return No_State_Change;
};

etl::fsm_state_id_t NoForce::on_event(const event::ForceModeSwitch &e) { return e.target_mode; }

etl::fsm_state_id_t NoForce::on_event_unknown(const etl::imessage &) { return No_State_Change; }
}  // namespace state
}  // namespace fsm