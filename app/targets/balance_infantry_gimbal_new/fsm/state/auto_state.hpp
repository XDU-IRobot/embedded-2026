#pragma once

#include "../fsm_common.hpp"

namespace fsm {
namespace state {

struct Auto : etl::fsm_state<Gimbal, Auto, StateId::kAuto,  //
                             event::ForceModeSwitch,        //
                             event::ControlLoop> {
  etl::fsm_state_id_t on_enter_state();
  etl::fsm_state_id_t on_event(const event::ControlLoop& e);
  etl::fsm_state_id_t on_event(const event::ForceModeSwitch& e);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage&);

  float auto_pitch_tar{0}, auto_yaw_tar{0};
  float yaw_ff_{0}, pitch_ff_{0};
};

}  // namespace state
}  // namespace fsm
