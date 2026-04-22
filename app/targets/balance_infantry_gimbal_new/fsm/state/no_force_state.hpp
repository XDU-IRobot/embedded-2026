#pragma once

#include "../fsm_common.hpp"

namespace fsm {
namespace state {

struct NoForce : etl::fsm_state<Gimbal, NoForce, StateId::kNoForce,  //
                                event::ForceModeSwitch,              //
                                event::ControlLoop> {
  etl::fsm_state_id_t on_enter_state();
  etl::fsm_state_id_t on_event(const event::ControlLoop& e);
  etl::fsm_state_id_t on_event(const event::ForceModeSwitch& e);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage&);
};

}  // namespace state
}  // namespace fsm