#pragma once

#include "../fsm_common.hpp"

namespace fsm {
    namespace state {

        struct Manual : etl::fsm_state<Gimbal, Manual, StateId::kManual,  //
                event::ForceModeSwitch,              //
                event::ControlLoop>
        {
            etl::fsm_state_id_t on_enter_state();
            etl::fsm_state_id_t on_event(const event::ControlLoop& e);
            etl::fsm_state_id_t on_event(const event::ForceModeSwitch& e);
            etl::fsm_state_id_t on_event_unknown(const etl::imessage&);
        private:
            float rc_pitch_tar{0},rc_yaw_tar{0};

        };


    }  // namespace state
}  // namespace fsm