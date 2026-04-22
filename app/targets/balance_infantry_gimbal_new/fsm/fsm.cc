
#include "fsm.hpp"

// fsm&state instances


namespace fsm {
    Gimbal gimbal;
    namespace state {
        NoForce no_force_state;
        Manual manual_state;
        FollowTrajectory follow_trajectory_state;
        Auto auto_state;
    }  // namespace state

    void Init() {
        static etl::ifsm_state *state_list[] = {
                &state::no_force_state,           //
                &state::manual_state,             //
                &state::follow_trajectory_state,  //
                &state::auto_state,                //

        };
        gimbal.set_states(state_list, std::size(state_list));
        gimbal.start();
    }

}  // namespace fsm