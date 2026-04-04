
#include "fsm.hpp"

// fsm&state instances
namespace fsm {
Gimbal gimbal;
namespace state {
static NoForce no_force;
static Manual manual;
static FollowTrajectory follow_trajectory;
}  // namespace state

void Init() {
  static etl::ifsm_state *state_list[] = {
      &state::no_force,           //
      &state::manual,             //
      &state::follow_trajectory,  //
  };
  gimbal.set_states(state_list, std::size(state_list));
  gimbal.start();
}

}  // namespace fsm