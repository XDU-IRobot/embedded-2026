
#include "fsm.hpp"

// fsm&state instances
namespace fsm {
Sentry sentry;
namespace state {
static Unable unable;
static NoForce no_force;
static Test test;
static Match match;
}  // namespace state

void Init() {
  static etl::ifsm_state *state_list[] = {
      &state::unable,    //
      &state::no_force,  //
      &state::test,      //
      &state::match,     //
  };
  sentry.set_states(state_list, std::size(state_list));
  sentry.start();
}

}  // namespace fsm