#ifndef BOARDC_STATE_HPP
#define BOARDC_STATE_HPP
class State_Management {
  State_Management() = default;
  enum class main_state {
    Offline,
    Game,
  };
};
#endif  // BOARDC_STATE_HPP