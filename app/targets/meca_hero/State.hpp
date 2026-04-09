#ifndef BOARDC_STATE_HPP
#define BOARDC_STATE_HPP

class StateMachine {
public:
  StateMachine() = delete;

  explicit StateMachine(int waiting_count) : waiting_count_(waiting_count), count(waiting_count) {}

  void SetWaitingCount(int count) { waiting_count_ = count; }

  enum class MainState {
    kOffline,
    kGame,
    kTest,
    kWaiting,
  };

  enum class SubState {
    kNoAct,
    kFollow,
    kAimbot,
    kSnipe,
    kRadarAimbot,
  };

  void MainStateUpdate();

  void SubStateUpdate();

  [[nodiscard]] MainState getMainState() const { return current_main_state_; };

  [[nodiscard]] SubState getSubState() const { return current_sub_state_; };

private:
  MainState current_main_state_{MainState::kOffline};
  MainState last_main_state_{MainState::kOffline};
  SubState current_sub_state_{SubState::kNoAct};
  SubState last_sub_state_{SubState::kNoAct};

  int waiting_count_{0};
  int count {0};
};
#endif  // BOARDC_STATE_HPP