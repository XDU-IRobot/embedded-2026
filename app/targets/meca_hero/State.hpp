#pragma once

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
    kNormal,
    kOverPower,  // 加速和上坡2in1
    kSnipe,
  };

  enum class ChassisState {
    kOffline,
    kNoForce,
    kNormal,
    kFollow,
  };

  enum class GimbalState {
    kOffline,
    kNoForce,
    kNormal,
    kAimbot,
    kRadarAimbot,
  };

  enum class AmmoState {
    kOffline,
    kNoForce,
    kNormal,
    kSnipe,
  };

  void StateUpdate();
  void DT7Switch();

  [[nodiscard]] MainState getMainState() const { return current_main_state_; };
  [[nodiscard]] SubState getSubState() const { return current_sub_state_; };
  [[nodiscard]] ChassisState getChassisState() const { return current_chassis_state_; };
  [[nodiscard]] GimbalState getGimbalState() const { return current_gimbal_state_; };
  [[nodiscard]] AmmoState getAmmoState() const { return current_ammo_state_; };

 private:
  MainState current_main_state_{MainState::kOffline};
  MainState last_main_state_{MainState::kOffline};

  SubState current_sub_state_{SubState::kNormal};
  SubState last_sub_state_{SubState::kNormal};

  ChassisState current_chassis_state_{ChassisState::kOffline};
  ChassisState last_chassis_state_{ChassisState::kNormal};

  AmmoState current_ammo_state_{AmmoState::kNormal};
  AmmoState last_ammo_state_{AmmoState::kNormal};

  GimbalState current_gimbal_state_{GimbalState::kOffline};
  GimbalState last_gimbal_state_{GimbalState::kNormal};

  int waiting_count_{0};
  int count_{0};
};