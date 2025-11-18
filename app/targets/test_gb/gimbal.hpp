
#pragma once

#include <etl/fsm.h>

#include "controllers/gimbal_2dof.hpp"

namespace test_gimbal {

constexpr etl::message_router_id_t kGimbalFsmId = 0;

class Gimbal : public etl::fsm {
 public:
  Gimbal() : fsm(kGimbalFsmId) {}
  void LogUnknownEvent(const etl::imessage& msg) {}

 protected:
  Gimbal2Dof controller_;
};

//***************************************************************************
//// Events
namespace fsm_events {
struct EventId {
  enum {
    kModeSwitchRequest,  ///< 模式切换请求，消息由遥控器模块发出
    kManualCommand,      ///< 手动控制命令，消息由遥控器模块或键鼠模块发出
    kAimbotCommand,      ///< 自瞄控制命令，消息由自瞄模块发出
  };
};

//***********************************
struct ModeSwitchRequest : etl::message<EventId::kModeSwitchRequest> {
  enum Mode {
    kManual,
    kAimbot,
    kNoForce,
  } mode_{kNoForce};
};

//***********************************
struct ManualCommand : etl::message<EventId::kManualCommand> {
  ManualCommand(float roll, float pitch, float yaw, bool is_relative = true)
      : is_relative_(is_relative), euler_rpy(roll, pitch, yaw) {}

  bool is_relative_;  ///< 绝对姿态还是相对增量
  Eigen::Vector3f euler_rpy;
};

//***********************************
struct AimbotCommand : etl::message<EventId::kAimbotCommand> {
  AimbotCommand(float roll, float pitch, float yaw, bool is_relative = true)
      : is_relative_(is_relative), euler_rpy(roll, pitch, yaw) {}

  bool is_relative_;  ///< 绝对姿态还是相对增量
  Eigen::Vector3f euler_rpy;
};

}  // namespace fsm_events

//***************************************************************************
//// States
namespace fsm_states {
class Manual : public etl::fsm_state<Gimbal, Manual, fsm_events::ModeSwitchRequest::kManual,
                                     fsm_events::ModeSwitchRequest, fsm_events::ManualCommand> {
 public:
  //***********************************
  etl::fsm_state_id_t on_enter_state() { return No_State_Change; }
  //***********************************
  void on_exit_state() { get_fsm_context(); }
  etl::fsm_state_id_t on_event(const fsm_events::ModeSwitchRequest& event) { return event.mode_; }
  etl::fsm_state_id_t on_event_unknown(const etl::imessage& event) {
    get_fsm_context().LogUnknownEvent(event);
    return No_State_Change;
  }
};
class Aimbot : public etl::fsm_state<Gimbal, Aimbot, fsm_events::ModeSwitchRequest::kAimbot,
                                     fsm_events::ModeSwitchRequest, fsm_events::AimbotCommand> {
 public:
  etl::fsm_state_id_t on_event(const fsm_events::ModeSwitchRequest& event) { return event.mode_; }
  etl::fsm_state_id_t on_event_unknown(const etl::imessage& event) {
    get_fsm_context().LogUnknownEvent(event);
    return No_State_Change;
  }
};
class NoForce
    : public etl::fsm_state<Gimbal, NoForce, fsm_events::ModeSwitchRequest::kNoForce, fsm_events::ModeSwitchRequest> {
 public:
  etl::fsm_state_id_t on_event(const fsm_events::ModeSwitchRequest& event) { return event.mode_; }
  etl::fsm_state_id_t on_event_unknown(const etl::imessage& event) {
    get_fsm_context().LogUnknownEvent(event);
    return No_State_Change;
  }
};
}  // namespace fsm_states

}  // namespace test_gimbal