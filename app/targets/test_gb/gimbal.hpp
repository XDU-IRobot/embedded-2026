
#pragma once

#include <etl/fsm.h>

#include "controllers/gimbal_2dof.hpp"

namespace test_gb {

using namespace rm::modules::angle_literals;

//***************************************************************************
//// Events
namespace fsm_events {
struct EventId {
  enum {
    kModeSwitchRequest,  ///< 模式切换请求，消息由遥控器模块发出
    kManualCommand,      ///< 手动控制命令，消息由遥控器模块或键鼠模块发出
    kAimbotCommand,      ///< 自瞄控制命令，消息由自瞄模块发出
    kStateUpdate,        ///< 当前姿态更新，消息由姿态解算模块发出
    kControlLoopTick,    ///< 控制周期到达，消息由定时器模块发出
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

//***********************************
struct StateUpdate : etl::message<EventId::kStateUpdate> {
  float pitch;
  float pitch_speed;
  float yaw;
  float yaw_speed;
};

//***********************************
struct ControlLoopTick : etl::message<EventId::kControlLoopTick> {};
}  // namespace fsm_events

constexpr etl::message_router_id_t kGimbalFsmId = 0;

struct Gimbal : etl::fsm {
  Gimbal() : fsm(kGimbalFsmId) {}
  void LogUnknownEvent(const etl::imessage& msg) {
    // do nothing
  }

  const auto& output() const { return output_; }

  // protected:
  Gimbal2Dof controller_;
  fsm_events::StateUpdate current_state_;
  struct {
    float yaw_torque;
    float pitch_torque;
  } output_;
};

//***************************************************************************
//// States
namespace fsm_states {

// COMMON EVENT HANDLERS
#define IGNORE_UNKNOWN_EVENT                                         \
  etl::fsm_state_id_t on_event_unknown(const etl::imessage& event) { \
    get_fsm_context().LogUnknownEvent(event);                        \
    return No_State_Change;                                          \
  }

#define ACCEPT_MODE_SWITCH_REQUEST \
  etl::fsm_state_id_t on_event(const fsm_events::ModeSwitchRequest& event) { return event.mode_; }

#define DO_CONTROL_LOOP                                                                                           \
  etl::fsm_state_id_t on_event(const fsm_events::ControlLoopTick&) {                                              \
    const auto& current_state = get_fsm_context().current_state_;                                                 \
    auto& controller = get_fsm_context().controller_;                                                             \
    controller.Update(current_state.yaw, current_state.yaw_speed, current_state.pitch, current_state.pitch_speed, \
                      0.002f);                                                                                    \
                                                                                                                  \
    const auto& output = controller.output();                                                                     \
    get_fsm_context().output_.yaw_torque = output.yaw;                                                            \
    get_fsm_context().output_.pitch_torque = output.pitch;                                                        \
    return No_State_Change;                                                                                       \
  }

#define ACCEPT_STATE_UPDATE                                            \
  etl::fsm_state_id_t on_event(const fsm_events::StateUpdate& event) { \
    get_fsm_context().current_state_ = event;                          \
    return No_State_Change;                                            \
  }

//***********************************
struct Manual : etl::fsm_state<Gimbal, Manual,                          //
                               fsm_events::ModeSwitchRequest::kManual,  //
                               fsm_events::ModeSwitchRequest,           //
                               fsm_events::ManualCommand> {
  etl::fsm_state_id_t on_enter_state() {
    get_fsm_context().controller_.Enable(true);
    return No_State_Change;
  }
  etl::fsm_state_id_t on_event(const fsm_events::ManualCommand& event) {
    auto& controller = get_fsm_context().controller_;
    auto [yaw, _1, pitch, _2] = controller.state();
    if (event.is_relative_) {
      pitch = rm::modules::Clamp(pitch + event.euler_rpy(1), -(36._deg).rad(), (36._deg).rad());
      yaw = rm::modules::Wrap(yaw + event.euler_rpy(2), -M_PI, M_PI);
      controller.SetTarget(yaw, pitch);
    }
    return No_State_Change;
  }
  DO_CONTROL_LOOP;
  ACCEPT_MODE_SWITCH_REQUEST;
  IGNORE_UNKNOWN_EVENT;
  ACCEPT_STATE_UPDATE;
};

//***********************************
struct Aimbot : etl::fsm_state<Gimbal, Aimbot,                          //
                               fsm_events::ModeSwitchRequest::kAimbot,  //
                               fsm_events::ModeSwitchRequest,           //
                               fsm_events::AimbotCommand> {
  etl::fsm_state_id_t on_enter_state() {
    get_fsm_context().controller_.Enable(true);
    return No_State_Change;
  }
  DO_CONTROL_LOOP;
  ACCEPT_MODE_SWITCH_REQUEST;
  IGNORE_UNKNOWN_EVENT;
  ACCEPT_STATE_UPDATE;
};

//***********************************
struct NoForce : etl::fsm_state<Gimbal, NoForce,                          //
                                fsm_events::ModeSwitchRequest::kNoForce,  //
                                fsm_events::ModeSwitchRequest> {
  etl::fsm_state_id_t on_enter_state() {
    get_fsm_context().controller_.Enable(false);
    return No_State_Change;
  }
  DO_CONTROL_LOOP;
  ACCEPT_MODE_SWITCH_REQUEST;
  IGNORE_UNKNOWN_EVENT;
  ACCEPT_STATE_UPDATE;
};

}  // namespace fsm_states

}  // namespace test_gb
