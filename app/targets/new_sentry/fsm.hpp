
#pragma once

#include <etl/fsm.h>
#include <etl/random.h>

#include "main.hpp"

namespace fsm {

struct EventId {
  enum {
    kControlLoop,      ///< 控制循环时钟事件
    kForceModeSwitch,  ///< 强制切换模式命令
  };
};

struct StateId {
  enum : etl::fsm_state_id_t {
    kUnable,   ///<
    kNoForce,  ///<
    kTest,     ///<
    kMatch,    ///<
  };
};

namespace event {
/*******************************************/
struct ControlLoop : etl::message<EventId::kControlLoop> {};

/*******************************************/
struct ForceModeSwitch : etl::message<EventId::kForceModeSwitch> {
  explicit ForceModeSwitch(etl::fsm_state_id_t target) : target_mode(target) {}
  etl::fsm_state_id_t target_mode;
};

}  // namespace event

class Sentry : public etl::fsm {
  constexpr static etl::message_router_id_t kMessageRouterId = 0;

 public:
  Sentry() : fsm(kMessageRouterId) {}
};
extern Sentry sentry;

namespace state {

///*********************************
///*******  helper macros  *********
///*********************************
#define ACCEPT_MODE_SWITCH() \
  etl::fsm_state_id_t on_event(const event::ForceModeSwitch &e) { return e.target_mode; }
#define IGNORE_UNINTEREST_EVENT() \
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &) { return No_State_Change; }
#define ENTER etl::fsm_state_id_t on_enter_state()
#define REACT(EventType) etl::fsm_state_id_t on_event(const EventType &e)
///*********************************
///*********************************
///*********************************

/*******************************************/
struct Unable : etl::fsm_state<Sentry, Unable, StateId::kUnable,  //
                               event::ForceModeSwitch,            //
                               event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER { return No_State_Change; }
  REACT(event::ControlLoop) { return No_State_Change; }
};

/*******************************************/
struct NoForce : etl::fsm_state<Sentry, NoForce, StateId::kNoForce,  //
                                event::ForceModeSwitch,              //
                                event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER { return No_State_Change; }
  REACT(event::ControlLoop) { return No_State_Change; }
};

/*******************************************/
struct Test : etl::fsm_state<Sentry, Test, StateId::kTest,  //
                             event::ForceModeSwitch,        //
                             event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER { return No_State_Change; }
  REACT(event::ControlLoop) { return No_State_Change; }
};

/*******************************************/
struct Match : etl::fsm_state<Sentry, Match, StateId::kMatch,  //
                              event::ForceModeSwitch,          //
                              event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER { return No_State_Change; }
  REACT(event::ControlLoop) { return No_State_Change; }
};

#undef IGNORE_UNINTEREST_EVENT
#undef ENTER
#undef REACT

}  // namespace state

void Init();

}  // namespace fsm