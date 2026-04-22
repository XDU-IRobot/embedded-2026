#pragma once

#include <etl/fsm.h>
#include "globals.hpp"
#include "trajectory_generator.hpp"
#include "dynamics.hpp"

namespace fsm {

// 前向声明状态机
    class Gimbal;

// 事件和状态ID定义
    struct EventId {
        enum {
            kControlLoop,
            kForceModeSwitch,
        };
    };

    struct StateId {
        enum : etl::fsm_state_id_t {
            kNoForce,
            kManual,
            kFollowTrajectory,
            kAuto,
        };
    };

    namespace event {
        struct ControlLoop : etl::message<EventId::kControlLoop> {};
        struct ForceModeSwitch : etl::message<EventId::kForceModeSwitch> {
            explicit ForceModeSwitch(etl::fsm_state_id_t target) : target_mode(target) {}
            etl::fsm_state_id_t target_mode;
        };
    }  // namespace event

}  // namespace fsm