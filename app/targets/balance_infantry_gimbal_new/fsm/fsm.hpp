#pragma once

#include "fsm_common.hpp"
#include "auto_state.hpp"
#include "no_force_state.hpp"
#include "manual_state.hpp"
#include "follow_trajectory_state.hpp"

namespace fsm {

    class Gimbal : public etl::fsm {
        constexpr static etl::message_router_id_t kMessageRouterId = 0;
    public:
        Gimbal() : fsm(kMessageRouterId) {}

        bool shoot_state{false};
    };

// 全局状态机实例
    extern Gimbal gimbal;
// 初始化函数
    void Init();

}  // namespace fsm