//
// Created by 34236 on 2025/11/25.
//
/* 处理状态机相关的逻辑 */

#ifndef BOARDC_DART_STATEMACHINE_HH
#define BOARDC_DART_STATEMACHINE_HH

#include "dart_core.hpp"

void DartStateMachineUpdate(DartState &state);

void DartStateManualUpdate();

void DartStateAdjustUpdate();
#endif  // BOARDC_DART_STATEMACHINE_HH
