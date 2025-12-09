
/* 处理状态机相关的逻辑 */

#ifndef BOARDC_DART_STATEMACHINE_HH
#define BOARDC_DART_STATEMACHINE_HH

#include "dart_core.hpp"

void DartStateMachineUpdate(DartState &state);
void DartStateManualUpdate();
void DartStateAdjustUpdate();
void DartStateUnableUpdate();

void DartStateInitUpdate();
void DartStateLoadUpdate();
void DartStateAddUpdate();
void DartStateAimUpdate();
void DartStateFireUpdate();


#endif  // BOARDC_DART_STATEMACHINE_HH
