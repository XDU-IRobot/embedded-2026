
#include "minipc.hpp"
#include "minipc_bridge.h"

#include "global.hpp"

extern "C" void CDCRecvCallback(uint8_t *buf, uint32_t len) { global.minipc->RxCallback(buf, len); }
