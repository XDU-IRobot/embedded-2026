#include <librm.hpp>

#include "can.h"
#include "usart.h"
#include "main.h"
#include "StateMachine.h"
extern "C" [[noreturn]] void AppMain(void) {
  for (;;) {
    __WFI();
  }
}