#include <librm.hpp>
#include "dart_core.hpp"
#include "main.hpp"

extern "C" [[noreturn]] void AppMain(void) {
  dart_rack = new DartRack();
  DartRack::Init();
  for (;;) {
    __WFI();
  }
}