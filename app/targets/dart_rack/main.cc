#include <librm.hpp>

#include "main.hpp"
extern "C" [[noreturn]] void AppMain(void) {
  for (;;) {
    __WFI();
  }
}