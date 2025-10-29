
#pragma once

#include "tim.h"

class LED {
 public:
  void Init();

  void operator()(uint32_t argb);
};