#include "test_gb/gimbal.hpp"
#include "globals.hpp"

namespace test_gb {

GlobalWarehouse *globals{nullptr};

void MainLoop() { rm::device::DjiMotor<>::SendCommand(); }

extern "C" [[noreturn]] void AppMain(void) {
  globals = new GlobalWarehouse;
  globals->Init();

  using rm::device::DR16;
  globals->rc_l_switch_watcher.OnValueChange(
      [&](const DR16::SwitchPosition &old_value, const DR16::SwitchPosition &new_value) {});
  globals->rc_r_switch_watcher.OnValueChange(
      [&](const DR16::SwitchPosition &old_value, const DR16::SwitchPosition &new_value) {});

  // 创建主循环定时任务，定频1khz
  TimerTask mainloop_500hz{
      &htim13,                                   //
      etl::delegate<void()>::create<MainLoop>()  //
  };
  mainloop_500hz.SetPrescalerAndPeriod(168 - 1, 1000 - 1);
  mainloop_500hz.Start();

  for (;;) {
    __WFI();
  }
}

}  // namespace test_gb