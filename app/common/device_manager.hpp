
#pragma once

#include <librm.hpp>
#include <etl/vector.h>

template <size_t MaxDevices>
class DeviceManager {
 public:
  DeviceManager &operator<<(rm::device::Device *device) {
    if (devices_.size() < MaxDevices) {
      devices_.push_back(device);
    } else {
      for (;;);  // 超出最大设备数量，进入死循环以便调试
    }
    return *this;
  }

  bool all_device_ok() const {
    return std::all_of(devices_.begin(), devices_.end(), [](rm::device::Device *dev) { return dev->IsAlive(); });
  }

 private:
  etl::vector<rm::device::Device *, MaxDevices> devices_;
};