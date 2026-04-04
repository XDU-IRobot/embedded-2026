#ifndef CMS_H
#define CMS_H

#include "librm.hpp"

using namespace rm;
using namespace rm::device;

const u32 CAP_BUFFER_ID = 0x02E;
const u32 CAP_POWER_ID = 0x02F;
const u32 CAP_FEDBACK_ID = 0x030;

const u16 OUTPUTPOWER = 360;
const u16 INPUTPOWER = 120;

class CMS final : public CanDevice {
 public:
  explicit CMS(hal::CanInterface &can);
  CMS() = delete;
  ~CMS() override = default;

  void RxCallback(const hal::CanFrame *msg) override;
  void SendCapBuffer(u16 power);
  void SendCapPower(u16 powerlimit);

 private:
  u8 tx_buf_[8]{0};

 public:
  float cms_i{0};
  float cms_v{0};
  u16 cms_status{0};
  u32 offline_counter{0};
};

inline int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min) {
  int16_t b = (a - a_min) / (a_max - a_min) * (float)(b_max - b_min) + (float)b_min + 0.5f;
  return b;
}

inline float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min) {
  float b = (float)(a - a_min) / (float)(a_max - a_min) * (b_max - b_min) + b_min;
  return b;
}

#endif /* CMS_H */