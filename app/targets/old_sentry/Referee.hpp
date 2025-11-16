#ifndef REFEREE_HPP
#define REFEREE_HPP

#include <librm.hpp>

using namespace rm;

class RcTcRefereeData {
 public:
  RcTcRefereeData() = delete;

  explicit RcTcRefereeData(rm::hal::SerialInterface &serial);

  void Begin();

  void RxCallback(const std::vector<u8> &data, u16 rx_len);

 private:
  rm::hal::SerialInterface *serial_;
};

#endif  // REFEREE_HPP