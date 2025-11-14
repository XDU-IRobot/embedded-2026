#ifndef REFEREE_HPP
#define REFEREE_HPP

#include <librm.hpp>

namespace rm::device {
    class RcTcRefereeData : public Device {
    public:
        RcTcRefereeData() = delete;

        explicit RcTcRefereeData(rm::hal::SerialInterface &serial);

        void Begin();

        void RxCallback(const std::vector<u8> &data, u16 rx_len);

    private:
        rm::hal::SerialInterface *serial_;
    };
}

#endif  // REFEREE_HPP
