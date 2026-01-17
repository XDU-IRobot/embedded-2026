#include "Referee.hpp"

namespace rm::device {
    RxReferee::RxReferee(rm::hal::SerialInterface &serial) : serial_(&serial) {
        static rm::hal::SerialRxCallbackFunction rx_callback =
            std::bind(&RxReferee::RxCallback, this, std::placeholders::_1, std::placeholders::_2);
        this->serial_->AttachRxCallback(rx_callback);
    }

    void RxReferee::Begin() { this->serial_->Begin(); }

    void RxReferee::RxCallback(const std::vector<u8> &data, u16 rx_len) {
        // Heartbeat();
        for (u16 i = 0; i < rx_len; i++) {
            gimbal->referee_data_buffer << data.at(i);
        }
    }
}  // namespace rm::device