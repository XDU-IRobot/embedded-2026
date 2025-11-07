#include "referee.hpp"
#include "main.cc"

RcTcRefereeData::RcTcRefereeData(rm::hal::SerialInterface &serial) : serial_(&serial) {
    static rm::hal::SerialRxCallbackFunction rx_callback =
            std::bind(&RcTcRefereeData::RxCallback, this, std::placeholders::_1, std::placeholders::_2);
    this->serial_->AttachRxCallback(rx_callback);
}

void RcTcRefereeData::Begin() { this->serial_->Begin(); }

void RcTcRefereeData::RxCallback(const std::vector<u8> &data, u16 rx_len) {
    for (u16 i = 0; i < rx_len; i++) {
        referee_data_buffer << data.at(i);
    }
}
