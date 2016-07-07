#pragma once

#include <mbed.h>
#include <rtos.h>
#include "logger.hpp"

#include <algorithm>
#include <functional>
#include <map>
#include <stdexcept>

template <typename RX_CALLBACK, typename TX_CALLBACK>
class CommPort {
public:
    CommPort(std::function<RX_CALLBACK> rxC = nullptr,
             std::function<TX_CALLBACK> txC = nullptr)
        : _rxCallback(rxC), _txCallback(txC){};

    /// Counters for the number of packets sent/received via this port
    unsigned int rxCount = 0, txCount = 0;

    // Set functions for each RX/TX callback.
    void setRxCallback(const std::function<RX_CALLBACK>& func) {
        _rxCallback = func;
    }
    void setTxCallback(const std::function<TX_CALLBACK>& func) {
        _txCallback = func;
    }

    /// Methods that return a reference to the TX/RX callback function pointers
    std::function<RX_CALLBACK>& rxCallback() { return _rxCallback; }
    const std::function<RX_CALLBACK>& rxCallback() const { return _rxCallback; }
    std::function<TX_CALLBACK>& txCallback() { return _txCallback; }
    const std::function<TX_CALLBACK>& txCallback() const { return _txCallback; }

    // Returns the current packet counts to zero
    void resetPacketCount() {
        rxCount = 0;
        txCount = 0;
    }

private:
    // the class members that hold the function pointers
    std::function<RX_CALLBACK> _rxCallback;
    std::function<TX_CALLBACK> _txCallback;
};
