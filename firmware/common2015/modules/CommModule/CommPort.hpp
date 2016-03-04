#pragma once

#include <mbed.h>
#include <rtos.h>
#include "logger.hpp"

#include <algorithm>
#include <map>
#include <stdexcept>
#include <functional>

template <class T>
class CommPort {
public:
    CommPort(std::function<T> rxC = nullptr, std::function<T> txC = nullptr)
        : _rxCallback(rxC), _txCallback(txC){};

    /// Counters for the number of packets sent/received via this port
    unsigned int rxCount = 0, txCount = 0;

    // Set functions for each RX/TX callback.
    void setRxCallback(const std::function<T>& func) { _rxCallback = func; }
    void setTxCallback(const std::function<T>& func) { _txCallback = func; }

    // Methods that return a reference to the TX/RX callback function pointers
    std::function<T>& rxCallback() { return _rxCallback; }
    std::function<T>& txCallback() { return _txCallback; }

    // Returns the current packet counts to zero
    void resetPacketCount() {
        rxCount = 0;
        txCount = 0;
    }

    // Standard display function for a CommPort
    void printPort() const {
        printf("%u\t%u\t%s\t\t%s\r\n", rxCount, txCount,
               _rxCallback != nullptr ? "YES" : "NO",
               _txCallback != nullptr ? "YES" : "NO");

        Console::Instance()->Flush();
    }

private:
    // the class members that hold the function pointers
    std::function<T> _rxCallback, _txCallback;
};
