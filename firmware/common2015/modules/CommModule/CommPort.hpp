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
    CommPort(uint8_t number = 0, std::function<T> rxC = nullptr,
             std::function<T> txC = nullptr)
        : _isOpen(false), _rxCallback(rxC), _txCallback(txC) {
        portNbr(number);
        resetPacketCount();
    };

    // Compare between 2 CommPort objects
    bool operator==(const CommPort& p) const {
        return portNbr() == p.portNbr();
    }

    // Overload the less than operator for sorting/finding ports using iterators
    bool operator<(const CommPort& p) const { return portNbr() < p.portNbr(); }

    uint8_t portNbr() const { return _nbr; }
    void portNbr(uint8_t nbr) { _nbr = nbr; }

    bool valid() const { return _nbr != 0; }

    // Open a port or check if a port is capable of providing communication.
    void open() { _isOpen = true; }
    void close() { _isOpen = false; }

    bool isOpen() const { return _isOpen; }

    bool isReady() const {
        return isOpen() && hasTxCallback() && hasRxCallback();
    }

    // Set functions for each RX/TX callback.
    void setRxCallback(const std::function<T>& func) { _rxCallback = func; }
    void setTxCallback(const std::function<T>& func) { _txCallback = func; }

    // Methods that return a reference to the TX/RX callback function pointers
    std::function<T>& rxCallback() { return _rxCallback; }
    std::function<T>& txCallback() { return _txCallback; }

    // Check if an RX/TX callback function has been set for the port.
    bool hasTxCallback() const { return _txCallback != nullptr; }
    bool hasRxCallback() const { return _rxCallback != nullptr; }

    // Get a value or reference to the TX/RX packet count for modifying
    unsigned int txCount() const { return _txCount; }
    unsigned int rxCount() const { return _rxCount; }

    void incTxCount() { _txCount++; }
    void incRxCount() { _rxCount++; }

    // Standard display function for a CommPort
    void printPort() const {
        printf("%2u\t\t%u\t%u\t%s\t\t%s\t\t%s\r\n", portNbr(), rxCount(),
               txCount(), hasRxCallback() ? "YES" : "NO",
               hasTxCallback() ? "YES" : "NO", isOpen() ? "OPEN" : "CLOSED");

        Console::Instance()->Flush();
    }

    // Returns the current packet counts to zero
    void resetPacketCount() {
        _rxCount = 0;
        _txCount = 0;
    }

private:
    // The number assigned to the port
    uint8_t _nbr;

    // If the port is open, it will also be valid
    bool _isOpen;

    // Where each upstream & downstream packet count is stored
    unsigned int _rxCount, _txCount;

    // the class members that hold the function pointers
    std::function<T> _rxCallback, _txCallback;
};

/// Class to manage the available/unavailable ports
template <class T>
class CommPorts {
public:
    CommPorts<T> operator+=(const CommPort<T>& p) {
        if (_ports.find(p.portNbr()) != _ports.end()) {
            // the port already exists
            LOG(WARN, "Overwriting existing port '%d'", p.portNbr());
        }

        _ports[p.portNbr()] = p;

        return *this;
    }

    CommPort<T>& operator[](uint8_t portportNbr) { return _ports[portportNbr]; }

    bool hasPort(uint8_t portportNbr) const {
        return _ports.find(portportNbr) != _ports.end();
    }

    int count() const { return _ports.size(); }

    int countOpen() const {
        int count = 0;

        for (auto& kvpair : _ports) {
            if (kvpair.second.isOpen()) count++;
        }

        return count;
    }

    bool empty() const { return _ports.empty(); }

    // Get the total count (across all ports) of each RX/TX packet count
    unsigned int totalRxCount() const {
        unsigned int count = 0;

        for (auto& kvpair : _ports) {
            count += kvpair.second.rxCount();
        }

        return count;
    }
    unsigned int totalTxCount() const {
        unsigned int count = 0;

        for (auto& kvpair : _ports) {
            count += kvpair.second.txCount();
        }

        return count;
    }

    void printPorts() const {
        for (auto& kvpair : _ports) {
            kvpair.second.printPort();
        }
    }

    void printHeader() const {
        printf("PORT\t\tIN\tOUT\tRX CBCK\t\tTX CBCK\t\tSTATE\r\n");
        Console::Instance()->Flush();
    }

    void printFooter() const {
        printf(
            "==========================\r\n"
            "Total:\t\t%u\t%u\r\n",
            totalRxCount(), totalTxCount());
        Console::Instance()->Flush();
    }

private:
    std::map<uint8_t, CommPort<T>> _ports;
};
