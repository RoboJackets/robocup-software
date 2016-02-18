#pragma once

#include <mbed.h>
#include <rtos.h>

#include <algorithm>
#include <vector>
#include <functional>

template <class T>
class CommPort {
public:
    CommPort(uint8_t number = 0, std::function<T> rxC = nullptr,
             std::function<T> txC = nullptr)
        : _isOpen(false), _rxCallback(rxC), _txCallback(txC) {
        Nbr(number);
        resetPacketCount();
    };

    // Compare between 2 CommPort objects
    bool operator==(const CommPort& p) const {
        return (this->Nbr() == p.Nbr());
    }
    // Compare between a CommPort object and a port number
    bool operator==(unsigned int portNbr) const {
        return (this->Nbr() == portNbr ? true : false);
    }

    // Overload the less than operator for sorting/finding ports using iterators
    bool operator<(const CommPort& p) const {
        return (this->Nbr() < p.Nbr() ? true : false);
    }

    uint8_t Nbr() const { return _nbr; }

    bool valid() const { return _nbr != 0; }

    void Nbr(uint8_t nbr) { _nbr = nbr; }

    // Open a port or check if a port is capable of providing communication.
    bool Open() {
        if (isReady()) {
            _isOpen = true;
            return true;
        } else {
            return false;
        }
    }

    void Close() { _isOpen = false; }

    // Check if the port has already been opened.
    bool isOpen() const { return _isOpen; }

    // Set functions for each RX/TX callback.
    void setRxCallback(const std::function<T>& func) {
        _rxCallback = func;
        return *this;
    }
    void setTxCallback(const std::function<T>& func) {
        _txCallback = func;
        return *this;
    }

    // Methods that return a reference to the TX/RX callback function pointers
    std::function<T>& RXCallback() { return _rxCallback; }
    std::function<T>& TXCallback() { return _txCallback; }

    // Check if an RX/TX callback function has been set for the port.
    bool hasTXCallback() const { return _txCallback != nullptr; }
    bool hasRXCallback() const { return _rxCallback != nullptr; }

    // Check if the port object is a valid port
    // this will be false when indexing a non-existent port number
    bool Exists() const {
        return (hasRXCallback() || hasTXCallback()) && valid();
    }

    // Get a value or reference to the TX/RX packet count for modifying
    unsigned int txCount() const { return _txCount; }
    unsigned int rxCount() const { return _rxCount; }

    void incTxCount() { _txCount++; }
    void incRxCount() { _rxCount++; }

    // Standard display function for a CommPort
    void PrintPort() const {
        printf("%2u\t\t%u\t%u\t%s\t\t%s\t\t%s\r\n", Nbr(), rxCount(), txCount(),
               hasRXCallback() ? "YES" : "NO", hasTXCallback() ? "YES" : "NO",
               isOpen() ? "OPEN" : "CLOSED");

        Console::Instance()->Flush();
    }

    // Returns the current packet counts to zero
    void resetPacketCount() {
        _rxCount = 0;
        _txCount = 0;
    }

protected:
    // Returns true if the port can provide an RX callback routine
    bool isReady() const { return (_isOpen ? true : hasRXCallback()); }

private:
    // The number assigned to the port
    uint8_t _nbr;

    // If the port is open, it will also be valid
    bool _isOpen;

    // Where each upstream & downstream packet count is stored
    unsigned int _rxCount, _txCount;

    // the class members that hold the function pointers
    std::function<T> _rxCallback;
    std::function<T> _txCallback;
};

// Function for defining how to sort a std::vector of CommPort objects
template <class T>
bool PortCompare(const CommPort<T>& a, const CommPort<T>& b) {
    return a < b;
}

// Class to manage the available/unavailable ports
template <class T>
class CommPorts : CommPort<T> {
private:
    typename std::vector<CommPort<T>> ports;
    typename std::vector<CommPort<T>>::iterator pIt;
    CommPort<T> blackhole_port;

    const CommPorts<T>& sort() {
        std::sort(ports.begin(), ports.end(), PortCompare<T>);
        return *this;
    }

    const CommPorts<T>& add(const CommPort<T>& p) {
        pIt = find(ports.begin(), ports.end(), p);

        if (pIt == ports.end()) {
            ports.push_back(p);
        }

        return this->sort();
    }

public:
    // constructor
    CommPorts<T>() {
        // create a null port that we can return for invalid indexing for the []
        // operator
        blackhole_port = CommPort<T>();
    }

    CommPorts<T> operator+=(const CommPort<T>& p) {
        return this->add(CommPort<T>(p));
    }

    CommPort<T>& operator[](const int portNbr) {
        pIt = find(ports.begin(), ports.end(), (unsigned int)portNbr);

        if (pIt != ports.end()) {
            return *&(*pIt);
        }

        return blackhole_port;
        // throw std::runtime_error("No port for the given number");
    }

    int count() const { return ports.size(); }

    int count_open() const {
        int count = 0;

        for (auto it = ports.begin(); it != ports.end(); ++it) {
            if (it->isOpen()) count++;
        }

        return count;
    }

    bool empty() const { return ports.empty(); }

    // Get the total count (across all ports) of each RX/TX packet count
    unsigned int allRXPackets() const {
        unsigned int pcks = 0;

        for (auto& port : ports) {
            pcks += port.rxCount();
        }

        return pcks;
    }
    unsigned int allTXPackets() const {
        unsigned int pcks = 0;

        for (auto& port : ports) {
            pcks += port.txCount();
        }

        return pcks;
    }

    void PrintPorts() {
        if (empty() == false) {
            PrintHeader();

            for (auto& port : ports) {
                port.PrintPort();
            }
        }
    }

    void PrintHeader() {
        printf("PORT\t\tIN\tOUT\tRX CBCK\t\tTX CBCK\t\tSTATE\r\n");
        Console::Instance()->Flush();
    }

    void PrintFooter() {
        printf(
            "==========================\r\n"
            "Total:\t\t%u\t%u\r\n",
            allRXPackets(), allTXPackets());
        Console::Instance()->Flush();
    }
};
