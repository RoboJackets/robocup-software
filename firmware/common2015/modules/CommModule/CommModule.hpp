#pragma once

#include <mbed.h>
#include <rtos.h>

#include "rtp.hpp"
#include "helper-funcs.hpp"
#include "rtos-mgmt/mail-helper.hpp"
#include "Console.hpp"
#include "CommPort.hpp"

#include <algorithm>
#include <vector>
#include <functional>
#include <memory>

/* These define the function pointer type that's used for every callback
 * function type set through the CommModule class.
 */
typedef void(CommCallback)(rtp::packet*);
typedef CommPort<CommCallback> CommPort_t;
typedef CommPorts<CommCallback> CommPorts_t;

/**
 * @brief A high-level firmware class for packet handling & routing
 *
 * The CommModule class provides the packet management routing
 * by distributing incoming packets to the correct area of the
 * firmware and distributing outgoing packets to the correct
 * hardware interface.
 */
class CommModule {
private:
    CommPorts_t _ports;

public:
    /// The destructor frees up allocated memory and stops threads
    ~CommModule();

    /// Access the singleton CommModule instance
    static shared_ptr<CommModule>& Instance();

    /// initializes and starts rx/tx threads and mail queues
    void init();

    // Class constants
    // Be careful of the queue sizes. The errors that result from
    // over allocation are very tricky to catch.
    static const size_t TX_QUEUE_SIZE = 3;
    static const size_t RX_QUEUE_SIZE = 3;

    // Set a TX callback function on an object
    template <typename B>
    void setTxHandler(B* obj, void (B::*mptr)(rtp::packet*), uint8_t portNbr) {
        if (!_ports.hasPort(portNbr)) {
            _ports += CommPort_t(portNbr);
        }

        _ports[portNbr].txCallback() =
            std::bind(mptr, obj, std::placeholders::_1);

        ready();
    }

    // Set an RX callback function on an object
    template <typename B>
    void setRxHandler(B* obj, void (B::*mptr)(rtp::packet*), uint8_t portNbr) {
        if (!_ports.hasPort(portNbr)) {
            _ports += CommPort_t(portNbr);
        }

        _ports[portNbr].rxCallback() =
            std::bind(mptr, obj, std::placeholders::_1);

        ready();
    }

    // Set a normal RX callback function without an object
    void setRxHandler(CommCallback callback, uint8_t);
    void setTxHandler(CommCallback callback, uint8_t);

    // Open a socket connection for communicating.
    void openSocket(uint8_t portNbr);

    // Send a rtp::packet. The details of exactly how the packet will be sent
    // are determined from the rtp::packet's port and subclass values
    void send(const rtp::packet&);
    void receive(const rtp::packet&);

    unsigned int numRxPackets() const;
    unsigned int numTxPackets() const;
    void resetCount(unsigned int portNbr);

    void printInfo() const;

    void close(unsigned int portNbr);
    bool isReady() const;
    int numOpenSockets() const;

protected:
    // Memory Queue IDs
    osMailQId _txQueue;
    osMailQId _rxQueue;

    // Thread IDs
    osThreadId _txID;
    osThreadId _rxID;

private:
    // The working threads for handeling rx and tx data queues
    static void txThread(void const*);
    static void rxThread(void const*);

    void ready();

    static std::shared_ptr<CommModule> instance;

    bool _isReady = false;

    // Thread and Mail defintion data structures
    osThreadDef_t _txDef;
    osThreadDef_t _rxDef;

    // Mail helper objects
    MailHelper<rtp::packet, TX_QUEUE_SIZE> _txQueueHelper;
    MailHelper<rtp::packet, RX_QUEUE_SIZE> _rxQueueHelper;
};
