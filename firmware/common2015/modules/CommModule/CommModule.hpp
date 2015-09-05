#pragma once

#include <mbed.h>
#include <rtos.h>

#include "rtp.hpp"
#include "rtos-mgmt/thread-helper.hpp"
#include "rtos-mgmt/mail-helper.hpp"
#include "Console.hpp"
#include "CommPort.hpp"

#include <algorithm>
#include <vector>
#include <functional>
#include <memory>


#define COMM_MODULE_TX_QUEUE_SIZE           5
#define COMM_MODULE_RX_QUEUE_SIZE           5
#define COMM_MODULE_NBR_PORTS               16
#define COMM_MODULE_SIGNAL_START_THREAD     0x01


/* These define the function pointer type that's used for every callback
 * function type set through the CommModule class.
 */
typedef void                        (FunctionPtr_t)(rtp::packet*);
typedef CommPort<FunctionPtr_t>     CommPort_t;
typedef CommPorts<FunctionPtr_t>    CommPorts_t;


// forward declaration of the template is needed for the class
extern CommPort_t _tmpPort;


/**
 * The CommModule class provides the packet management routing
 * by distributing incoming packets to the correct area of the
 * firmware and distributing outgoing packets to the correct
 * hardware interface.
 */


/**
 * @brief      { A high-level firmware class for packet handling & routing }
 */
class CommModule
{
private:
    static CommPorts_t      _ports;

public:
    // Class constants - set in CommModule.cpp
    static const int NBR_PORTS;
    static const int TX_QUEUE_SIZE;
    static const int RX_QUEUE_SIZE;

    static void Init(void);

    // Set a TX callback function on an object
    template <typename B>
    static void TxHandler(B* obj, void(B::*mptr)(rtp::packet*), uint8_t portNbr)
    {
        if ( !_ports[portNbr].Exists() ) {

            CommPort_t _tmpPort(portNbr);

            _tmpPort.TXCallback() = std::bind(mptr, obj, std::placeholders::_1);

            _ports += _tmpPort;

        } else  {

            _ports[portNbr].TXCallback() = std::bind(mptr, obj, std::placeholders::_1);
        }

        ready();
    }

    // Set an RX callback function on an object
    template <typename B>
    static void RxHandler(B* obj, void(B::*mptr)(rtp::packet*), uint8_t portNbr)
    {
        if ( !_ports[portNbr].Exists() ) {

            CommPort_t _tmpPort(portNbr);

            _tmpPort.RXCallback() = std::bind(mptr, obj, std::placeholders::_1);

            _ports += _tmpPort;

        } else {

            _ports[portNbr].RXCallback() = std::bind(mptr, obj, std::placeholders::_1);
        }

        ready();
    }

    // Set a normal RX callback function without an object
    static void RxHandler(void(*ptr)(rtp::packet*), uint8_t);
    static void TxHandler(void(*ptr)(rtp::packet*), uint8_t);

    // Open a socket connection for communicating.
    static bool openSocket(uint8_t);

    // Send a rtp::packet. The details of exactly how the packet will be sent are determined from the rtp::packet's port and subclass values
    static void send(rtp::packet&);
    static void receive(rtp::packet&);

    static unsigned int NumRXPackets(void);
    static unsigned int NumTXPackets(void);

    static void PrintInfo(bool forceHeader = false);

    static void ResetCount(unsigned int portNbr);
    static void Close(unsigned int portNbr);
    static bool isReady(void);
    static int  NumOpenSockets(void);

    static void txLED(DigitalInOut*);
    static void rxLED(DigitalInOut*);

protected:
    // NOP function for keeping a communication link active
    void nopFunc(void);

    // Memory Queue IDs
    osMailQId   _txQueue;
    osMailQId   _rxQueue;

    // Thread IDs
    static osThreadId      _txID;
    static osThreadId      _rxID;

    DigitalInOut* _rxLED;
    DigitalInOut* _txLED;

private:
    // Private constructor
    CommModule();

    static shared_ptr<CommModule>& Instance(void);

    // Used to help define the class's threads in the constructor
    friend void define_thread(osThreadDef_t&, void(*task)(void const* arg), osPriority, uint32_t, unsigned char*);

    // The working threads for handeling rx and tx data queues
    static void txThread(void const*);
    static void rxThread(void const*);

    static void ready(void);

    static void PrintHeader(void);

    static std::shared_ptr<CommModule> instance;

    static bool _isReady;

    // Thread and Mail defintion data structures
    osThreadDef_t   _txDef;
    osThreadDef_t   _rxDef;
    osMailQDef_t    _txQDef;
    osMailQDef_t    _rxQDef;

    // Mail helper objects
    MailHelper<rtp::packet, COMM_MODULE_TX_QUEUE_SIZE>   _txQueueHelper;
    MailHelper<rtp::packet, COMM_MODULE_RX_QUEUE_SIZE>   _rxQueueHelper;
};
