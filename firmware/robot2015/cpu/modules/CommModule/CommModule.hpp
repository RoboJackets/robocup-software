#pragma once

#include "mbed.h"
#include "cmsis_os.h"
#include "RTP.hpp"
#include "ThreadHelper.hpp"
#include "MailHelper.hpp"
#include "logger.hpp"
#include "FunctionPointerRJ.hpp"

#include <algorithm>    // std::binary_search, std::sort
#include <vector>

#define COMM_MODULE_TX_QUEUE_SIZE           20
#define COMM_MODULE_RX_QUEUE_SIZE           4
#define COMM_MODULE_NBR_PORTS               15
#define COMM_MODULE_SIGNAL_START_THREAD     0x01

class CommLink;

/**
 *  The CommModule class acts as the interface to provide an abstraction layer between the user and hardware for communications.
 */
class CommModule
{
public:
    /// Default Constructor
    CommModule();

    /// Deconstructor
    virtual ~CommModule();

    // Class constants - set in CommModule.cpp
    static const int NBR_PORTS;
    static const int TX_QUEUE_SIZE;
    static const int RX_QUEUE_SIZE;

    /// Bind a member function to call on an object when sending a packet
    template <typename T>
    void TxHandler(T *tptr, void(T::*mptr)(RTP_t*), uint8_t portNbr) {
        _txH_called = true;
        ready();
        _tx_handles[portNbr].attach(tptr, mptr);
    }

    /// Bind a callback function to call when sending a packet
    void TxHandler(void(*)(RTP_t*), uint8_t);

    /// Bind a member functi on to call on an object when receiving a packet
    template <typename T>
    void RxHandler(T *tptr, void(T::*mptr)(RTP_t*), uint8_t portNbr) {
        ready();
        _rx_handles[portNbr].attach(tptr, mptr);
    }

    /// Bind a callback function to call when receiving a packet
    void RxHandler(void(*)(RTP_t*), uint8_t);

    /// Start listening on a port nuber that has been initialized
    void openSocket(uint8_t);

    /// Send a packet according to the passed packet's values
    void send(RTP_t&);

    /// Receive a packet according to the passed packet's values
    void receive(RTP_t&);

protected:
    /// NOP function for keeping a oommunication link active
    void nopFunc(void);

    // Memory Queue IDs
    osMailQId   _txQueue;
    osMailQId   _rxQueue;

    // Thread IDs
    osThreadId      _txID;
    osThreadId      _rxID;

    /// Vector container used for keeping track of the opened ports
    std::vector<uint8_t> *_open_ports;

private:
    // Used to help define the class's threads in the constructor
    friend void define_thread(osThreadDef_t&, void(*task)(void const *arg), osPriority, uint32_t, unsigned char*);

    // The working threads for handeling rx and tx data queues
    static void txThread(void const*);
    static void rxThread(void const*);

    // Used for starting the thread operations
    void ready(void);
    static bool isReady;

    // Thread and Mail defintion data structures
    osThreadDef_t   _txDef;
    osThreadDef_t   _rxDef;
    osMailQDef_t    _txQDef;
    osMailQDef_t    _rxQDef;

    // Mail helper objects
    MailHelper<RTP_t, COMM_MODULE_TX_QUEUE_SIZE>   _txQueueHelper;
    MailHelper<RTP_t, COMM_MODULE_RX_QUEUE_SIZE>   _rxQueueHelper;

    // Array of the link-layer object pointers
    CommLink        *_link[COMM_MODULE_NBR_PORTS];

    // Array of callback function pointers
    FunctionPointerRJ   _rx_handles[COMM_MODULE_NBR_PORTS];
    FunctionPointerRJ   _tx_handles[COMM_MODULE_NBR_PORTS];
    
    bool    _txH_called;
    bool    _rxH_called;
};

