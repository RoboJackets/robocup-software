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

#define COMM_MODULE_TX_QUEUE_SIZE           5
#define COMM_MODULE_RX_QUEUE_SIZE           5
#define COMM_MODULE_NBR_PORTS               16
#define COMM_MODULE_SIGNAL_START_THREAD     0x01

class CommLink;

// Base class for a communication module
class CommModule
{
public:
    /// Default Constructor
    CommModule();

    // Deconstructor
    virtual ~CommModule();

    // Class constants - set in CommModule.cpp
    static const int NBR_PORTS;
    static const int TX_QUEUE_SIZE;
    static const int RX_QUEUE_SIZE;

    // Open a socket connection for communicating.
    template <typename T>
    void TxHandler(T *tptr, void(T::*mptr)(RTP_t*), uint8_t portNbr) {
        _txH_called[portNbr] = true;
        ready();
        _tx_handles[portNbr].attach(tptr, mptr);
    }
    
    template <typename T>
    void RxHandler(T *tptr, void(T::*mptr)(RTP_t*), uint8_t portNbr) {
        _rxH_called[portNbr] = true;
        ready();
        _rx_handles[portNbr].attach(tptr, mptr);
    }

    void TxHandler(void(*)(RTP_t*), uint8_t);
    void RxHandler(void(*)(RTP_t*), uint8_t);
    
    void RxHandler(void(*)(void), uint8_t);

    void openSocket(uint8_t);

    // Send a RTP packet. The details of exactly how the packet will be sent are determined from the RTP packet's port and subclass values
    void send(RTP_t&);
    void receive(RTP_t&);
    
    //osThreadId rxID(void);

protected:
    // NOP function for keeping a oommunication link active
    void nopFunc(void);

    // Memory Queue IDs
    osMailQId   _txQueue;
    osMailQId   _rxQueue;

    // Thread IDs
    osThreadId      _txID;
    osThreadId      _rxID;

    std::vector<uint8_t> *_open_ports;

private:
    // Used to help define the class's threads in the constructor
    friend void define_thread(osThreadDef_t&, void(*task)(void const *arg), osPriority, uint32_t, unsigned char*);

    // The working threads for handeling rx and tx data queues
    static void txThread(void const*);
    static void rxThread(void const*);

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

    CommLink        *_link[COMM_MODULE_NBR_PORTS];

    FunctionPointerRJ   _rx_handles[COMM_MODULE_NBR_PORTS];
    FunctionPointerRJ   _tx_handles[COMM_MODULE_NBR_PORTS];
    
    bool    _txH_called[COMM_MODULE_NBR_PORTS];
    bool    _rxH_called[COMM_MODULE_NBR_PORTS];

    // Ignore for now
    // bool _dynamic_stack;
};

