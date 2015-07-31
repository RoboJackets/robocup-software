#pragma once

#include "mbed.h"
#include "cmsis_os.h"
#include "robot_types.hpp"
#include "ThreadHelper.hpp"
#include "MailHelper.hpp"
#include "logger.hpp"

#include <algorithm>
#include <vector>
#include <functional>

#define COMM_MODULE_TX_QUEUE_SIZE           5
#define COMM_MODULE_RX_QUEUE_SIZE           5
#define COMM_MODULE_NBR_PORTS               16
#define COMM_MODULE_SIGNAL_START_THREAD     0x01

typedef std::vector<std::function<void(RTP_t*)>> func_t;

void rx_callback_test(RTP_t *);
void Task_CommCtrl(void const *);
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

    static unsigned int txPackets;
    static unsigned int rxPackets;

    template <typename T>
    static void TxHandler(T *obj, void(T::*mptr)(RTP_t *), uint8_t portNbr) {
        _txH_called[portNbr] = true;
        _link[portNbr] = obj;
        _tx_handles.at(portNbr) = std::bind(mptr, obj, std::placeholders::_1);
        ready();
    }

    template <typename T>
    static void RxHandler(T *obj, void(T::*mptr)(RTP_t *), uint8_t portNbr) {
        _rxH_called[portNbr] = true;
        _rx_handles.at(portNbr) = std::bind(mptr, obj, std::placeholders::_1);
        ready();
    }

    static void RxHandler(void(*ptr)(RTP_t *), uint8_t);

    // Open a socket connection for communicating.
    static void openSocket(uint8_t);

    // Send a RTP packet. The details of exactly how the packet will be sent are determined from the RTP packet's port and subclass values
    static void send(RTP_t &);
    static void receive(RTP_t &);

    static unsigned int NumRXPackets(void);
    static unsigned int NumTXPackets(void);

protected:
    // NOP function for keeping a communication link active
    void nopFunc(void);

    // Memory Queue IDs
    static osMailQId   _txQueue;
    static osMailQId   _rxQueue;

    // Thread IDs
    static osThreadId      _txID;
    static osThreadId      _rxID;

    static std::vector<uint8_t> *_open_ports;

private:
    // Used to help define the class's threads in the constructor
    friend void define_thread(osThreadDef_t &, void(*task)(void const *arg), osPriority, uint32_t, unsigned char *);

    // The working threads for handeling rx and tx data queues
    static void txThread(void const *);
    static void rxThread(void const *);

    static void ready(void);

    static bool isReady;

    // Thread and Mail defintion data structures
    osThreadDef_t   _txDef;
    osThreadDef_t   _rxDef;
    osMailQDef_t    _txQDef;
    osMailQDef_t    _rxQDef;

    // Mail helper objects
    MailHelper<RTP_t, COMM_MODULE_TX_QUEUE_SIZE>   _txQueueHelper;
    MailHelper<RTP_t, COMM_MODULE_RX_QUEUE_SIZE>   _rxQueueHelper;

    static CommLink *_link[COMM_MODULE_NBR_PORTS];

    static func_t   _rx_handles;
    static func_t   _tx_handles;

    static bool     _txH_called[COMM_MODULE_NBR_PORTS];
    static bool     _rxH_called[COMM_MODULE_NBR_PORTS];
};

