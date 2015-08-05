#pragma once

#include "mbed.h"
#include "rtos.h"
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

void Task_CommCtrl(void const*);
void comm_cmdProcess(const vector<string>&);


template <class T>
class CommPort
{
  public:
    // Constructor
    CommPort()
        : is_valid(false),
          is_open(false),
          nbr(0xFF),
          rx_callback(nullptr),
          tx_callback(nullptr),
          rx_packets(0),
          tx_packets(0)
    {};

    CommPort(uint8_t number, std::function<T> rxC = nullptr, std::function<T> txC = nullptr)
        : is_valid(true),
          is_open(false),
          nbr(number),
          rx_callback(rxC),
          tx_callback(txC),
          rx_packets(0),
          tx_packets(0)
    {};

    virtual ~CommPort() {};

    // Copy assignment
    const CommPort& operator=(const CommPort<T>& p)
    {
        this->is_valid       = p.is_valid;
        this->is_open        = p.is_open;
        this->nbr            = p.nbr;
        this->rx_packets     = p.rx_packets;
        this->tx_packets     = p.tx_packets;
        this->rx_callback    = p.rx_callback;
        this->tx_callback    = p.tx_callback;
        return *this;
    }

    const bool operator==(const CommPort<T>& p) const
    {
        if (this->is_valid == p.Exists()) {
            if (this->is_open == p.isOpen()) {
                if (this->rx_callback == p.rx_callback) {
                    if (this->tx_callback == p.tx_callback) {
                        return true;
                    }
                }
            }
        }

        return false;
    }

    const bool operator==(unsigned int portNbr) const
    {
        return (this->nbr == portNbr ? true : false);
    }

    const uint8_t Nbr(void) const
    {
        return this->nbr;
    }

    void Nbr(uint8_t _nbr)
    {
        this->nbr = _nbr;
    }

    // Open a port or check if a port is capable of providing communication.
    bool Open(void)
    {
        if (isReady()) {
            this->is_open = true;

            return isOpen();
        } else {
            return false;
        }
    }

    // Check if the port has already been opened.
    bool isOpen(void) const
    {
        return this->is_open;
    }

    bool isClosed(void) const
    {
        return !isOpen();
    }

    // Set functions for each RX/TX callback.
    void RXCallback(const std::function<T>& func)
    {
        rx_callback = func;
    }
    void TXCallback(const std::function<T>& func)
    {
        tx_callback = func;
    }

    // Check if an RX/TX callback function has been set for the port.
    bool hasTXCallback(void) const
    {
        return ((this->tx_callback == nullptr) ? false : true);
    }
    bool hasRXCallback(void) const
    {
        return ((this->rx_callback == nullptr) ? false : true);
    }

    // Check if the port object is
    bool Exists(void) const
    {
        return this->is_valid;
    }

    // Get the packet count
    unsigned int TXPackets(void) const
    {
        return this->tx_packets;
    }

    unsigned int RXPackets(void) const
    {
        return this->rx_packets;
    }

    void TXPackets(unsigned int offset)
    {
        tx_packets += offset;
    }
    void RXPackets(unsigned int offset)
    {
        rx_packets += offset;
    }

    // the class members that hold the function pointers
    std::function<T>    rx_callback, tx_callback;

    // Returns true if the port can provide an RX callback routine.
    bool isReady(void) const
    {
        return (is_open ? true : hasRXCallback());
    }

    bool                is_valid, is_open;
    uint8_t             nbr;
    unsigned int        rx_packets, tx_packets;

  protected:

};

template<class T>
struct findPort : std::unary_function<CommPort<T>, bool> {
    uint8_t portNbr;
    findPort(uint8_t p): portNbr(p) { }
    bool operator()(const CommPort<T>& p) { return p.Nbr() == portNbr; }
};

template<class T>
bool PortSort(const CommPort<T>& a, const CommPort<T>& b)
{
    return (a.Nbr() < b.Nbr());
}

template<class T>
class CommPorts : public CommPort<T>
{
  public:
    CommPorts(void) : ports() {};

    virtual ~CommPorts(void) {};

    const CommPorts<T>& sort(void)
    {
        std::sort(ports.begin(), ports.end(), PortSort<T>);
        return *this;
    }

    const CommPorts<T>& add(const CommPort<T>& p)
    {
        pIt = find_if(ports.begin(), ports.end(), findPort<T>(p.Nbr()));

        if (pIt == ports.end()) {
            ports.push_back(p);
        }

        return this->sort();
    }

    void add(unsigned int portNum, std::function<T>& rxCall = nullptr, std::function<T>& txCall = nullptr)
    {
        if (rxCall != nullptr || txCall != nullptr) {
            return;
        } else {
            CommPort<T>* portAdd = new CommPort<T>(portNum);

            if (rxCall == nullptr) {
                portAdd->RXCallback(std::bind(rxCall, std::placeholders::_1));
            }

            if (txCall == nullptr) {
                portAdd->TXCallback(std::bind(txCall, std::placeholders::_1));
            }

            add(*portAdd);
            delete portAdd;
        }

    }

    /*
        const CommPorts& operator+ (const CommPort<T>& p)
        {
            return this->add(p);
        }
        */

    const CommPorts& operator+= (const CommPort<T>& p)
    {
        return this->add(p);
    }

    CommPort<T>& operator[](const int portNbr)
    {
        pIt = find_if(ports.begin(), ports.end(), findPort<T>(portNbr));

        if (pIt != ports.end()) {
            return *& (*pIt);
        }

        return *&(*(ports.begin()));
    }

    int count(void)
    {
        return ports.size();
    }

    bool empty(void)
    {
        return ports.empty();
    }

  private:
    typename std::vector<CommPort<T>> ports;
    typename std::vector<CommPort<T>>::iterator pIt;
};



typedef void                        (FunctionPtr_t)(RTP_t*);

typedef CommPort<FunctionPtr_t>     CommPort_t;
typedef CommPorts<FunctionPtr_t>    CommPorts_t;


extern CommPort_t      portAdd;

// Base class for a communication module
class CommModule
{
  private:
    static CommPorts_t       _ports;

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


    template <typename B>
    static void TxHandler(B* obj, void(B::*mptr)(RTP_t*), uint8_t portNbr)
    {
        if ( !_ports[portNbr].Exists() ) {

            CommPort_t portAdd(portNbr);

            portAdd.TXCallback(std::bind(mptr, obj, std::placeholders::_1));

            _ports += portAdd;

        } else if ( _ports[portNbr].hasRXCallback() ) {

            _ports[portNbr].TXCallback(std::bind(mptr, obj, std::placeholders::_1));

        } else {

            return;
        }

        ready();
    }

    template <typename B>
    static void RxHandler(B* obj, void(B::*mptr)(RTP_t*), uint8_t portNbr)
    {
        if ( !_ports[portNbr].Exists() ) {

            CommPort_t portAdd(portNbr);

            portAdd.RXCallback(std::bind(mptr, obj, std::placeholders::_1));

            _ports += portAdd;

        } else if ( _ports[portNbr].hasTXCallback() ) {

            _ports[portNbr].RXCallback(std::bind(mptr, obj, std::placeholders::_1));

        } else {

            return;
        }

        ready();
    }

    static void RxHandler(void(*ptr)(RTP_t*), uint8_t);

// Open a socket connection for communicating.
    static bool openSocket(uint8_t);

// Send a RTP packet. The details of exactly how the packet will be sent are determined from the RTP packet's port and subclass values
    static void send(RTP_t&);
    static void receive(RTP_t&);

    static unsigned int NumRXPackets(void);
    static unsigned int NumTXPackets(void);

    static void PrintInfo(bool forceHeader = false);

  protected:
// NOP function for keeping a communication link active
    void nopFunc(void);

// Memory Queue IDs
    static osMailQId   _txQueue;
    static osMailQId   _rxQueue;

// Thread IDs
    static osThreadId      _txID;
    static osThreadId      _rxID;

  private:
// Used to help define the class's threads in the constructor
    friend void define_thread(osThreadDef_t&, void(*task)(void const* arg), osPriority, uint32_t, unsigned char*);

// The working threads for handeling rx and tx data queues
    static void txThread(void const*);
    static void rxThread(void const*);

    static void ready(void);

    static bool isReady;

    static void PrintHeader(void);

// Thread and Mail defintion data structures
    osThreadDef_t   _txDef;
    osThreadDef_t   _rxDef;
    osMailQDef_t    _txQDef;
    osMailQDef_t    _rxQDef;

// Mail helper objects
    MailHelper<RTP_t, COMM_MODULE_TX_QUEUE_SIZE>   _txQueueHelper;
    MailHelper<RTP_t, COMM_MODULE_RX_QUEUE_SIZE>   _rxQueueHelper;
};
