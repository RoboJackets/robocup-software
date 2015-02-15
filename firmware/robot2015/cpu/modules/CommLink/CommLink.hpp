#ifndef COMMUNICATION_LINK_H
#define COMMUNICATION_LINK_H

#include "mbed.h"
#include "cmsis_os.h"
#include "RTP.hpp"
#include "ThreadHelper.hpp"
#include "MailHelper.hpp"
#include "CommModule.hpp"

#define COMM_LINK_TX_QUEUE_SIZE 3
#define COMM_LINK_RX_QUEUE_SIZE 3
#define COMM_LINK_SIGNAL_START_THREAD   0x01
#define COMM_LINK_SIGNAL_TX_TRIGGER   0x02
#define COMM_LINK_SIGNAL_RX_TRIGGER   0x04
#define COMM_LINK_SIGNAL_MODULE_LINKED   0x08
#define COMM_LINK_BUFFER_SIZE   64

/**
 * CommLink Class used as the hal (hardware abstraction layer) module for interfacing communication links to the higher-level firmware
 */
class CommLink
{
public:
    /// Constructor
    CommLink(){};
    
    /// Constructor
    CommLink(PinName, PinName, PinName, PinName = NC, PinName = NC);

    /// Deconstructor
    virtual ~CommLink(){}; // Don't forget to include deconstructor implementation in derived classes that frees memory

    // Class constants for the data queue sizes
    static const int TX_QUEUE_SIZE;
    static const int RX_QUEUE_SIZE;

    /// Perform a soft reset for a communication link's hardware device
    virtual void reset(void) = 0;
    
    /// Perform tests to determine if the hardware is able to properly function
    virtual int32_t selfTest(void) = 0;
    
    /// Determine if communication can occur with another device
    virtual bool isConnected(void) = 0;
    
    /// Let the class know of the higher-level communication functions by passing it the CommModule object to use
    void setModule(CommModule&);

    /// Send a packet
    void sendPacket(RTP_t*);

    /// Receive a packet
    void receivePacket(RTP_t*);

protected:
    /// Send data to a hardware device defined in derived classes
    virtual int32_t sendData(uint8_t*, uint8_t) = 0;

    /// Get data from a hardware device defined in derived classes
    virtual int32_t getData(uint8_t*, uint8_t*) = 0;

    /// Interrupt Service Routine implemented in the base class for proper threading
    void ISR(void);

    /// Toggle the "Chip Select" pin
    void toggle_cs(void);
    
    /// Used for giving derived classes a standaradized way to inform the base class that it is ready for communication and to begin the threads
    void ready(void);   // Always call CommLink::ready() after derived class is ready for communication

    /// Initialize the SPI bus from the class's members
    void setup_spi(void);

    // The data queues for temporarily holding received packets and packets that need to be transmitted
    osMailQId   _txQueue;
    osMailQId   _rxQueue;

    // Pin names
    PinName     _miso_pin;
    PinName     _mosi_pin;
    PinName     _sck_pin;
    PinName     _cs_pin;    // CS pin
    PinName     _int_pin;   // Interrupt pin

    // Pin objects
    SPI         *_spi;      // SPI pointer
    DigitalOut  *_cs;       // Chip Select pointer
    InterruptIn *_int_in;    // Interrupt pin

private:
    // Used to help define the class's threads in the constructor
    friend void define_thread(osThreadDef_t&, void(*task)(void const *arg), osPriority, uint32_t, unsigned char*);

    // Data queues
    MailHelper<RTP_t, COMM_LINK_TX_QUEUE_SIZE>   _txQueueHelper;
    MailHelper<RTP_t, COMM_LINK_RX_QUEUE_SIZE>   _rxQueueHelper;
    
    // Thread definitions and IDs
    osThreadDef_t   _txDef;
    osThreadDef_t   _rxDef;
    osThreadId      _txID;
    osThreadId      _rxID;

    // The working threads for handeling RX/TX data queue operations
    static void txThread(void const*);
    static void rxThread(void const*);

    // Methods for initializing a transceiver's pins for communication
    void setup(void);
    void setup_pins(PinName = NC, PinName = NC, PinName = NC, PinName = NC, PinName = NC);
    void setup_cs(void);
    void setup_interrupt(void);
    
    // Used for tracking the number of link-level communication interfaces
    static unsigned int _nbr_links;

    uint8_t buf[COMM_LINK_BUFFER_SIZE];

    CommModule     *_comm_module;
};

#endif  // COMMUNICATION_LINK_H
