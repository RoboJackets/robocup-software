#pragma once

#include <mbed.h>
#include <rtos.h>

#include "rj-macros.hpp"
#include "robot-types.hpp"
#include "rtos-mgmt/thread-helper.hpp"
#include "rtos-mgmt/mail-helper.hpp"
#include "CommModule.hpp"


#define COMM_LINK_RX_QUEUE_SIZE         3
#define COMM_LINK_SIGNAL_START_THREAD   0x01
#define COMM_LINK_SIGNAL_RX_TRIGGER     0x02
#define COMM_LINK_BUFFER_SIZE           64


#define FOREACH_COMM_ERR(ERR)   \
    ERR(COMM_SUCCESS)           \
    ERR(COMM_FAILURE)           \
    ERR(COMM_DEV_BUF_ERR)       \
    ERR(COMM_FUNC_BUF_ERR)      \
    ERR(COMM_FALSE_TRIG)        \
    ERR(COMM_NO_DATA)


/**
 * CommLink Error Levels.
 */
enum { FOREACH_COMM_ERR(GENERATE_ENUM) };


/**
 * CommLink Class used as the hal (hardware abstraction layer) module for interfacing communication links to the higher-level firmware
 */
class CommLink
{
  public:
    /// Defautl Constructor
    CommLink(void) {};

    /// Constructor
    CommLink(PinName, PinName, PinName, PinName = NC, PinName = NC);

    /// Deconstructor
    virtual ~CommLink(void) {}; // Don't forget to include deconstructor implementation in derived classes that frees memory

    // Class constants for the data queue sizes
    static const int RX_QUEUE_SIZE;

    // The pure virtual methods for making CommLink an abstract class
    /// Perform a soft reset for a communication link's hardware device
    virtual void reset(void) = 0;

    /// Perform tests to determine if the hardware is able to properly function
    virtual int32_t selfTest(void) = 0;

    /// Determine if communication can occur with another device
    virtual bool isConnected(void) = 0;

    /// Send & Receive through the RTP structure
    void sendPacket(RTP_t*);
    void receivePacket(RTP_t*);

  protected:
    virtual int32_t sendData(uint8_t*, uint8_t) = 0;    // write data out to the radio device using SPI
    virtual int32_t getData(uint8_t*, uint8_t*) = 0;   // read data in from the radio device using SPI

    void ISR(void);
    void toggle_cs(void);

    /// Used for giving derived classes a standaradized way to inform the base class that it is ready for communication and to begin the threads
    void ready(void);   // Always call CommLink::ready() after derived class is ready for communication
    void setup_spi(void);

    // The data queues for temporarily holding received packets
    osMailQId   _rxQueue;

    // ============== PIN NAMES ==============
    // SPI bus pins
    PinName     _miso_pin;
    PinName     _mosi_pin;
    PinName     _sck_pin;
    PinName     _cs_pin;    // CS pin
    PinName     _int_pin;   // Interrupt pin

    // ============== PIN OBJECTS ==============
    SPI*         _spi;      // SPI pointer
    DigitalOut*  _cs;       // Chip Select pointer
    InterruptIn* _int_in;    // Interrupt pin

  private:
    // Used to help define the class's threads in the constructor
    friend void define_thread(osThreadDef_t&, void(*task)(void const* arg), osPriority, uint32_t, unsigned char*);

    /**
     * Data queue helper for RX queue.
     */
    MailHelper<RTP_t, COMM_LINK_RX_QUEUE_SIZE>   _rxQueueHelper;

    // Thread definitions and IDs
    osThreadDef_t   _rxDef;
    osThreadId      _rxID;

    // The working threads for handeling RX data queue operations
    static void rxThread(void const*);

    // Methods for initializing a transceiver's pins for communication
    void setup(void);
    void setup_pins(PinName = NC, PinName = NC, PinName = NC, PinName = NC, PinName = NC);
    void setup_cs(void);
    void setup_interrupt(void);

    // Used for tracking the number of link-level communication interfaces
    static unsigned int _nbr_links;

    uint8_t buf[COMM_LINK_BUFFER_SIZE];
};
