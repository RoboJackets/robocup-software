#pragma once

#include <mbed.h>
#include <rtos.h>

#include "rj-macros.hpp"
#include "rtp.hpp"
#include "rtos-mgmt/thread-helper.hpp"
#include "rtos-mgmt/mail-helper.hpp"
#include "CommModule.hpp"


#define COMM_LINK_RX_QUEUE_SIZE         3
#define COMM_LINK_SIGNAL_START_THREAD   0x01
#define COMM_LINK_SIGNAL_RX_TRIGGER     0x02
#define COMM_LINK_BUFFER_SIZE           64


/**
 * CommLink Error Levels.
 */
#define FOREACH_COMM_ERR(ERR_FUNC)   \
    ERR_FUNC(COMM_SUCCESS)           \
    ERR_FUNC(COMM_FAILURE)           \
    ERR_FUNC(COMM_DEV_BUF_ERR)       \
    ERR_FUNC(COMM_FUNC_BUF_ERR)      \
    ERR_FUNC(COMM_FALSE_TRIG)        \
    ERR_FUNC(COMM_NO_DATA)
enum { FOREACH_COMM_ERR(GENERATE_ENUM) };


/**
 * CommLink Class used as the hal (hardware abstraction layer) module for
 * interfacing communication links to the higher-level firmware.
 */
class CommLink
{
public:
    /// Constructor
    CommLink(PinName mosi, PinName miso, PinName sck, PinName cs, PinName int_pin);

    /// Deconstructor
    virtual ~CommLink() {};

    // Class constants for the data queue sizes
    static const int RX_QUEUE_SIZE;

    /// Perform a soft reset for a communication link's hardware device
    virtual void reset() = 0;

    /// Perform tests to determine if the hardware is able to properly function
    virtual int32_t selfTest() = 0;

    /// Determine if communication can occur with another device
    virtual bool isConnected() = 0;

    /// Send & Receive through the rtp structure
    void sendPacket(rtp::packet* pkt);
    void receivePacket(rtp::packet* pktOut);

protected:
    virtual int32_t sendData(const uint8_t* buf, uint8_t len) = 0;    // write data out to the radio device using SPI
    virtual int32_t getData(uint8_t* buf, uint8_t* lenOut) = 0;   // read data in from the radio device using SPI

    void ISR();

    // the chip must be "selected" before beginning SPI communication, then
    // deselected afterwards.  Note: chip-select is active-low.
    void chip_select() { _cs = 0; }
    void chip_deselect() { _cs = 1; }

    /// Used for giving derived classes a standaradized way to inform the base class that it is ready for communication and to begin the threads
    void ready();   // Always call CommLink::ready() after derived class is ready for communication
    void setup_spi();

    // The data queues for temporarily holding received packets
    osMailQId   _rxQueue;

    PinName _miso_pin;

    // ============== PIN OBJECTS ==============
    SPI         _spi;      // SPI
    DigitalOut  _cs;       // Chip Select
    InterruptIn _int_in;   // Interrupt pin

private:
    // Used to help define the class's threads in the constructor
    friend void define_thread(osThreadDef_t&, void(*task)(void const* arg), osPriority, uint32_t, unsigned char*);

    /**
     * Data queue helper for RX queue.
     */
    MailHelper<rtp::packet, COMM_LINK_RX_QUEUE_SIZE> _rxQueueHelper;

    // Thread definitions and IDs
    osThreadDef_t   _rxDef;
    osThreadId      _rxID;

    // The working threads for handeling RX data queue operations
    static void rxThread(void const*);


    uint8_t buf[COMM_LINK_BUFFER_SIZE];
};
