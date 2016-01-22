#pragma once

#include <mbed.h>
#include <rtos.h>

#include "rj-macros.hpp"
#include "rtp.hpp"
#include "helper-funcs.hpp"
#include "rtos-mgmt/mail-helper.hpp"
#include "CommModule.hpp"

#define FOREACH_COMM_ERR(ERR) \
    ERR(COMM_SUCCESS)         \
    ERR(COMM_FAILURE)         \
    ERR(COMM_DEV_BUF_ERR)     \
    ERR(COMM_FUNC_BUF_ERR)    \
    ERR(COMM_FALSE_TRIG)      \
    ERR(COMM_NO_DATA)

/**
 * CommLink Error Levels.
 */
enum { FOREACH_COMM_ERR(GENERATE_ENUM) };

/**
 * CommLink Class used as the hal (hardware abstraction layer) module for
 * interfacing communication links to the higher-level firmware
 */
class CommLink {
public:
    /// Defautl Constructor
    CommLink(){};

    /// Constructor
    CommLink(PinName, PinName, PinName, PinName = NC, PinName = NC);

    /// Virtual deconstructor
    /// Always define and call CommLink::cleanup() in a derived class's
    /// deconstructor!
    virtual ~CommLink();

    // Class constants for data queues
    static const size_t RX_QUEUE_SIZE = 2;
    // static const size_t BUFFER_SIZE = 64;

    // The pure virtual methods for making CommLink an abstract class
    /// Perform a soft reset for a communication link's hardware device
    virtual void reset(void) = 0;

    /// Perform tests to determine if the hardware is able to properly function
    virtual int32_t selfTest(void) = 0;

    /// Determine if communication can occur with another device
    virtual bool isConnected(void) = 0;

    /// Send & Receive through the rtp structure
    void sendPacket(rtp::packet*);

    void receivePacket(rtp::packet*);

protected:
    virtual int32_t sendData(
        uint8_t*, uint8_t) = 0;  // write data out to the radio device using SPI
    virtual int32_t getData(
        uint8_t*,
        uint8_t*) = 0;  // read data in from the radio device using SPI

    /// Kill any threads and free the allocated stack.
    /// Always call in any derived class's deconstructors!
    void cleanup();
    void ISR();
    void toggle_cs();

    /// Used for giving derived classes a standaradized way to inform the base
    /// class that it is ready for communication and to begin the threads
    //
    // Always call CommLink::ready() after derived class is ready
    void ready();
    void setup_spi(int baudrate = DEFAULT_BAUD);
    uint8_t twos_compliment(uint8_t val);

    // The data queues for temporarily holding received packets
    osMailQId _rxQueue;

    // SPI bus pins
    PinName _miso_pin;
    PinName _mosi_pin;
    PinName _sck_pin;
    PinName _cs_pin;   // CS pin
    PinName _int_pin;  // Interrupt pin

    SPI* _spi;             // SPI pointer
    DigitalOut* _cs;       // Chip Select pointer
    InterruptIn* _int_in;  // Interrupt pin

    static const int DEFAULT_BAUD = 5000000;

private:
    // Used to help define the class's threads in the constructor
    friend void define_thread(osThreadDef_t&, void (*task)(void const* arg),
                              osPriority, uint32_t);

    /**
     * Data queue helper for RX queue.
     */
    MailHelper<rtp::packet, RX_QUEUE_SIZE> _rxQueueHelper;

    // Thread definitions and IDs
    osThreadDef_t _rxDef;
    osThreadId _rxID;

    // The working threads for handeling RX data queue operations
    static void rxThread(void const*);

    // Methods for initializing a transceiver's pins for communication
    void setup(void);
    void setup_pins(PinName = NC, PinName = NC, PinName = NC, PinName = NC,
                    PinName = NC);
    void setup_cs(void);
    void setup_interrupt(void);
};
