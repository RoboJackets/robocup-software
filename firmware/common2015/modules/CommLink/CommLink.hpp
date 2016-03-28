#pragma once

#include <mbed.h>
#include <rtos.h>

#include "rj-macros.hpp"
#include "rtp.hpp"
#include "helper-funcs.hpp"
#include "rtos-mgmt/mail-helper.hpp"
#include "CommModule.hpp"
#include "SharedSPI.hpp"

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
    /// Default Constructor
    CommLink();

    /// Constructor
    CommLink(std::shared_ptr<SharedSPI> spiBus, PinName nCs = NC,
             PinName int_pin = NC);

    /// Virtual deconstructor
    /// Kills any threads and frees the allocated stack.
    virtual ~CommLink() {}

    // Class constants for data queues
    static const size_t RX_QUEUE_SIZE = 2;

    // The pure virtual methods for making CommLink an abstract class
    /// Perform a soft reset for a communication link's hardware device
    virtual void reset() = 0;

    /// Perform tests to determine if the hardware is able to properly function
    virtual int32_t selfTest() = 0;

    /// Determine if communication can occur with another device
    virtual bool isConnected() const = 0;

    /// Send & Receive through the rtp structure
    void sendPacket(rtp::packet*);

protected:
    // write data out to the radio device using SPI
    virtual int32_t sendData(const uint8_t* buf, uint8_t len) = 0;

    /**
     * @brief Read data from the radio's RX buffer
     *
     * Copies the contents of the RX buffer into the given @buf parameter.
     *
     * @param buf The buffer to write data into
     *
     * @return An error/success code.  See the comm error enum above.
     */
    virtual int32_t getData(std::vector<uint8_t>* buffer) = 0;

    /// Interrupt Service Routine - KEEP OPERATIONS TO ABSOLUTE MINIMUM HERE AND
    /// IN ANY OVERRIDDEN BASE CLASS IMPLEMENTATIONS OF THIS CLASS METHOD
    void ISR();

    /// Activate the chip select pin and acquire a lock on the shared spi bus
    void radio_select();

    /// Deactivate the chip select pin and release a lock on the shared spi bus
    void radio_deselect();

    /// Used for giving derived classes a standaradized way to inform the base
    /// class that it is ready for communication and to begin the threads
    //
    // Always call CommLink::ready() after derived class is ready
    void ready();

    template <typename T>
    T twos_compliment(T val) {
        return ~val + 1;
    }

    std::shared_ptr<SharedSPI> _spi;
    DigitalOut _nCs;
    InterruptIn _int_in;

private:
    Thread _rxThread;

    // The working thread for handling RX data queue operations
    void rxThread();

    static void rxThreadHelper(const void* linkInst) {
        CommLink* link = (CommLink*)linkInst;
        link->rxThread();
    }
};
