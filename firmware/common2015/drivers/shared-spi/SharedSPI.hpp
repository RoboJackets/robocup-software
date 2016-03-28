#pragma once

#include <mbed.h>
#include <rtos.h>
#include "assert.hpp"

/**
 * A simple wrapper over mbed's SPI class that includes a mutex.
 *
 * This makes it easier to correctly use a shared SPI bus (with multiple
 * devices) in a multi-threaded environment.
 */
class SharedSPI : public mbed::SPI {
public:
    SharedSPI(PinName mosi, PinName miso, PinName sck) : SPI(mosi, miso, sck) {}

    osStatus lock(uint32_t millisec = osWaitForever) {
        return _mutex.lock(millisec);
    }

    bool trylock() { return _mutex.trylock(); }

    osStatus unlock() { return _mutex.unlock(); }

private:
    Mutex _mutex;
};
