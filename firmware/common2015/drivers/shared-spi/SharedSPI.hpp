#pragma once

#include <mbed.h>
#include <rtos.h>

#include <memory>

#include "assert.hpp"

/**
 * A simple wrapper over mbed's SPI class that includes a mutex.
 *
 * This makes it easier to correctly use a shared SPI bus (with multiple
 * devices) in a multi-threaded environment.
 */
class SharedSPI : public mbed::SPI, public Mutex {
public:
    SharedSPI(PinName mosi, PinName miso, PinName sck) : SPI(mosi, miso, sck) {}
};


/**
 * Classes that provide an interface to a device on the shared spi bus should
 * inherit from this class.
 */
template<class DIGITAL_OUT = mbed::DigitalOut>
class SharedSPIDevice {
public:
    SharedSPIDevice(std::shared_ptr<SharedSPI> spi, DIGITAL_OUT cs,
                    bool csInverted = true)
        : _spi(spi), _cs(cs) {

        /// The value we set the chip select pin to in order to assert it (it's
        /// often inverted).
        _csAssertValue = csInverted ? 0 : 1;

        /// Initialize to a de-asserted state
        _cs = !_csAssertValue;
    }

    void chip_select() {
        _spi->lock();
        _cs = _csAssertValue;
    }

    void chip_deselect() {
        _cs = _csAssertValue;
        _spi->unlock();
    }

protected:
    std::shared_ptr<SharedSPI> _spi;
    DIGITAL_OUT _cs;

private:
    int _csAssertValue;
};
