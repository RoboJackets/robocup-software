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
template <class DIGITAL_OUT = mbed::DigitalOut>
class SharedSPIDevice {
public:
    SharedSPIDevice(std::shared_ptr<SharedSPI> spi, DIGITAL_OUT cs,
                    bool csInverted = true)
        : _spi(spi), _cs(cs) {
        ASSERT(spi != nullptr);

        /// The value we set the chip select pin to in order to assert it (it's
        /// often inverted).
        _csAssertValue = csInverted ? 0 : 1;

        /// Initialize to a de-asserted state
        _cs = !_csAssertValue;
    }

    void chipSelect() {
        _spi->lock();
        _spi->frequency(_frequency);
        _cs = _csAssertValue;
    }

    void chipDeselect() {
        _cs = _csAssertValue;
        _spi->unlock();
    }

    /// Set the SPI frequency for this device
    void setSPIFrequency(int hz) { _frequency = hz; }

protected:
    std::shared_ptr<SharedSPI> _spi;
    DIGITAL_OUT _cs;

private:
    int _csAssertValue;

    /// The SPI bus frequency used by this device.
    /// This default value is the same as the mbed's default (1MHz).
    int _frequency = 1000000;
};
