#pragma once

#include <mbed.h>
#include <USBDevice.h>
#include <USBEndpoints.h>
#include <functional>

/** Subclass of USBDevice to customize usb descriptors and setup two bulk
 * endpints, one IN and one OUT.
 */
class RJBaseUSBDevice : public USBDevice {
public:
    RJBaseUSBDevice(uint16_t vendor_id, uint16_t product_id,
                    uint16_t product_release)
        : USBDevice(vendor_id, product_id, product_release) {}

    /**
     * Callback functions that must be defined in order for control transfers to
     * work.  These are called in an ISR context by the USBCallback_request()
     * method.
     */
    std::function<void(uint8_t reg, uint8_t val)> writeRegisterCallback =
        nullptr;
    std::function<void(uint8_t strobe)> strobeCallback = nullptr;
    std::function<uint8_t(uint8_t reg)> readRegisterCallback = nullptr;

    /**
    * Called for control transfers.  If it's a VENDOR request, we handle it. The
    * types of requests that will come through this callback include requests to
    * set radio registers, send radio commands, etc
    *
    * @return true if the request was handled.  False indicates to the usb
    *     subsystem that it should be handled elsewhere
    */
    virtual bool USBCallback_request();

    virtual bool USBCallback_setConfiguration(uint8_t configuration);

    /// override configuration and string descriptors
    uint8_t* stringImanufacturerDesc();
    uint8_t* stringIproductDesc();
    uint8_t* configurationDesc();

private:
    uint8_t _controlTransferReplyValue;
};
