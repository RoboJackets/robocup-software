#pragma once

#include <mbed.h>
#include <USBDevice.h>

/** Subclass of USBDevice to customize usb descriptors and setup two bulk
 * endpints, one IN and one OUT.
 */
class RJBaseUSBDevice : public USBDevice {
public:
    RJBaseUSBDevice(uint16_t vendor_id = 0x1234, uint16_t product_id = 0x0006,
                    uint16_t product_release = 0x0001)
        : USBDevice(vendor_id, product_id, product_release) {}

    virtual bool USBCallback_setConfiguration(uint8_t configuration);

    /// override configuration and string descriptors
    uint8_t* stringImanufacturerDesc();
    uint8_t* stringIproductDesc();
    uint8_t* configurationDesc();
};
