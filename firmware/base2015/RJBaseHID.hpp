#pragma once

#include <mbed.h>
#include <USBHID.h>

/** Subclass of USBHID to customize usb product and manufacturer strings.  The
 * descriptor methods were copied from mbed's USBDevice.cpp file and modified
 */
class RJBaseHID : public USBDevice {
public:
    RJBaseHID(uint16_t vendor_id = 0x1234, uint16_t product_id = 0x0006,
              uint16_t product_release = 0x0001)
        : USBDevice(vendor_id, product_id, product_release) {}

    uint8_t* stringImanufacturerDesc() {
        // clang-format off
        static uint8_t stringImanufacturerDescriptor[] = {
            0x18,              /*bLength*/
            STRING_DESCRIPTOR, /*bDescriptorType 0x03*/
            'R', 0, 'o', 0, 'b', 0, 'o', 0, 'J', 0, 'a', 0, 'c', 0, 'k', 0, 'e', 0, 't', 0, 's',
            0, /*bString iManufacturer - RoboJackets*/
        };
        // clang-format on
        return stringImanufacturerDescriptor;
    }

    uint8_t* stringIproductDesc() {
        // clang-format off
        static uint8_t stringIproductDescriptor[] = {
            0x14,              /*bLength*/
            STRING_DESCRIPTOR, /*bDescriptorType 0x03*/
            'B', 0, 'a', 0, 's', 0, 'e', 0, ' ', 0, '2', 0, '0', 0, '1', 0, '5',
            0 /*bString iProduct - Base 2015*/
        };
        // clang-format on
        return stringIproductDescriptor;
    }
};
