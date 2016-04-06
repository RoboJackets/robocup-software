#include "RJBaseUSBDevice.hpp"
#include <USBEndpoints.h>
#include <USBDescriptor.h>
#include <USBDevice_Types.h>

bool RJBaseUSBDevice::USBCallback_setConfiguration(uint8_t configuration) {
    // Configuration 1 is our only configuration
    if (configuration != 1) return false;

    addEndpoint(EPBULK_OUT, MAX_PACKET_SIZE_EPBULK);
    addEndpoint(EPBULK_IN, MAX_PACKET_SIZE_EPBULK);

    // We activate the endpoint to be able to recceive data
    readStart(EPBULK_OUT, MAX_PACKET_SIZE_EPBULK);

    return true;
}

uint8_t* RJBaseUSBDevice::stringImanufacturerDesc() {
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

uint8_t* RJBaseUSBDevice::stringIproductDesc() {
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

// Used for putting words in descriptor byte arrays
#define WORD(x) ((x)&0xff), (((x) >> 8) & 0xff)

#define CONFIG_SIZE 9
#define INTF_SIZE 9
#define EP_SIZE 7

// The contents of this method were copied from the usb config descriptor in
// the old base station's firmware.  See 'device.c' for context.
uint8_t* RJBaseUSBDevice::configurationDesc() {
    static uint8_t configurationDescriptor[] = {
        // Configuration
        CONFIG_SIZE,  // bLength
        2,            // bDescriptorType
        WORD(         // wTotalLength
            CONFIG_SIZE + INTF_SIZE + EP_SIZE * 2),
        1,     // bNumInterfaces
        1,     // bConfigurationValue
        0,     // iConfiguration
        0xc0,  // bmAttributes
        0,     // MaxPower

        // Interface 0
        INTF_SIZE,  // bLength
        4,          // bDescriptorType
        0,          // bInterfaceNumber
        0,          // bAlternateSetting
        2,          // bNumEndpoints (not counting endpoint 0)
        0xff,       // bInterfaceClass
        0xff,       // bInterfaceSubclass
        0,          // bInterfaceProtocol
        0,          // iInterface

        // Endpoint 1: bulk OUT
        EP_SIZE,                       // bLength
        5,                             // bDescriptorType
        1,                             // bEndpointAddress
        2,                             // bmAttributes
        WORD(MAX_PACKET_SIZE_EPBULK),  // wMaxPacketSize
        0,                             // bInterval

        // Endpoint 2: bulk IN
        EP_SIZE,                       // bLength
        5,                             // bDescriptorType
        0x82,                          // bEndpointAddress
        2,                             // bmAttributes
        WORD(MAX_PACKET_SIZE_EPBULK),  // wMaxPacketSize
        0                              // bInterval
    };
    return configurationDescriptor;
}
