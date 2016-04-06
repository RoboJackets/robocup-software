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

    virtual bool USBCallback_setConfiguration(uint8_t configuration) {
        // NOTE: the below was copied from USBCDC.cpp
        
        // // Configure endpoints > 0
        // addEndpoint(EPINT_IN, MAX_PACKET_SIZE_EPINT);
        // addEndpoint(EPBULK_IN, MAX_PACKET_SIZE_EPBULK);
        // addEndpoint(EPBULK_OUT, MAX_PACKET_SIZE_EPBULK);

        // // We activate the endpoint to be able to recceive data
        // readStart(EPBULK_OUT, MAX_PACKET_SIZE_EPBULK);

        return true;
    };

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

#define CONFIG1_DESC_SIZE (9 + 8 + 9 + 5 + 5 + 4 + 5 + 7 + 9 + 7 + 7)

    uint8_t* configurationDesc() {
        static uint8_t configDescriptor[] = {
            // configuration descriptor
            9,                       // bLength
            2,                       // bDescriptorType
            LSB(CONFIG1_DESC_SIZE),  // wTotalLength
            MSB(CONFIG1_DESC_SIZE),
            2,     // bNumInterfaces
            1,     // bConfigurationValue
            0,     // iConfiguration
            0x80,  // bmAttributes
            50,    // bMaxPower

            // IAD to associate the two CDC interfaces
            0x08,  // bLength
            0x0b,  // bDescriptorType
            0x00,  // bFirstInterface
            0x02,  // bInterfaceCount
            0x02,  // bFunctionClass
            0x02,  // bFunctionSubClass
            0,     // bFunctionProtocol
            0,     // iFunction

            // interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
            9,     // bLength
            4,     // bDescriptorType
            0,     // bInterfaceNumber
            0,     // bAlternateSetting
            1,     // bNumEndpoints
            0x02,  // bInterfaceClass
            0x02,  // bInterfaceSubClass
            0x01,  // bInterfaceProtocol
            0,     // iInterface

            // CDC Header Functional Descriptor, CDC Spec 5.2.3.1, Table 26
            5,           // bFunctionLength
            0x24,        // bDescriptorType
            0x00,        // bDescriptorSubtype
            0x10, 0x01,  // bcdCDC

            // Call Management Functional Descriptor, CDC Spec 5.2.3.2, Table 27
            5,     // bFunctionLength
            0x24,  // bDescriptorType
            0x01,  // bDescriptorSubtype
            0x03,  // bmCapabilities
            1,     // bDataInterface

            // Abstract Control Management Functional Descriptor, CDC Spec
            // 5.2.3.3, Table 28
            4,     // bFunctionLength
            0x24,  // bDescriptorType
            0x02,  // bDescriptorSubtype
            0x06,  // bmCapabilities

            // Union Functional Descriptor, CDC Spec 5.2.3.8, Table 33
            5,     // bFunctionLength
            0x24,  // bDescriptorType
            0x06,  // bDescriptorSubtype
            0,     // bMasterInterface
            1,     // bSlaveInterface0

            // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
            ENDPOINT_DESCRIPTOR_LENGTH,  // bLength
            ENDPOINT_DESCRIPTOR,         // bDescriptorType
            PHY_TO_DESC(EPINT_IN),       // bEndpointAddress
            E_INTERRUPT,                 // bmAttributes (0x03=intr)
            LSB(MAX_PACKET_SIZE_EPINT),  // wMaxPacketSize (LSB)
            MSB(MAX_PACKET_SIZE_EPINT),  // wMaxPacketSize (MSB)
            16,                          // bInterval

            // interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
            9,     // bLength
            4,     // bDescriptorType
            1,     // bInterfaceNumber
            0,     // bAlternateSetting
            2,     // bNumEndpoints
            0x0A,  // bInterfaceClass
            0x00,  // bInterfaceSubClass
            0x00,  // bInterfaceProtocol
            0,     // iInterface

            // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
            ENDPOINT_DESCRIPTOR_LENGTH,   // bLength
            ENDPOINT_DESCRIPTOR,          // bDescriptorType
            PHY_TO_DESC(EPBULK_IN),       // bEndpointAddress
            E_BULK,                       // bmAttributes (0x02=bulk)
            LSB(MAX_PACKET_SIZE_EPBULK),  // wMaxPacketSize (LSB)
            MSB(MAX_PACKET_SIZE_EPBULK),  // wMaxPacketSize (MSB)
            0,                            // bInterval

            // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
            ENDPOINT_DESCRIPTOR_LENGTH,   // bLength
            ENDPOINT_DESCRIPTOR,          // bDescriptorType
            PHY_TO_DESC(EPBULK_OUT),      // bEndpointAddress
            E_BULK,                       // bmAttributes (0x02=bulk)
            LSB(MAX_PACKET_SIZE_EPBULK),  // wMaxPacketSize (LSB)
            MSB(MAX_PACKET_SIZE_EPBULK),  // wMaxPacketSize (MSB)
            0                             // bInterval
        };
        return configDescriptor;
    }
};
