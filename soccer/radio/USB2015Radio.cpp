#define _USB2015RADIO_DEBUG

#include "USB2015Radio.hpp"
#include <hidapi.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <wchar.h>

USB2015Radio::USB2015Radio(const unsigned int PRODUCT_ID, const unsigned int VENDOR_ID) {
        if (!hid_init()) {
            //change to exception
            std::cerr << "Cannot initialize the HID interface for radio 2015" << std::endl;
            return;
        }

#ifdef _USB2015RADIO_DEBUG
    struct hid_device_info *devices, *currentDevice;
    devices = hid_enumerate(0x00, 0x00);
    currentDevice = devices;
    while (currentDevice) {
        printf("Device Found\n  type: %04hx %04hx\n",
                currentDevice->vendor_id, 
                currentDevice->product_id);
        printf("  Manufacturer: %ls\n", currentDevice->manufacturer_string);
        printf("  Product:      %ls\n", currentDevice->product_string);
        printf("  Release:      %hx\n", currentDevice->release_number);
        printf("  Interface:    %d\n",  currentDevice->interface_number);
        currentDevice = currentDevice->next;    
    }
    hid_free_enumeration(devices);
#endif  

    hid_device *handle = hid_open(VENDOR_ID, PRODUCT_ID, NULL);
    if (handle == NULL) {
        //throw exception
    }

    base = handle;
}

USB2015Radio::~USB2015Radio() {
    hid_close(base);
}

bool USB2015Radio::isOpen() const {
    return false;
}

void USB2015Radio::receive() {
    return;
}

void USB2015Radio::send(Packet::RadioTx& pkt) {
    return;
}

void USB2015Radio::switchTeam(bool blueTeam) {
    std::cerr << "Error: switching teams on non-simulation radio" << std::endl;
}
