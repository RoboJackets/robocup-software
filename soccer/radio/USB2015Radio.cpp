#pragma once

#define _USB2015RADIO_DEBUG

#include "USB2015Radio.hpp"
#include <hidapi.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <wchar.h>

USB2015Radio::USB2015Radio(const unsigned int PRODUCT_ID, const unsigned int VENDOR_ID) {
    if (!hidAPIInit) {
        hidAPIInit = (hid_init() == 0 ? true : false);
        if (!hidAPIInit) {}
            //change to exception
            std::cerr << "Cannot initialize the HID interface for radio 2015" << std::endl;
        }
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

    hid_device *handle = hid_open(VENDIR_ID, PRODUCT_ID, NULL);
    if (handle == NULL) {
        //throw exception
    }

    base = handle;
    numInstances++;
}

USB2015Radio::~USB2015Radio() {
    hid_close(base);
    if (--numInstances == 0) {
        hidAPIInit = static_cast<bool>(hid_exit());
    }
}

bool USB2015Radio::isOpen() {
    return false;
}