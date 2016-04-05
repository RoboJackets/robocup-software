#include <hidapi.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <wchar.h>
#include "../usb-interface.hpp"

int main(int argc, char* argv[]) {
    // print out info for all usb devices connected
    hid_device* handle;
    struct hid_device_info *devices, *currentDevice;
    devices = hid_enumerate(0x00, 0x00);
    currentDevice = devices;
    while (currentDevice) {
        printf("Device Found\n");
        printf("  type: %04hx %04hx\n", currentDevice->vendor_id,
               currentDevice->product_id);
        printf("  Manufacturer: %ls\n", currentDevice->manufacturer_string);
        printf("  Product:      %ls\n", currentDevice->product_string);
        printf("  Release:      %hx\n", currentDevice->release_number);
        printf("  Interface:    %d\n", currentDevice->interface_number);
        printf("  Serial #:     %ls\n", currentDevice->serial_number);
        printf("  Path: %s\n", currentDevice->path);
        currentDevice = currentDevice->next;
    }
    hid_free_enumeration(devices);

    printf("\n");

    // attempt to connect to base station
    handle = hid_open(RJ_BASE2015_VENDOR_ID, RJ_BASE2015_PRODUCT_ID, nullptr);
    if (handle == NULL) {
        printf("unable to open device\r\n");
        return -1;
    }

    // repeatedly send a character to the device, iterating from 'A' to 'Z'
    printf("opened device\r\n");
    unsigned char buf[64];

    // this value as byte 0 indicates VENDOR_TYPE.  Without it, the transfer
    // isn't passed to our custom mbed program's functions and instead gets
    // eaten by lower-level mbed code in USBHID/USBDevice.
    buf[0] = 2 << 5;

    buf[1] = 'A';
    while (true) {
        buf[2] = '\0';
        hid_write(handle, buf, strlen((const char*)buf));
        sleep(1);
        printf("wrote string '%s'\n", (char*)buf);
        buf[1]++;
        if (buf[1] > 'Z') buf[1] = 'A';
    }

    return 0;
}
