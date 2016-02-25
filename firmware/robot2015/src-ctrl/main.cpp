/**
 * Program an AVR with an mbed.
 */

#include "AVR910.hpp"
#include "errno.h"

//ATTtiny
#define ATTINY84A_PAGESIZE  32 //Size in words.
#define ATTINY84A_NUM_PAGES 128

LocalFileSystem local("local");
Serial pc(USBTX, USBRX);
AVR910 mAVRISP(p5, p6, p7, p8); //mosi, miso, sclk, nreset.

int main() {
    pc.baud(57600);
    bool success  = false;
    int response =  0;

    //Read the vendor code [0x1E == Atmel].
    response = mAVRISP.readVendorCode();

    if (response == ATMEL_VENDOR_CODE) {
        pc.printf("Microcontroller is an Atmel [0x%02x]\r\n", response);
    } else if (response == DEVICE_LOCKED) {
        pc.printf("Device is locked\r\n");
        return -1;
    } else {
        pc.printf("Microcontroller is not an Atmel\r\n");
        return -1;
    }

    //Read part family and flash size - see datasheet for code meaning.
    response = mAVRISP.readPartFamilyAndFlashSize();

    if (response == 0xFF) {
        pc.printf("Device code erased or target missing\r\n");
    } else if (response == 0x01) {
        pc.printf("Device locked\r\n");
        return -1;
    } else {
        pc.printf("Part family and flash size code is: 0x%02x\r\n", response);
    }

    //Read part number.
    response = mAVRISP.readPartNumber();

    if (response == 0xFF) {
        pc.printf("Device code erased or target missing\r\n");
    } else if (response == 0x02) {
        pc.printf("Device locked\r\n");
        return -1;
    } else {
        pc.printf("Part number code is: 0x%02x\r\n", response);
    }

    //Open binary file to write to AVR.
    FILE *fp = fopen("/local/rj-kickr.nib", "r");
    if (fp == NULL) {
        pc.printf("Failed to open binary. Please check the file path\r\n");
        return -1;
    } else {
        //Program it!
        pc.printf("Binary file opened successfully\r\n");
        success = mAVRISP.program(fp,
                                  ATTINY84A_PAGESIZE,
                                  ATTINY84A_NUM_PAGES);
        fclose(fp);
    }

    if (!success) {
        printf("Programming failed.\r\n");
    } else {
        printf("Programming was successful!\r\n");
    }

    // To add another reset, should be handled by AVR910 already
    
    //DigitalOut reset(p8);
    //reset = 0;
    //wait(.02);
    //reset = 1;
    
}
