#include <string>
#include "PCLink.hpp"

PCLink::PCLink(void)
    : usbLink(64, 64, RJ_VENDOR_ID, RJ_PRODUCT_ID, RJ_RELEASE) {}

PCLink::PCLink(uint16_t vendorID = RJ_VENDOR_ID,
               uint16_t productID = RJ_PRODUCT_ID,
               uint16_t release = RJ_RELEASE)
    : usbLink(64, 64, vendorID, productID, release) {
      pc = NULL;
}

PCLink::~PCLink(void) {
    // delete usbLink;
}

void PCLink::setSerialDebugging(Serial *pc) {
   this->pc = pc;
}

void PCLink::setLed(DigitalOut *led) {
   this->led = led;
}

void PCLink::read(void) { 
   if (usbLink.readNB(&in)) {
      *led = !(*led);

      if (pc != NULL) {
         pc->printf("%d\r\n", in.data[0]);
      }
   }
}

void PCLink::reply(void) {
    usbLink.sendNB(&out);
}
