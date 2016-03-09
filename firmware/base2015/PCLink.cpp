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

      switch (in.data[0]) {
         case HID_FWD_GLOB_PKT:
            //set pkt data
            break;
         case HID_FWD_GLOB_TX:
            //tx pkt data
            break;
         case HID_FWD_BOT_TUNE_PKT:
            //set tune data
            break;
         case HID_FWD_BOT_TUNE_TX:
            //tx tune data
            break;
         case HID_FWD_
      }

      if (pc != NULL) {
         // std::string tmp(reinterpret_cast<const char *>(in.data));
         // tmp = tmp.append("\r\n");
         // pc->printf("%s", tmp.c_str()); 
         pc->printf("%d\r\n", in.data[0]);
      }
   }
}

void PCLink::reply(void) {
    usbLink.sendNB(&out);
}