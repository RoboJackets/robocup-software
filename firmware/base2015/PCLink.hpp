#include <mbed.h>
#include <USBHID.h>

class PCLink {
public:
    static const uint16_t RJ_VENDOR_ID = 0x524A;
    static const uint16_t RJ_PRODUCT_ID = 0x4253;
    static const uint16_t RJ_RELEASE = 0x0000;

    PCLink(void);
    PCLink(uint16_t vendorID, uint16_t productID, uint16_t release);
    virtual ~PCLink(void);
    void setSerialDebugging(Serial *pc);
    void setLed(DigitalOut *led);
    void read(void);
    void reply(void);
private:
    Serial *pc;
    DigitalOut *led;
    USBHID usbLink;
    HID_REPORT in;
    HID_REPORT out;
    // unsigned char buf[65];
};
