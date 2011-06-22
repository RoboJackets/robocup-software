#ifndef _USB_H_
#define _USB_H_

typedef struct
{
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} USB_Setup_Request;

#endif // _USB_H_
