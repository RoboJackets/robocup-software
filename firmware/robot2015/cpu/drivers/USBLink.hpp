#if 0

#ifndef USB_LINK_H
#define USB_LINK_H

#include "mbed.h"
#include "CommLink.h"
#include "cmsis_os.h"
#include "RTP.h"
#include "ThreadHelper.h"

class USBLink : public CommLink
{
public:
    USBLink();
    virtual ~USBLink();

    virtual uint32_t sendPacket(RTP_t*);
    virtual uint32_t receivePacket(RTP_t*);
    virtual uint32_t reset(void);
    virtual uint32_t selfTest(void);
    virtual bool isConnected(void);

protected:


private:


};

#endif  // USB_LINK_H

#endif
