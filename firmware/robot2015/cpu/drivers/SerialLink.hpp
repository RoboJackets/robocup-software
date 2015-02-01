#if 0

#ifndef SERIAL_LINK_H
#define SERIAL_LINK_H

#include "mbed.h"
#include "CommLink.h"
#include "cmsis_os.h"
#include "RTP.h"
#include "ThreadHelper.h"

class SerialLink : public CommLink
{
public:
    SerialLink();
    virtual ~SerialLink();

    virtual uint32_t sendPacket(RTP_t*);
    virtual uint32_t receivePacket(RTP_t*);
    virtual uint32_t reset(void);
    virtual uint32_t selfTest(void);
    virtual bool isConnected(void);

protected:


private:


};

#endif  // SERIAL_LINK_H

#endif
