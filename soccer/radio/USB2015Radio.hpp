#include <hidapi.h>
#include "Radio.hpp"

class USB2015Radio : public Radio {
public:
    static const unsigned int RJ_VENDOR_ID = 0x524A;
    static const unsigned int RJ_BASE_2015 = 0x4253;

    USB2015Radio(const unsigned int VENDOR_ID = RJ_VENDOR_ID, const unsigned int PRODUCT_ID = RJ_BASE_2015);
    virtual ~USB2015Radio();

    virtual bool isOpen() const override;
    virtual void send(Packet::RadioTx& packet) override;
    virtual void receive() override;
    virtual void switchTeam(bool blueTeam) override;

    std::string getProtocolVersion();


private:
    static volatile bool hidAPIInit = false;
    static volatile int numInstances = 0;
    hid_device *base;

    const unsigned char hidBufLen = 64;
    unsigned char hidTxBuf[64];
    unsigned char hidRxBuf[64];
}