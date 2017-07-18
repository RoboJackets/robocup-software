#pragma once

#include <libusb.h>
#include <stdint.h>
#include <QMutex>
#include <mutex>

#include "Radio.hpp"

// Included for packet layout
#include "firmware-common/common2015/utils/rtp.hpp"
#include "firmware-common/common2015/utils/DebugCommunicationStrings.hpp"

/**
 * @brief Radio IO with real robots
 *
 * @details This class provides us the ability to communicate with real robots
 *     using our own radio protocol. The radio sends one large packet to all of
 *     the robots at once that contains the data in each robot's radioTx packet.
 *     Note that it isn't sent in protobuf format though, it's sent straight-up
 *     data to avoid the overhead of protobuf.  Robots respond individually in
 *     order of their shell numbers in a set time slot.  The bot with the lowest
 *     shell number replies in the first time slot, and so on.  This ensures
 *     that robots don't jam each other's communication.
 */
class USBRadio : public Radio {
public:
    USBRadio();
    ~USBRadio();

    virtual bool isOpen() const override;
    virtual void send(Packet::RadioTx& packet) override;
    virtual void receive() override;

    virtual void channel(int n) override;
    void switchTeam(bool) override {}

protected:
    std::vector<DebugCommunication::DebugResponse> current_receive_debug;
    std::mutex current_receive_debug_mutex;
    libusb_context* _usb_context;
    libusb_device_handle* _device;

    // These transfers are used to receive packets.
    // Try increasing this constant for larger RX packet throughput.
    static const int NumRXTransfers = 4;
    libusb_transfer* _rxTransfers[NumRXTransfers];
    uint8_t _rxBuffers[NumRXTransfers][rtp::ReverseSize + 2];

    QMutex _mutex;
    bool _printedError;

    static void rxCompleted(struct libusb_transfer* transfer);
    void handleRxData(uint8_t* buf);

    bool open();

    // Low level operations
    void command(uint8_t cmd);
    void write(uint8_t reg, uint8_t value);
    uint8_t read(uint8_t reg);
};
