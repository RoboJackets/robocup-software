#include "CommLink.hpp"

#include "logger.hpp"
#include "assert.hpp"

#define COMM_LINK_SIGNAL_START_THREAD (1 << 0)
#define COMM_LINK_SIGNAL_RX_TRIGGER (1 << 1)

const char* COMM_ERR_STRING[] = {FOREACH_COMM_ERR(GENERATE_STRING)};

CommLink::CommLink(shared_ptr<SharedSPI> sharedSPI, PinName nCs,
                   PinName int_pin)
    : SharedSPIDevice(sharedSPI, nCs, true),
      _int_in(int_pin),
      _rxThread(&CommLink::rxThreadHelper, this, osPriorityNormal,
                DEFAULT_STACK_SIZE / 2) {
    ASSERT(_spi != nullptr);
    _int_in.mode(PullUp);
}

// =================== RX THREAD ===================
// Task operations for placing received data into the received data queue
void CommLink::rxThread() {
    // Only continue past this point once the hardware link is initialized
    Thread::signal_wait(COMM_LINK_SIGNAL_START_THREAD);

    // Store our priority so we know what to reset it to if ever needed
    const osPriority threadPriority = _rxThread.get_priority();

    LOG(INIT, "RX communication link ready!\r\n    Thread ID: %u, Priority: %d",
        _rxThread.gettid(), threadPriority);

    // Set the function to call on an interrupt trigger
    _int_in.rise(this, &CommLink::ISR);

    rtp::packet p;
    std::vector<uint8_t> buf;
    buf.reserve(rtp::MAX_DATA_SZ);

    while (true) {
        // Wait until new data has arrived
        // this is triggered by CommLink::ISR()
        Thread::signal_wait(COMM_LINK_SIGNAL_RX_TRIGGER);

        LOG(INF3, "RX interrupt triggered");

        // Get the received data from the external chip
        buf.clear();
        int32_t response = getData(&buf);

        if (response == COMM_SUCCESS) {
            // Write the data to the CommModule object's rxQueue
            p.recv(buf);
            CommModule::Instance->receive(p);
        }
    }
}

// Called by the derived class to begin thread operations
void CommLink::ready() { _rxThread.signal_set(COMM_LINK_SIGNAL_START_THREAD); }

void CommLink::sendPacket(rtp::packet* p) {
    std::vector<uint8_t> buffer;
    p->pack(&buffer);
    sendData(buffer.data(), buffer.size());
}

void CommLink::ISR() { _rxThread.signal_set(COMM_LINK_SIGNAL_RX_TRIGGER); }
