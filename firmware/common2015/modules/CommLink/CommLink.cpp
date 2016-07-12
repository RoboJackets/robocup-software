#include "CommLink.hpp"

#include "assert.hpp"
#include "logger.hpp"

#define COMM_LINK_SIGNAL_START_THREAD (1 << 0)
#define COMM_LINK_SIGNAL_RX_TRIGGER (1 << 1)

const char* COMM_ERR_STRING[] = {FOREACH_COMM_ERR(GENERATE_STRING)};

CommLink::CommLink(shared_ptr<SharedSPI> sharedSPI, PinName nCs,
                   PinName int_pin)
    : SharedSPIDevice(sharedSPI, nCs, true),
      _int_in(int_pin),
      _rxThread(&CommLink::rxThreadHelper, this, osPriorityNormal,
                DEFAULT_STACK_SIZE / 2) {
    setSPIFrequency(5000000);
    _int_in.mode(PullUp);
}

// =================== RX THREAD ===================
// Task operations for placing received data into the received data queue
void CommLink::rxThread() {
    // Store our priority so we know what to reset it to if ever needed
    const osPriority threadPriority = _rxThread.get_priority();

    // Set the function to call on an interrupt trigger
    _int_in.fall(this, &CommLink::ISR);

    std::vector<uint8_t> buf;
    buf.reserve(rtp::MAX_DATA_SZ);

    // Only continue past this point once the hardware link is initialized
    Thread::signal_wait(COMM_LINK_SIGNAL_START_THREAD);

    LOG(INIT, "RX communication link ready!\r\n    Thread ID: %u, Priority: %d",
        ((P_TCB)_rxThread.gettid())->task_id, threadPriority);

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
            rtp::packet p;
            p.recv(buf);
            CommModule::Instance->receive(std::move(p));
        }
    }
}

// Called by the derived class to begin thread operations
void CommLink::ready() { _rxThread.signal_set(COMM_LINK_SIGNAL_START_THREAD); }

void CommLink::ISR() { _rxThread.signal_set(COMM_LINK_SIGNAL_RX_TRIGGER); }
