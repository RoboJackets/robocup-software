#include "CommModule.hpp"
#include "CommPort.hpp"
#include "assert.hpp"
#include "helper-funcs.hpp"
#include "logger.hpp"

#include <ctime>

using namespace std;

#define COMM_MODULE_SIGNAL_START_THREAD (1 << 0)

std::shared_ptr<CommModule> CommModule::Instance;

CommModule::~CommModule() {
    // note: the destructor for the Thread class automatically calls
    // terminate(), so we don't have to do it here.
}

CommModule::CommModule(std::shared_ptr<FlashingTimeoutLED> rxTimeoutLED,
                       std::shared_ptr<FlashingTimeoutLED> txTimeoutLED)
    : _rxThread(&CommModule::rxThreadHelper, this, osPriorityAboveNormal,
                DEFAULT_STACK_SIZE / 2),
      _txThread(&CommModule::txThreadHelper, this, osPriorityHigh,
                DEFAULT_STACK_SIZE / 2),
      _rxTimeoutLED(rxTimeoutLED),
      _txTimeoutLED(txTimeoutLED) {
    // Create the data queues.
    _txQueue = osMailCreate(_txQueueHelper.def(), nullptr);
    _rxQueue = osMailCreate(_rxQueueHelper.def(), nullptr);
}

void CommModule::rxThreadHelper(void const* moduleInst) {
    CommModule* module = (CommModule*)moduleInst;
    module->rxThread();
}
void CommModule::txThreadHelper(void const* moduleInst) {
    CommModule* module = (CommModule*)moduleInst;
    module->txThread();
}

void CommModule::txThread() {
    // Only continue past this point once at least one hardware link is
    // initialized
    Thread::signal_wait(COMM_MODULE_SIGNAL_START_THREAD);

    // Store our priority so we know what to reset it to if ever needed
    const osPriority threadPriority = _txThread.get_priority();

    LOG(INIT,
        "TX communication module ready!\r\n    Thread ID: %u, Priority: %d",
        ((P_TCB)_txThread.gettid())->task_id, threadPriority);

    // Signal to the RX thread that it can begin
    _rxThread.signal_set(COMM_MODULE_SIGNAL_START_THREAD);

    while (true) {
        // When a new rtp::packet is put in the TX queue, begin operations (does
        // nothing if no new data in queue)
        osEvent evt = osMailGet(_txQueue, osWaitForever);

        if (evt.status == osEventMail) {
            // Get a pointer to the packet's memory location
            rtp::packet* p = (rtp::packet*)evt.value.p;

            // Bump up the thread's priority
            osStatus tState = _txThread.set_priority(osPriorityRealtime);
            ASSERT(tState == osOK);

            // this renews a countdown for turning off the
            // strobing thread once it expires
            if (p->header.address != rtp::LOOPBACK_ADDRESS && _txTimeoutLED) {
                _txTimeoutLED->renew();
            }

            // Call the user callback function
            if (_ports.find(p->header.port) != _ports.end() &&
                _ports[p->header.port].txCallback() != nullptr) {
                _ports[p->header.port].txCallback()(p);
                _ports[p->header.port].txCount++;

                // LOG(INF2, "Transmission:\r\n    Port:\t%u\r\n",
                // p->header.port);
            }

            // Release the allocated memory once data is sent
            osMailFree(_txQueue, p);

            tState = _txThread.set_priority(threadPriority);
            ASSERT(tState == osOK);
        }
    }
}

void CommModule::rxThread() {
    // Only continue past this point once at least one (1) hardware link is
    // initialized
    Thread::signal_wait(COMM_MODULE_SIGNAL_START_THREAD);

    // set this true immediately after we are released execution
    _isReady = true;

    // Store our priority so we know what to reset it to if ever needed
    const osPriority threadPriority = _rxThread.get_priority();

    LOG(INIT,
        "RX communication module ready!\r\n    Thread ID: %u, Priority: %d",
        ((P_TCB)_rxThread.gettid())->task_id, threadPriority);

    while (true) {
        // Wait until new data is placed in the class's RX queue from a CommLink
        // class
        osEvent evt = osMailGet(_rxQueue, osWaitForever);

        if (evt.status == osEventMail) {
            // get a pointer to where the data is stored
            rtp::packet* p = (rtp::packet*)evt.value.p;

            // Bump up the thread's priority
            osStatus tState = _rxThread.set_priority(osPriorityRealtime);
            ASSERT(tState == osOK);

            // this renews a countdown for turning off the strobing thread once
            // it expires
            if (p->header.address != rtp::LOOPBACK_ADDRESS && _rxTimeoutLED) {
                _rxTimeoutLED->renew();
            }

            // Call the user callback function (if set)
            if (_ports.find(p->header.port) != _ports.end() &&
                _ports[p->header.port].rxCallback() != nullptr) {
                _ports[p->header.port].rxCallback()(std::move(*p));
                _ports[p->header.port].rxCount++;

                // LOG(INF2, "Reception:\r\n    Port:\t%u\r\n", p->header.port);
            }

            // free memory allocated for mail
            osMailFree(_rxQueue, p);

            tState = _rxThread.set_priority(threadPriority);
            ASSERT(tState == osOK);
        }
    }
}

void CommModule::setRxHandler(std::function<CommRxCallback> callback,
                              uint8_t portNbr) {
    _ports[portNbr].rxCallback() = std::bind(callback, std::placeholders::_1);

    ready();
}

void CommModule::setTxHandler(std::function<CommTxCallback> callback,
                              uint8_t portNbr) {
    _ports[portNbr].txCallback() = std::bind(callback, std::placeholders::_1);

    ready();
}

void CommModule::ready() {
    if (_isReady) return;

    // Start running the TX thread - it will trigger with to startup the RX
    // thread
    _txThread.signal_set(COMM_MODULE_SIGNAL_START_THREAD);
}

void CommModule::send(rtp::packet packet) {
    // Check to make sure a socket for the port exists
    if (_ports.find(packet.header.port) != _ports.end() &&
        _ports[packet.header.port].txCallback() != nullptr) {
        // Allocate a block of memory for the data.
        rtp::packet* p = (rtp::packet*)osMailAlloc(_txQueue, osWaitForever);
        if (!p) {
            LOG(FATAL, "Unable to allocate packet onto mail queue");
            return;
        }

        // Copy the contents into the allocated memory block
        *p = std::move(packet);

        // Place the passed packet into the txQueue.
        osMailPut(_txQueue, p);

    } else {
        LOG(WARN,
            "Failed to send %u byte packet: There is no open transmitting "
            "socket for port %u",
            packet.payload.size(), packet.header.port);
    }
}

void CommModule::receive(rtp::packet packet) {
    // Check to make sure a socket for the port exists
    if (_ports.find(packet.header.port) != _ports.end() &&
        _ports[packet.header.port].rxCallback() != nullptr) {
        // Allocate a block of memory for the data.
        rtp::packet* p = (rtp::packet*)osMailAlloc(_rxQueue, osWaitForever);
        if (!p) {
            LOG(FATAL, "Unable to allocate packet onto mail queue");
            return;
        }

        // Copy the contents into the allocated memory block
        // TODO: move semantics
        *p = std::move(packet);

        // Place the passed packet into the rxQueue.
        osMailPut(_rxQueue, p);
    } else {
        LOG(WARN,
            "Failed to receive %u byte packet: There is no open receiving "
            "socket for port %u",
            packet.payload.size(), packet.header.port);
    }
}

unsigned int CommModule::numRxPackets() const {
    unsigned int count = 0;
    for (auto& kvpair : _ports) {
        count += kvpair.second.rxCount;
    }
    return count;
}

unsigned int CommModule::numTxPackets() const {
    unsigned int count = 0;
    for (auto& kvpair : _ports) {
        count += kvpair.second.txCount;
    }
    return count;
}

void CommModule::printInfo() const {
    printf("PORT\t\tIN\tOUT\tRX CBCK\t\tTX CBCK\r\n");

    for (const auto& kvpair : _ports) {
        const CommPort_t& p = kvpair.second;
        printf("%d\t\t%u\t%u\t%s\t\t%s\r\n", kvpair.first, p.rxCount, p.txCount,
               p.rxCallback() ? "YES" : "NO", p.txCallback() ? "YES" : "NO");
    }

    printf(
        "==========================\r\n"
        "Total:\t\t%u\t%u\r\n",
        numRxPackets(), numTxPackets());

    Console::Instance()->Flush();
}

void CommModule::resetCount(unsigned int portNbr) {
    _ports[portNbr].resetPacketCount();
}

void CommModule::close(unsigned int portNbr) { _ports.erase(portNbr); }

bool CommModule::isReady() const { return _isReady; }

int CommModule::numOpenSockets() const {
    size_t count = 0;
    for (const auto& kvpair : _ports) {
        if (kvpair.second.rxCallback() != nullptr ||
            kvpair.second.txCallback() != nullptr)
            count++;
    }

    return count;
}
