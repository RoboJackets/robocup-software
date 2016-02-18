#include "CommModule.hpp"

#include <ctime>

#include "CommPort.hpp"

#include "helper-funcs.hpp"
#include "logger.hpp"
#include "assert.hpp"

#define COMM_MODULE_SIGNAL_START_THREAD (1 << 0)

std::shared_ptr<CommModule> CommModule::instance;

CommModule::~CommModule() {
    // terminate the threads
    osThreadTerminate(_txID);
    osThreadTerminate(_rxID);

    // release the allocated memory
    delete[](_txDef.stack_pointer);
    delete[](_rxDef.stack_pointer);
}

void CommModule::init() {
    // Create the data queues.
    _txQueue = osMailCreate(_txQueueHelper.def(), nullptr);
    _rxQueue = osMailCreate(_rxQueueHelper.def(), nullptr);

    // Define the TX & RX task threads.
    define_thread(_txDef, &CommModule::txThread, osPriorityHigh);
    define_thread(_rxDef, &CommModule::rxThread, osPriorityAboveNormal);

    // Create the TX & RX threads - pass them a pointer to the
    // created object.
    _txID = osThreadCreate(&_txDef, nullptr);
    _rxID = osThreadCreate(&_rxDef, nullptr);
}

shared_ptr<CommModule>& CommModule::Instance() {
    if (!instance) instance.reset(new CommModule);

    return instance;
}

void CommModule::txThread(void const* arg) {
    // Only continue past this point once at least one hardware link is
    // initialized
    osSignalWait(COMM_MODULE_SIGNAL_START_THREAD, osWaitForever);

    ASSERT(instance->_txID != nullptr);

    // Store our priority so we know what to reset it to if ever needed
    const osPriority threadPriority = osThreadGetPriority(instance->_txID);

    // Start up a ticker that disables the strobing TX LED. This is essentially
    // a watchdog timer for the TX LED's activity light
    RtosTimer led_ticker_timeout(commLightsTimeout_TX, osTimerOnce, nullptr);

    LOG(INIT,
        "TX communication module ready!\r\n    Thread ID:\t%u\r\n    "
        "Priority:\t%d",
        instance->_txID, threadPriority);

    // Signal to the RX thread that it can begin
    osSignalSet(instance->_rxID, COMM_MODULE_SIGNAL_START_THREAD);

    while (true) {
        // When a new rtp::packet is put in the TX queue, begin operations (does
        // nothing if no new data in queue)
        osEvent evt = osMailGet(instance->_txQueue, osWaitForever);

        if (evt.status == osEventMail) {
            // Get a pointer to the packet's memory location
            rtp::packet* p = (rtp::packet*)evt.value.p;

            // Bump up the thread's priority
            osStatus tState =
                osThreadSetPriority(instance->_txID, osPriorityRealtime);
            ASSERT(tState == osOK);

            // Call the user callback function
            if (instance->_ports[p->port()].isOpen()) {
                instance->_ports[p->port()].txCallback()(p);

                // Increment the packet counter by 1
                instance->_ports[p->port()].incTxCount();

                LOG(INF2, "Transmission:\r\n    Port:\t%u\r\n    Subclass:\t%u",
                    p->port(), p->subclass());
            }

            // this renews a countdown for turning off the
            // strobing thread once it expires
            if (p->address() != 127) {
                led_ticker_timeout.start(275);
                commLightsRenew_TX();
            }

            // Release the allocated memory once data is sent
            osMailFree(instance->_txQueue, p);

            tState = osThreadSetPriority(instance->_txID, threadPriority);
            ASSERT(tState == osOK);
        }
    }

    osThreadTerminate(instance->_txID);
}

void CommModule::rxThread(void const* arg) {
    // Only continue past this point once at least one (1) hardware link is
    // initialized
    osSignalWait(COMM_MODULE_SIGNAL_START_THREAD, osWaitForever);

    ASSERT(instance->_rxID != nullptr);

    // set this true immediately after we are released execution
    instance->_isReady = true;

    // Store our priority so we know what to reset it to if ever needed
    const osPriority threadPriority = osThreadGetPriority(instance->_rxID);

    // Start up a ticker that disables the strobing RX LED. This is essentially
    // a watchdog timer for the RX LED's activity light
    RtosTimer led_ticker_timeout(commLightsTimeout_RX, osTimerOnce, nullptr);

    LOG(INIT,
        "RX communication module ready!\r\n    Thread ID: %u, Priority: %d",
        instance->_rxID, threadPriority);

    while (true) {
        // Wait until new data is placed in the class's RX queue from a CommLink
        // class
        osEvent evt = osMailGet(instance->_rxQueue, osWaitForever);

        if (evt.status == osEventMail) {
            // get a pointer to where the data is stored
            rtp::packet* p = (rtp::packet*)evt.value.p;

            // Bump up the thread's priority
            osStatus tState =
                osThreadSetPriority(instance->_rxID, osPriorityRealtime);
            ASSERT(tState == osOK);

            // Call the user callback function (if set)
            if (instance->_ports[p->port()].isOpen()) {
                instance->_ports[p->port()].rxCallback()(p);

                // Increment the packet counter by 1
                instance->_ports[p->port()].incRxCount();

                LOG(INF2, "Reception:\r\n    Port:\t%u\r\n    Subclass:\t%u",
                    p->port(), p->subclass());
            }

            // this renews a countdown for turning off the
            // strobing thread once it expires
            if (p->address() != 127) {
                led_ticker_timeout.start(275);
                commLightsRenew_RX();
            }

            // free memory allocated for mail
            osMailFree(instance->_rxQueue, p);

            tState = osThreadSetPriority(instance->_rxID, threadPriority);
            ASSERT(tState == osOK);
        }
    }

    osThreadTerminate(instance->_rxID);
}

void CommModule::setRxHandler(CommCallback callback, uint8_t portNbr) {
    if (!_ports.hasPort(portNbr)) {
        _ports += CommPort_t(portNbr);
    }

    _ports[portNbr].rxCallback() = std::bind(callback, std::placeholders::_1);

    ready();
}

void CommModule::setTxHandler(CommCallback callback, uint8_t portNbr) {
    if (!_ports.hasPort(portNbr)) {
        _ports += CommPort_t(portNbr);
    }

    _ports[portNbr].txCallback() = std::bind(callback, std::placeholders::_1);

    ready();
}

void CommModule::openSocket(uint8_t portNbr) {
    // create the port if it doesn't exist
    if (!_ports.hasPort(portNbr)) {
        _ports += CommPort_t(portNbr);
    }

    CommPort_t& port = _ports[portNbr];

    if (!port.isOpen()) {
        port.open();
        LOG(INF1, "Port %u opened", portNbr);
    }
}

void CommModule::ready() {
    if (_isReady) return;

    // Start running the TX thread - it will trigger with to startup the RX
    // thread
    osSignalSet(_txID, COMM_MODULE_SIGNAL_START_THREAD);
}

void CommModule::send(const rtp::packet& packet) {
    // Check to make sure a socket for the port exists
    if (_ports[packet.port()].isOpen() &&
        _ports[packet.port()].hasTxCallback()) {
        // Allocate a block of memory for the data.
        rtp::packet* p = (rtp::packet*)osMailAlloc(_txQueue, osWaitForever);

        // Copy the contents into the allocated memory block
        *p = packet;

        // Place the passed packet into the txQueue.
        osMailPut(_txQueue, p);

    } else {
        LOG(WARN,
            "Failed to send %u byte packet: There is no open transmitting "
            "socket for port %u",
            packet.payload.size(), packet.port());
    }
}

void CommModule::receive(const rtp::packet& packet) {
    // Check to make sure a socket for the port exists
    if (_ports[packet.port()].isOpen() &&
        _ports[packet.port()].hasRxCallback()) {
        // Allocate a block of memory for the data.
        rtp::packet* p = (rtp::packet*)osMailAlloc(_rxQueue, osWaitForever);

        // Copy the contents into the allocated memory block
        *p = packet;

        // Place the passed packet into the rxQueue.
        osMailPut(_rxQueue, p);

    } else {
        LOG(WARN,
            "Failed to receive %u byte packet: There is no open receiving "
            "socket for port %u",
            packet.payload.size(), packet.port());
    }
}

unsigned int CommModule::numRxPackets() const { return _ports.totalRxCount(); }

unsigned int CommModule::numTxPackets() const { return _ports.totalTxCount(); }

void CommModule::printInfo() const {
    _ports.printHeader();
    _ports.printPorts();
    _ports.printFooter();

    Console::Instance()->Flush();
}

void CommModule::resetCount(unsigned int portNbr) {
    _ports[portNbr].resetPacketCount();
}

void CommModule::close(unsigned int portNbr) { _ports[portNbr].close(); }

bool CommModule::isReady() const { return _isReady; }

int CommModule::numOpenSockets() const { return _ports.countOpen(); }
