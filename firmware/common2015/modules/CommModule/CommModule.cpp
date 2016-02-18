#include "CommModule.hpp"

#include <ctime>

#include "CommPort.hpp"

#include "helper-funcs.hpp"
#include "logger.hpp"
#include "assert.hpp"

#define COMM_MODULE_SIGNAL_START_THREAD (1 << 0)

// Class declarations since everything in CommModule is static
bool CommModule::_isReady = false;
osThreadId CommModule::_txID;
osThreadId CommModule::_rxID;
CommPorts_t CommModule::_ports;

std::shared_ptr<CommModule> CommModule::instance;

// Default constructor
CommModule::CommModule() : _txQueueHelper(), _rxQueueHelper() {}

CommModule::~CommModule() { cleanup(); }

void CommModule::cleanup() {
    // terminate the threads
    osThreadTerminate(_txID);
    osThreadTerminate(_rxID);

    // release the allocated memory
    delete[](_txDef.stack_pointer);
    delete[](_rxDef.stack_pointer);
}

void CommModule::Init() {
    // Make sure we have an instance to work with
    instance = Instance();

    // Create the data queues.
    instance->_txQueue = osMailCreate(instance->_txQueueHelper.def(), nullptr);
    instance->_rxQueue = osMailCreate(instance->_rxQueueHelper.def(), nullptr);

    // Define the TX & RX task threads.
    define_thread(instance->_txDef, &CommModule::txThread, osPriorityHigh);
    define_thread(instance->_rxDef, &CommModule::rxThread,
                  osPriorityAboveNormal);

    // Create the TX & RX threads - pass them a pointer to the
    // created object.
    _txID = osThreadCreate(&(instance->_txDef), nullptr);
    _rxID = osThreadCreate(&(instance->_rxDef), nullptr);
}

shared_ptr<CommModule>& CommModule::Instance() {
    if (instance.get() == nullptr) instance.reset(new CommModule);

    return instance;
}

void CommModule::txThread(void const* arg) {
    // Store our priority so we know what to reset it to if ever needed
    osPriority threadPriority;

    // Only continue past this point once at least one hardware link is
    // initialized
    osSignalWait(COMM_MODULE_SIGNAL_START_THREAD, osWaitForever);

    threadPriority = osThreadGetPriority(instance->_txID);
    ASSERT(instance->_txID != nullptr);

    // Start up a ticker that disables the strobing TX LED. This is essentially
    // a watchdog timer for the TX LED's activity light
    RtosTimer led_ticker_timeout(commLightsTimeout_TX, osTimerOnce, nullptr);

    LOG(INIT,
        "TX communication module ready!\r\n    Thread ID:\t%u\r\n    "
        "Priority:\t%d",
        instance->_txID, threadPriority);

    // Signal to the RX thread that it can begin
    osSignalSet(_rxID, COMM_MODULE_SIGNAL_START_THREAD);

    osEvent evt;

    while (true) {
        // When a new rtp::packet is put in the TX queue, begin operations (does
        // nothing if no new data in queue)
        evt = osMailGet(instance->_txQueue, osWaitForever);

        if (evt.status == osEventMail) {
            // Get a pointer to the packet's memory location
            rtp::packet* p = (rtp::packet*)evt.value.p;

            // Bump up the thread's priority
            osStatus tState = osThreadSetPriority(_txID, osPriorityRealtime);
            ASSERT(tState == osOK);

            // Call the user callback function
            if (_ports[p->port()].isOpen()) {
                _ports[p->port()].TXCallback()(p);

                // Increment the packet counter by 1
                _ports[p->port()].TXPackets()++;

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

            tState = osThreadSetPriority(_txID, threadPriority);
            ASSERT(tState == osOK);
        }
    }

    osThreadTerminate(_txID);
}

void CommModule::rxThread(void const* arg) {
    // Store our priority so we know what to reset it to if ever needed
    osPriority threadPriority;

    // Only continue past this point once at least one (1) hardware link is
    // initialized
    osSignalWait(COMM_MODULE_SIGNAL_START_THREAD, osWaitForever);
    // set this true immediately after we are released execution
    _isReady = true;

    threadPriority = osThreadGetPriority(instance->_rxID);
    ASSERT(instance->_rxID != nullptr);

    // Start up a ticker that disables the strobing RX LED. This is essentially
    // a watchdog timer for the RX LED's activity light
    RtosTimer led_ticker_timeout(commLightsTimeout_RX, osTimerOnce, nullptr);

    LOG(INIT,
        "RX communication module ready!\r\n    Thread ID: %u, Priority: %d",
        instance->_rxID, threadPriority);

    rtp::packet* p;
    osEvent evt;

    while (true) {
        // Wait until new data is placed in the class's RX queue from a CommLink
        // class
        evt = osMailGet(instance->_rxQueue, osWaitForever);

        if (evt.status == osEventMail) {
            // get a pointer to where the data is stored
            p = (rtp::packet*)evt.value.p;

            // Bump up the thread's priority
            osStatus tState = osThreadSetPriority(_rxID, osPriorityRealtime);
            ASSERT(tState == osOK);

            // Call the user callback function (if set)
            if (_ports[p->port()].isOpen()) {
                _ports[p->port()].RXCallback()(p);

                // Increment the packet counter by 1
                _ports[p->port()].RXPackets()++;

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

            tState = osThreadSetPriority(_rxID, threadPriority);
            ASSERT(tState == osOK);
        }
    }

    osThreadTerminate(_rxID);
}

void CommModule::RxHandler(void (*ptr)(rtp::packet*), uint8_t portNbr) {
    if (!_ports[portNbr].Exists()) {
        CommPort_t port(portNbr);
        port.RXCallback() = std::bind(ptr, std::placeholders::_1);
        _ports += port;

    } else {
        _ports[portNbr].RXCallback() = std::bind(ptr, std::placeholders::_1);
    }

    ready();
}

void CommModule::TxHandler(void (*ptr)(rtp::packet*), uint8_t portNbr) {
    if (!_ports[portNbr].Exists()) {
        CommPort_t port(portNbr);

        port.TXCallback() = std::bind(ptr, std::placeholders::_1);

        _ports += port;

    } else {
        _ports[portNbr].TXCallback() = std::bind(ptr, std::placeholders::_1);
    }

    ready();
}

bool CommModule::openSocket(uint8_t portNbr) {
    if (!_ports[portNbr].Exists()) {
        CommPort_t port(portNbr);

        _ports += port;

        return _ports[portNbr].Open();
    }

    if (_ports[portNbr].Open()) {
        // Everything looks to be setup, go ahead and enable it
        LOG(INF1, "Port %u opened", portNbr);

        return true;
    } else {
        // this almost never gets called. Probably going to remove it soon. Only
        // makes it this far if trying to open port 0 without any setup.
        // TX callback function was never set
        LOG(WARN,
            "Must set at least the RX callback function before opening socket "
            "on port %u.",
            portNbr);

        return false;
    }
}

void CommModule::ready() {
    if (_isReady == true) return;

    // Start running the TX thread - it will trigger with to startup the RX
    // thread
    osSignalSet(_txID, COMM_MODULE_SIGNAL_START_THREAD);
}

void CommModule::send(const rtp::packet& packet) {
    // Check to make sure a socket for the port exists
    if (_ports[packet.port()].isOpen() &&
        _ports[packet.port()].hasTXCallback()) {
        // Allocate a block of memory for the data.
        rtp::packet* p =
            (rtp::packet*)osMailAlloc(instance->_txQueue, osWaitForever);

        // Copy the contents into the allocated memory block
        *p = packet;

        // Place the passed packet into the txQueue.
        osMailPut(instance->_txQueue, p);

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
        _ports[packet.port()].hasRXCallback()) {
        // Allocate a block of memory for the data.
        rtp::packet* p =
            (rtp::packet*)osMailAlloc(instance->_rxQueue, osWaitForever);

        // Copy the contents into the allocated memory block
        *p = packet;

        // Place the passed packet into the rxQueue.
        osMailPut(instance->_rxQueue, p);

    } else {
        LOG(WARN,
            "Failed to receive %u byte packet: There is no open receiving "
            "socket for port %u",
            packet.payload.size(), packet.port());
    }
}

unsigned int CommModule::NumRXPackets() { return _ports.allRXPackets(); }

unsigned int CommModule::NumTXPackets() { return _ports.allTXPackets(); }

void CommModule::PrintInfo(bool forceHeader) {
    if (forceHeader == true && _ports.empty()) {
        PrintHeader();
        _ports.PrintFooter();
    } else {
        _ports.PrintPorts();
        _ports.PrintFooter();
    }

    Console::Instance()->Flush();
}

void CommModule::PrintHeader() { _ports.PrintHeader(); }

void CommModule::ResetCount(unsigned int portNbr) {
    _ports[portNbr].RXPackets() = 0;
    _ports[portNbr].TXPackets() = 0;
}

void CommModule::Close(unsigned int portNbr) { _ports[portNbr].Close(); }

bool CommModule::isReady() { return _isReady; }

int CommModule::NumOpenSockets() { return _ports.count_open(); }
