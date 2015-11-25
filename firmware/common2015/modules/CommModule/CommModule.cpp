/*! \file autolink.cpp

  A link to CommModule::Init

*/

#include "CommModule.hpp"

#include "CommPort.hpp"

#include "helper-funcs.hpp"
#include "logger.hpp"
#include "assert.hpp"

// Class declarations since everything in CommModule is static
bool CommModule::_isReady = false;
osThreadId CommModule::_txID;
osThreadId CommModule::_rxID;
CommPorts_t CommModule::_ports;

std::shared_ptr<CommModule> CommModule::instance;

// This one isn't apart of any class, but it's an extern in CommModule.hpp
CommPort_t _tmpPort;

// Default constructor
CommModule::CommModule()
    :  // [X] - 1.1 - Define the data queues.
      // =================
      _txQueueHelper(),
      _rxQueueHelper() {}

CommModule::~CommModule() { cleanup(); }

void CommModule::cleanup(void) {
    // terminate the threads
    osThreadTerminate(_txID);
    osThreadTerminate(_rxID);

    // release the allocated memory
    delete[](_txDef.stack_pointer);
    delete[](_rxDef.stack_pointer);
}

void CommModule::Init(void) {
    // [X] - 1.0 - Make sure we have an instance to work with
    auto instance = Instance();

    // [X] - 1.1 - Create the data queues.
    // =================
    instance->_txQueue = osMailCreate(instance->_txQueueHelper.def(), nullptr);
    instance->_rxQueue = osMailCreate(instance->_rxQueueHelper.def(), nullptr);

    // [X] - 1.2 - Define the TX & RX task threads.
    // =================
    define_thread(instance->_txDef, &CommModule::txThread);
    define_thread(instance->_rxDef, &CommModule::rxThread);

    // [X] - 1.3 - Create the TX & RX threads - pass them a pointer to the
    // created object.
    // =================
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

    // Only continue past this point once at least one (1) hardware link is
    // initialized
    osSignalWait(COMM_MODULE_SIGNAL_START_THREAD, osWaitForever);

    threadPriority = osThreadGetPriority(instance->_txID);
    ASSERT(instance->_txID != nullptr);

    // Check for the existance of a TX LED to flash
    if (instance->_txLED == nullptr) {
        LOG(SEVERE, "TX LED unset at thread start!");
    }

    LOG(INIT,
        "TX communication module ready!\r\n    Thread ID:\t%u\r\n    "
        "Priority:\t%d",
        instance->_txID, threadPriority);

    // Signal to the RX thread that it can begin
    osSignalSet(_rxID, COMM_MODULE_SIGNAL_START_THREAD);

    osEvent evt;

    while (true) {
        // When a new rtp::packet is put in the tx queue, begin operations (does
        // nothing if no new data in queue)
        evt = osMailGet(instance->_txQueue, osWaitForever);

        if (evt.status == osEventMail) {
            // Get a pointer to the packet's memory location
            rtp::packet* p = (rtp::packet*)evt.value.p;

            // Bump up the thread's priority
            ASSERT(osThreadSetPriority(_txID, osPriorityRealtime) == osOK);

            // Call the user callback function
            if (_ports[p->port].isOpen()) {
                _ports[p->port].TXCallback()(p);

                // Increment the packet counter by 1
                _ports[p->port].TXPackets()++;

                LOG(INF2, "Transmission:\r\n    Port:\t%u\r\n    Subclass:\t%u",
                    p->port, p->subclass);
            }

            // Release the allocated memory once data is sent
            osMailFree(instance->_txQueue, p);

            strobeStatusLED((void*)(instance->_txLED));

            ASSERT(osThreadSetPriority(_txID, threadPriority) == osOK);
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

    threadPriority = osThreadGetPriority(instance->_rxID);
    ASSERT(instance->_rxID != nullptr);

    // Check for the existance of an RX LED to flash
    if (instance->_rxLED == nullptr) {
        LOG(SEVERE, "rX LED unset at thread start!");
    }

    LOG(INIT,
        "RX communication module ready!\r\n    Thread ID:\t%u\r\n    "
        "Priority:\t%d",
        instance->_rxID, threadPriority);

    _isReady = true;

    rtp::packet* p;
    osEvent evt;

    while (true) {
        // Wait until new data is placed in the class's rxQueue from a CommLink
        // class
        evt = osMailGet(instance->_rxQueue, osWaitForever);

        if (evt.status == osEventMail) {
            // get a pointer to where the data is stored
            p = (rtp::packet*)evt.value.p;

            // Bump up the thread's priority
            ASSERT(osThreadSetPriority(_rxID, osPriorityRealtime) == osOK);

            // Call the user callback function (if set)
            if (_ports[p->port].isOpen()) {
                _ports[p->port].RXCallback()(p);

                // Increment the packet counter by 1
                _ports[p->port].RXPackets()++;

                LOG(INF2, "Reception:\r\n    Port:\t%u\r\n    Subclass:\t%u",
                    p->port, p->subclass);
            }

            osMailFree(instance->_rxQueue,
                       p);  // free memory allocated for mail

            strobeStatusLED((void*)(instance->_rxLED));
            ASSERT(osThreadSetPriority(_rxID, threadPriority) == osOK);
        }
    }

    osThreadTerminate(_rxID);
}

void CommModule::RxHandler(void (*ptr)(rtp::packet*), uint8_t portNbr) {
    if (!_ports[portNbr].Exists()) {
        CommPort_t _tmpPort(portNbr);

        _tmpPort.RXCallback() = std::bind(ptr, std::placeholders::_1);

        _ports += _tmpPort;

    } else {
        _ports[portNbr].RXCallback() = std::bind(ptr, std::placeholders::_1);
    }

    ready();
}

void CommModule::TxHandler(void (*ptr)(rtp::packet*), uint8_t portNbr) {
    if (!_ports[portNbr].Exists()) {
        CommPort_t _tmpPort(portNbr);

        _tmpPort.TXCallback() = std::bind(ptr, std::placeholders::_1);

        _ports += _tmpPort;

    } else {
        _ports[portNbr].TXCallback() = std::bind(ptr, std::placeholders::_1);
    }

    ready();
}

bool CommModule::openSocket(uint8_t portNbr) {
    if (!_ports[portNbr].Exists()) {
        CommPort_t _tmpPort(portNbr);

        _ports += _tmpPort;

        ready();

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
            "Must set TX & RX callback functions before opening socket.\r\n");

        return false;
    }
}

void CommModule::ready(void) {
    if (_isReady == true) return;

    // Start running the TX thread - it will trigger with to startup the RX
    // thread
    osSignalSet(_txID, COMM_MODULE_SIGNAL_START_THREAD);
}

void CommModule::send(const rtp::packet& packet) {
    // Check to make sure a socket for the port exists
    if (_ports[packet.port].isOpen() && _ports[packet.port].hasTXCallback()) {
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
            packet.payload_size, packet.port);
    }
}

void CommModule::receive(const rtp::packet& packet) {
    // Check to make sure a socket for the port exists
    if (_ports[packet.port].isOpen() && _ports[packet.port].hasRXCallback()) {
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
            packet.payload_size, packet.port);
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

    Console::Flush();
}

void CommModule::PrintHeader(void) { _ports.PrintHeader(); }

void CommModule::ResetCount(unsigned int portNbr) {
    _ports[portNbr].RXPackets() = 0;
    _ports[portNbr].TXPackets() = 0;
}

void CommModule::Close(unsigned int portNbr) { _ports[portNbr].Close(); }

bool CommModule::isReady(void) { return _isReady; }

int CommModule::NumOpenSockets(void) { return _ports.count_open(); }

void CommModule::txLED(DigitalInOut* led) { instance->_txLED = led; }

void CommModule::rxLED(DigitalInOut* led) { instance->_rxLED = led; }
