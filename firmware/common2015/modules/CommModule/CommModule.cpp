#include "CommModule.hpp"

#include "logger.hpp"


// Set the class's constants for streamlined use in other areas of the code
const int       CommModule::TX_QUEUE_SIZE = COMM_MODULE_TX_QUEUE_SIZE;
const int       CommModule::RX_QUEUE_SIZE = COMM_MODULE_RX_QUEUE_SIZE;
const int       CommModule::NBR_PORTS = COMM_MODULE_NBR_PORTS;

// Class declarations since everything in CommModule is static
bool            CommModule::_isReady = false;
osThreadId      CommModule::_txID;
osThreadId      CommModule::_rxID;
CommPorts_t     CommModule::_ports;

std::shared_ptr<CommModule> CommModule::instance;

// This one isn't apart of any class, but it's an extern in CommModule.hpp
CommPort_t      _tmpPort;


// Default constructor
CommModule::CommModule() :
    // [X] - 1.1 - Define the data queues.
    // =================
    _txQueueHelper(),
    _rxQueueHelper()
{}


void CommModule::Init(void)
{
    // [X] - 1.0 - Make sure we have an instance to work with
    auto instance = Instance();

    // [X] - 1.1 - Create the data queues.
    // =================
    instance->_txQueue = osMailCreate(instance->_txQueueHelper.def(), NULL);
    instance->_rxQueue = osMailCreate(instance->_rxQueueHelper.def(), NULL);

    // [X] - 1.2 - Define the TX & RX task threads.
    // =================
    define_thread(instance->_txDef, &CommModule::txThread);
    define_thread(instance->_rxDef, &CommModule::rxThread);

    // [X] - 1.3 - Create the TX & RX threads - pass them a pointer to the created object.
    // =================
    _txID = osThreadCreate(&(instance->_txDef), NULL);
    _rxID = osThreadCreate(&(instance->_rxDef), NULL);
}


shared_ptr<CommModule>& CommModule::Instance(void)
{
    if (instance.get() == nullptr)
        instance.reset(new CommModule);

    return instance;
}


void CommModule::txThread(void const* arg)
{
    // Store our priority so we know what to reset it to if ever needed
    osPriority threadPriority;

    // Only continue past this point once at least one (1) hardware link is initialized
    osSignalWait(COMM_MODULE_SIGNAL_START_THREAD, osWaitForever);

    if (_txID != nullptr)
        threadPriority  = osThreadGetPriority(_txID);
    else
        threadPriority = osPriorityIdle;

    LOG(INIT, "TX communication module ready!\r\n    Thread ID:\t%u\r\n    Priority:\t%d", _txID, threadPriority);

    osSignalSet(_rxID, COMM_MODULE_SIGNAL_START_THREAD);

    osEvent  evt;

    while (1) {

        // When a new RTP packet is put in the tx queue, begin operations (does nothing if no new data in queue)
        evt = osMailGet(instance->_txQueue, osWaitForever);

        if (evt.status == osEventMail) {

            // Get a pointer to the packet's memory location
            RTP_t* p = (RTP_t*)evt.value.p;

            // Bump up the thread's priority
            if (osThreadSetPriority(_txID, osPriorityRealtime) == osOK) {

                // Call the user callback function
                if (_ports[p->port].isOpen() ) {

                    _ports[p->port].TXCallback()(p);

                    _ports[p->port].TXPackets()++;   // Increment the packet counter by 1

                    LOG(INF3, "Transmission:    Port: %u    Subclass: %u", p->port, p->subclass);
                }

                // Release the allocated memory once data is sent
                osMailFree(instance->_txQueue, p);

                osThreadSetPriority(_txID, osPriorityNormal);
            }
        }
    }

    osThreadTerminate(_txID);
}


void CommModule::rxThread(void const* arg)
{
    // Store our priority so we know what to reset it to if ever needed
    osPriority threadPriority;

    // Only continue past this point once at least one (1) hardware link is initialized
    osSignalWait(COMM_MODULE_SIGNAL_START_THREAD, osWaitForever);

    if (_rxID != nullptr)
        threadPriority  = osThreadGetPriority(_rxID);
    else
        threadPriority = osPriorityIdle;

    LOG(INIT, "RX communication module ready!\r\n    Thread ID:\t%u\r\n    Priority:\t%d", _rxID, threadPriority);

    RTP_t* p;
    osEvent  evt;

    while (1) {

        // Wait until new data is placed in the class's rxQueue from a CommLink class
        evt = osMailGet(instance->_rxQueue, osWaitForever);

        if (evt.status == osEventMail) {
            // get a pointer to where the data is stored
            p = (RTP_t*)evt.value.p;

            // Bump up the thread's priority
            if (osThreadSetPriority(_rxID, osPriorityRealtime) == osOK) {
                // Call the user callback function (if set)
                if (_ports[p->port].isOpen() ) {

                    _ports[p->port].RXCallback()(p);

                    _ports[p->port].RXPackets()++;

                    LOG(INF3, "Reception: \r\n  Port: %u\r\n  Subclass: %u", p->port, p->subclass);
                }

                osMailFree(instance->_rxQueue, p);  // free memory allocated for mail

                osThreadSetPriority(_rxID, osPriorityNormal);
            }
        }
    }

    osThreadTerminate(_rxID);
}


void CommModule::RxHandler(void(*ptr)(RTP_t*), uint8_t portNbr)
{
    if ( !_ports[portNbr].Exists() ) {

        CommPort_t _tmpPort(portNbr);

        _tmpPort.RXCallback() = std::bind(ptr, std::placeholders::_1);

        _ports += _tmpPort;            // get a pointer to where the data is stored


    } else {

        _ports[portNbr].RXCallback() = std::bind(ptr, std::placeholders::_1);
    }

    ready();
}

void CommModule::TxHandler(void(*ptr)(RTP_t*), uint8_t portNbr)
{
    if ( !_ports[portNbr].Exists() ) {

        CommPort_t _tmpPort(portNbr);

        _tmpPort.TXCallback() = std::bind(ptr, std::placeholders::_1);

        _ports += _tmpPort;

    } else {

        _ports[portNbr].TXCallback() = std::bind(ptr, std::placeholders::_1);
    }

    ready();
}

bool CommModule::openSocket(uint8_t portNbr)
{
    if ( !_ports[portNbr].Exists() ) {
        CommPort_t _tmpPort(portNbr);
        _ports += _tmpPort;

        LOG(WARN, "Port %u established, but has no set callbacks", portNbr);

        ready();

        return _ports[portNbr].Open();
    }

    if ( _ports[portNbr].Open() ) {
        // Everything looks to be setup, go ahead and enable it
        LOG(INIT, "Port %u opened", portNbr);

        return true;
    } else {
        // this almost never gets called. I'm probably going to remove it soon. Only makes it this far if trying to open port 0 without any setup.

        // TX callback function was never set
        LOG(WARN, "Must set TX & RX callback functions before opening socket.\r\n");
        return false;
    }
}


void CommModule::ready(void)
{
    if (_isReady == true)
        return;

    // Start running the TX thread - it will trigger with to startup the RX thread
    osSignalSet(_txID, COMM_MODULE_SIGNAL_START_THREAD);

    _isReady = true;
}


void CommModule::send(RTP_t& packet)
{
    // [X] - 1 - Check to make sure a socket for the port exists
    if ( _ports[packet.port].isOpen() && _ports[packet.port].TXCallback() ) {

        packet.adjustSizes();

        // [X] - 1.1 - Allocate a block of memory for the data.
        // =================
        RTP_t* p = (RTP_t*)osMailAlloc(instance->_txQueue, osWaitForever);

        // [X] - 1.2 - Copy the contents into the allocated memory block
        // =================
        std::memcpy(p, &packet, packet.total_size);

        // [X] - 1.3 - Place the passed packet into the txQueue.
        // =================
        osMailPut(instance->_txQueue, p);

    } else {
        LOG(WARN, "Failed to send %u byte packet: There is no open transmitting socket for port %u", packet.payload_size, packet.port);
    }
}


void CommModule::receive(RTP_t& packet)
{
    // [X] - 1 - Check to make sure a socket for the port exists
    if ( _ports[packet.port].isOpen() && _ports[packet.port].RXCallback() ) {

        // [X] - 1.1 - Allocate a block of memory for the data.
        // =================
        RTP_t* p = (RTP_t*)osMailAlloc(instance->_rxQueue, osWaitForever);

        // [X] - 1.2 - Copy the contents into the allocated memory block
        // =================
        std::memcpy(p, &packet, packet.total_size);

        // [X] - 1.3 - Place the passed packet into the rxQueue.
        // =================
        osMailPut(instance->_rxQueue, p);

    } else {
        LOG(WARN, "Failed to receive %u byte packet: There is no open receiving socket for port %u", packet.payload_size, packet.port);
    }
}


unsigned int CommModule::NumRXPackets(void)
{
    return _ports.allRXPackets();
}


unsigned int CommModule::NumTXPackets(void)
{
    return _ports.allTXPackets();
}


void CommModule::PrintInfo(bool forceHeader)
{
    if (forceHeader == true && _ports.empty()) {
        PrintHeader();
        _ports.PrintFooter();
    } else {
        _ports.PrintPorts();
        _ports.PrintFooter();
    }

    Console::Flush();
}


void CommModule::PrintHeader(void)
{
    _ports.PrintHeader();
}


void CommModule::ResetCount(unsigned int portNbr)
{
    _ports[portNbr].RXPackets() = 0;
    _ports[portNbr].TXPackets() = 0;
}

void CommModule::Close(unsigned int portNbr)
{
    _ports[portNbr].Close();
}

bool CommModule::isReady(void)
{
    return _isReady;
}
