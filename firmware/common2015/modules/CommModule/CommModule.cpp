#include "CommModule.hpp"

#include "logger.hpp"


// Set the class's constants for streamlined use in other areas of the code
const int       CommModule::TX_QUEUE_SIZE = COMM_MODULE_TX_QUEUE_SIZE;
const int       CommModule::RX_QUEUE_SIZE = COMM_MODULE_RX_QUEUE_SIZE;
const int       CommModule::NBR_PORTS = COMM_MODULE_NBR_PORTS;

// Class declarations since everything in CommModule is static
bool            CommModule::isReady = false;
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
    // Only continue past this point once at least one (1) hardware link is initialized
    osSignalWait(COMM_MODULE_SIGNAL_START_THREAD, osWaitForever);

    LOG(INIT, "TX Communication Module Ready!\r\n    Thread ID:\t%u\r\n", _txID);

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
                if (_ports[p->port].isOpen()) {

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
}


void CommModule::rxThread(void const* arg)
{
    // Only continue past this point once at least one (1) hardware link is initialized
    osSignalWait(COMM_MODULE_SIGNAL_START_THREAD, osWaitForever);

    LOG(INIT, "RX Communication Module Ready!\r\n    Thread ID:\t%u\r\n", _rxID);

    RTP_t* p;
    osEvent  evt;

    while (1) {

        // Wait until new data is placed in the class's rxQueue from a CommLink class
        evt = osMailGet(instance->_rxQueue, osWaitForever);

        if (evt.status == osEventMail) {
            // get a pointer to where the data is stored
            p = (RTP_t*)evt.value.p;

            // Call the user callback function (if set)
            if (_ports[p->port].isOpen()) {

                _ports[p->port].RXCallback()(p);

                _ports[p->port].RXPackets()++;

                LOG(INF3, "Reception: \r\n  Port: %u\r\n  Subclass: %u", p->port, p->subclass);
            }

            osMailFree(instance->_rxQueue, p);  // free memory allocated for mail
        }
    }
}


void CommModule::RxHandler(void(*ptr)(RTP_t*), uint8_t portNbr)
{
    if ( !_ports[portNbr].Exists() ) {

        CommPort_t _tmpPort(portNbr);

        _tmpPort.RXCallback() = std::bind(ptr, std::placeholders::_1);

        _ports += _tmpPort;

    } else {

        _ports[portNbr].RXCallback() = std::bind(ptr, std::placeholders::_1);
    }

    ready();
}


bool CommModule::openSocket(uint8_t portNbr)
{
    if ( _ports[portNbr].Open() ) {
        // Everything looks to be setup, go ahead and enable it
        LOG(INIT, "Port %u opened", portNbr);

        return true;
    } else {
        // TX callback function was never set
        LOG(WARN, "Must set TX & RX callback functions before opening socket.\r\n");

        return false;
    }
}


void CommModule::ready(void)
{
    if (isReady == true)
        return;

    // Start running the TX thread - it will trigger with to startup the RX thread
    osSignalSet(_txID, COMM_MODULE_SIGNAL_START_THREAD);

    isReady = true;
}


void CommModule::send(RTP_t& packet)
{
    // [X] - 1 - Check to make sure a socket for the port exists
    if ( _ports[packet.port].isOpen() ) {

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
        LOG(WARN, "Failed to send %u byte packet: There is no open socket for port %u", packet.payload_size, packet.port);
    }
}


void CommModule::receive(RTP_t& packet)
{
    // [X] - 1 - Check to make sure a socket for the port exists
    if ( _ports[packet.port].isOpen() ) {

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
        LOG(WARN, "Failed to receive %u byte packet: There is no open socket for port %u", packet.payload_size, packet.port);
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


void comm_cmdProcess(const vector<string>& args)
{
    CommModule::PrintInfo(true);
}
