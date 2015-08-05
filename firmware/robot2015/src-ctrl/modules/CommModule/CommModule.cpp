#include "CommModule.hpp"

// Set the class's constants for streamlined use in other areas of the code
const int       CommModule::TX_QUEUE_SIZE = COMM_MODULE_TX_QUEUE_SIZE;
const int       CommModule::RX_QUEUE_SIZE = COMM_MODULE_RX_QUEUE_SIZE;
const int       CommModule::NBR_PORTS = COMM_MODULE_NBR_PORTS;
unsigned int    CommModule::txPackets = 0;
unsigned int    CommModule::rxPackets = 0;
osThreadId      CommModule::_txID;
osThreadId      CommModule::_rxID;
osMailQId       CommModule::_txQueue;
osMailQId       CommModule::_rxQueue;
bool            CommModule::isReady = false;
CommPorts_t     CommModule::_ports;

CommPort_t      portAdd;


// Default constructor
CommModule::CommModule() :
    // [X] - 1.1 - Define the data queues.
    // =================
    _txQueueHelper(),
    _rxQueueHelper()
{
    // [X] - 1.2 - Create the data queues.
    // =================
    _txQueue = osMailCreate(_txQueueHelper.def(), NULL);
    _rxQueue = osMailCreate(_rxQueueHelper.def(), NULL);

    // [X] - 2.1 - Define the TX & RX task threads.
    // =================
    define_thread(_txDef, &CommModule::txThread);
    define_thread(_rxDef, &CommModule::rxThread);

    // [X] - 2.2 - Create the TX & RX threads - pass them a pointer to the created object.
    // =================
    _txID = osThreadCreate(&_txDef, (void*)this);
    _rxID = osThreadCreate(&_rxDef, (void*)this);

    //_tx_handles.reserve(COMM_MODULE_NBR_PORTS);
    //_rx_handles.reserve(COMM_MODULE_NBR_PORTS);
    // _ports.reserve(COMM_MODULE_NBR_PORTS);
}

CommModule::~CommModule()
{
    //if (_open_ports)
    //delete _open_ports;
}

void CommModule::txThread(void const* arg)
{
    // CommModule* inst = (CommModule*)arg;

    // Only continue past this point once at least one (1) hardware link is initialized
    osSignalWait(COMM_MODULE_SIGNAL_START_THREAD, osWaitForever);

    LOG(INF2, "TX Communication Module Ready!");

    while (1) {

        // When a new RTP packet is put in the tx queue, begin operations (does nothing if no new data in queue)
        osEvent evt = osMailGet(_txQueue, osWaitForever);

        if (evt.status == osEventMail) {
            // Get a pointer to the packet's memory location
            RTP_t* p = (RTP_t*)evt.value.p;

            // Call the user callback function
            if (_ports[p->port].isOpen()) {
                _ports[p->port].tx_callback(p);

                LOG(INF3, "Transmission:    Port: %u    Subclass: %u", p->port, p->subclass);
            }

            // Release the allocated memory once data is sent
            osMailFree(_txQueue, p);
        }
    }
}

void CommModule::rxThread(void const* arg)
{
    // CommModule* inst = (CommModule*)arg;

    // Only continue past this point once at least one (1) hardware link is initialized
    osSignalWait(COMM_MODULE_SIGNAL_START_THREAD, osWaitForever);

    LOG(INF2, "RX Communication Module Ready!");

    RTP_t* p;
    osEvent  evt;

    while (1) {

        // Wait until new data is placed in the class's rxQueue from a CommLink class
        evt = osMailGet(_rxQueue, osWaitForever);

        if (evt.status == osEventMail) {
            // get a pointer to where the data is stored
            p = (RTP_t*)evt.value.p;

            // Call the user callback function (if set)
            if (_ports[p->port].isOpen()) {
                _ports[p->port].rx_callback(p);

                LOG(INF3, "Reception: \r\n  Port: %u\r\n  Subclass: %u", p->port, p->subclass);
            }

            osMailFree(_rxQueue, p);  // free memory allocated for mail
        }
    }
}

void CommModule::RxHandler(void(*ptr)(RTP_t*), uint8_t portNbr)
{
    if ( !_ports[portNbr].Exists() ) {

        portAdd = CommPort_t(portNbr);

        portAdd.RXCallback(std::bind(ptr, std::placeholders::_1));

        _ports += portAdd;

    } else if ( _ports[portNbr].hasTXCallback() ) {

        _ports[portNbr].RXCallback(std::bind(ptr, std::placeholders::_1));
    } else {

        return;
    }

    ready();
}

bool CommModule::openSocket(uint8_t portNbr)
{
    if ( _ports[portNbr].Open() ) {
        LOG(INF3, "Port %u opened", portNbr);

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

    isReady = true;

    osSignalSet(_txID, COMM_MODULE_SIGNAL_START_THREAD);
    osSignalSet(_rxID, COMM_MODULE_SIGNAL_START_THREAD);
}

void CommModule::send(RTP_t& packet)
{
    // [X] - 1 - Check to make sure a socket for the port exists
    //if (std::binary_search(_open_ports->begin(), _open_ports->end(), packet.port)) {
    if ( _ports[packet.port].isOpen() ) {
        //packet.payload_size += 1;   // Fixup factor for header bytes

        // [X] - 1.1 - Allocate a block of memory for the data.
        // =================
        RTP_t* p = (RTP_t*)osMailAlloc(_txQueue, osWaitForever);

        // [X] - 1.2 - Copy the contents into the allocated memory block
        // =================
        std::memcpy(p->raw, &packet.raw, p->total_size);

        // [X] - 1.3 - Place the passed packet into the txQueue.
        // =================
        osMailPut(_txQueue, p);

        _ports[packet.port].TXPackets(1);   // Increment the packet counter by 1
    } else {
        LOG(WARN, "Failed to send %u byte packet: There is no open socket for port %u", packet.payload_size, packet.port);
    }
}

void CommModule::receive(RTP_t& packet)
{
    // [X] - 1 - Check to make sure a socket for the port exists
    if ( _ports[packet.port].isOpen() ) {
        //packet.payload_size -= 1;   // Fixup factor for header bytes

        // [X] - 1.1 - Allocate a block of memory for the data.
        // =================
        RTP_t* p = (RTP_t*)osMailAlloc(_rxQueue, osWaitForever);

        // [X] - 1.2 - Copy the contents into the allocated memory block
        // =================
        std::memcpy(p->raw, &packet.raw, p->total_size);

        // [X] - 1.3 - Place the passed packet into the rxQueue.
        // =================
        osMailPut(_rxQueue, p);

        _ports[packet.port].RXPackets(1);
    } else {
        LOG(WARN, "Failed to receive %u byte packet: There is no open socket for port %u", packet.payload_size, packet.port);
    }
}

unsigned int CommModule::NumRXPackets(void)
{
    return CommModule::rxPackets;
}

unsigned int CommModule::NumTXPackets(void)
{
    return CommModule::txPackets;
}

void CommModule::PrintInfo(bool forceHeader)
{
    if (_ports.empty() == false) {

        PrintHeader();

        //for (pIt = _ports.begin(); pIt != _ports.end(); ++pIt) {
        for (int i = 0; i < _ports.count(); i++) {
            printf("%2u\t%u\t%u\t%s\t\t%s\r\n",
                   _ports[i].Nbr(),
                   _ports[i].RXPackets(),
                   _ports[i].TXPackets(),
                   _ports[i].hasRXCallback() ? "YES" : "NO",
                   _ports[i].hasTXCallback() ? "YES" : "NO"
                  );
        }

        printf("\r\n");
    } else {
        if (forceHeader == true) {
            PrintHeader();
        }
    }
}

void CommModule::PrintHeader(void)
{
    printf("PORT\tIN\tOUT\tRX CBCK\t\tTX CBCK\r\n");
}

void comm_cmdProcess(const vector<string>& args)
{
    CommModule::PrintInfo(true);
}