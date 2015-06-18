#include "CommModule.hpp"

// Set the class's constants for streamlined use in other areas of the code
const int CommModule::TX_QUEUE_SIZE = COMM_MODULE_TX_QUEUE_SIZE;
const int CommModule::RX_QUEUE_SIZE = COMM_MODULE_RX_QUEUE_SIZE;
const int CommModule::NBR_PORTS = COMM_MODULE_NBR_PORTS;

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

    // Initialize boolean arrays
    memset(_txH_called, 0, COMM_MODULE_NBR_PORTS);
    memset(_rxH_called, 0, COMM_MODULE_NBR_PORTS);
}

CommModule::~CommModule()
{
    if (_open_ports)
        delete _open_ports;
}

void CommModule::txThread(void const *arg)
{
    CommModule *inst = (CommModule*)arg;

    // Only continue past this point once at least one (1) hardware link is initialized
    osSignalWait(COMM_MODULE_SIGNAL_START_THREAD, osWaitForever);

    log(INF1, "CommModule", "TX Communication Module Ready!");

    while(1) {

        // When a new RTP packet is put in the tx queue, begin operations (does nothing if no new data in queue)
        osEvent evt = osMailGet(inst->_txQueue, osWaitForever);

        if (evt.status == osEventMail) {

            // Get a pointer to the packet's memory location
            RTP_t *p = (RTP_t*)evt.value.p;

            // Send the packet on the active communication link
            inst->_tx_handles[p->port].call(p);

            log(INF1, "CommModule", "Transmission:  Port: %u  Subclass: %u  Bytes: %u  Flags: SFS[%c], ACK[%c]\r\n", p->port, p->subclass, p->payload_size, (p->sfs ? 'X':' '), (p->ack ? 'X':' '));

            // Release the allocated memory once data is sent
            osMailFree(inst->_txQueue, p);
        }
    }
}

void CommModule::rxThread(void const *arg)
{
    CommModule *inst = (CommModule*)arg;

    // Only continue past this point once at least one (1) hardware link is initialized
    osSignalWait(COMM_MODULE_SIGNAL_START_THREAD, osWaitForever);

    log(INF1, "CommModule", "RX Communication Module Ready!");

    RTP_t *p;
    osEvent  evt;
    
    while(1) {

        // Wait until new data is placed in the class's rxQueue from a CommLink class
        evt = osMailGet(inst->_rxQueue, osWaitForever);

        if (evt.status == osEventMail) {
            
            // get a pointer to where the data is stored
            p = (RTP_t*)evt.value.p;

            // If there is an open socket for the port, call it.
            if (std::binary_search(inst->_open_ports->begin(), inst->_open_ports->end(), p->port)) {
                inst->_rx_handles[p->port].call(p);
            }

            log(INF1, "CommModule", "Reception: \r\n  Port: %u\r\n  Subclass: %u", p->port, p->subclass);

            osMailFree(inst->_rxQueue, p);  // free memory allocated for mail
        }
    }
}

void CommModule::TxHandler(void(*ptr)(RTP_t*), uint8_t portNbr)
{
    _txH_called[portNbr] = true;
    ready();
    _tx_handles[portNbr].attach(ptr);
}

void CommModule::RxHandler(void(*ptr)(RTP_t*), uint8_t portNbr)
{
    _rxH_called[portNbr] = true;
    ready();
    _rx_handles[portNbr].attach(ptr);
}

void CommModule::RxHandler(void(*ptr)(void), uint8_t portNbr)
{
    _rxH_called[portNbr] = true;
    ready();
    _rx_handles[portNbr].attach(ptr);
}


void CommModule::openSocket(uint8_t portNbr)
{
    ready();
    if (_txH_called[portNbr] & _rxH_called[portNbr]) {
        if (std::binary_search(_open_ports->begin(), _open_ports->end(), portNbr)) {
            log(WARN, "CommModule", "Port number %u already opened", portNbr);
        } else {
            // [X] - 1 - Add the port number to the list of active ports & keep sorted
            _open_ports->push_back(portNbr);
            std::sort(_open_ports->begin(), _open_ports->end());
            log(INF1, "CommModule", "Port %u opened", portNbr);
        }
    } else {
        // TX callback function was never set
        log(WARN, "CommModule", "Must set TX & RX callback functions before opening socket.\r\n");
    }
}


void CommModule::ready(void)
{
    static bool isReady = false;

    if (isReady) {
        return;
    }

    isReady = true;

    _open_ports = new std::vector<uint8_t>;

    osSignalSet(_txID, COMM_MODULE_SIGNAL_START_THREAD);
    osSignalSet(_rxID, COMM_MODULE_SIGNAL_START_THREAD);
}


void CommModule::send(RTP_t& packet)
{
    // [X] - 1 - Check to make sure a socket for the port exists
    if (std::binary_search(_open_ports->begin(), _open_ports->end(), packet.port)) {

        // [X] - 1.1 - Allocate a block of memory for the data.
        // =================
        RTP_t *p = (RTP_t*)osMailAlloc(_txQueue, osWaitForever);

        // [X] - 1.2 - Copy the contents into the allocated memory block
        // =================
        p->total_size = packet.payload_size + packet.sfs + 3;

        for (int i=0; i < p->total_size + 1; i++) // no need to include rssi & lqi values (the last 2 bytes)
            p->raw[i] = packet.raw[i];

        // [X] - 1.3 - Place the passed packet into the txQueue.
        // =================
        osMailPut(_txQueue, p);
    } else {
        log(WARN, "CommModule", "Failed to send %u byte packet: There is no open socket for port %u", packet.payload_size, packet.port);
    }
}


void CommModule::receive(RTP_t& packet)
{
    // [X] - 1 - Check to make sure a socket for the port exists
    if (std::binary_search(_open_ports->begin(), _open_ports->end(), packet.port)) {

        // [X] - 1.1 - Allocate a block of memory for the data.
        // =================
        RTP_t *p = (RTP_t*)osMailAlloc(_rxQueue, osWaitForever);

        // [X] - 1.2 - Copy the contents into the allocated memory block
        // =================
        for (int i=0; i<packet.total_size + 1; i++)
            p->raw[i] = packet.raw[i];

        // [X] - 1.3 - Place the passed packet into the txQueue.
        // =================
        osMailPut(_rxQueue, p);
    } else {
        log(WARN, "CommModule", "Failed to receive %u byte packet: There is no open socket for port %u", packet.payload_size, packet.port);
    }
}
