#include "CommLink.hpp"


// Set the class's constants for streamlined use in other areas of the code
const int CommLink::TX_QUEUE_SIZE = COMM_LINK_TX_QUEUE_SIZE;
const int CommLink::RX_QUEUE_SIZE = COMM_LINK_RX_QUEUE_SIZE;


// =================== CONSTRUCTORS ===================
// Default constructor
CommLink::CommLink()
{
}

CommLink::CommLink(PinName mosi, PinName miso, PinName sck, PinName cs, PinName int_pin) :
    _txQueueHelper(),
    _rxQueueHelper()
{
    static unsigned int _nbr_links = 0;

    setup_pins(mosi, miso, sck, cs, int_pin);
    setup();
    _nbr_links++;
}


// =================== CLASS SETUP ===================
void CommLink::setup()
{
    // [X] - 1 - Initialize the hardware for communication.
    // =================
    setup_spi();
    setup_cs();
    setup_interrupt();

    // [X] - 2 - Define the thread tasks for controlling the data queues
    // =================
    define_thread(_txDef, &CommLink::txThread);
    define_thread(_rxDef, &CommLink::rxThread);

    // [X] - 3 - Create the threads and pass them a pointer to the created object
    // =================
    _txID = osThreadCreate(&_txDef, (void *)this);
    _rxID = osThreadCreate(&_rxDef, (void *)this);
}


// =================== PIN SETUP ===================
void CommLink::setup_pins(PinName mosi, PinName miso, PinName sck, PinName cs, PinName int_pin)
{
    _mosi_pin = mosi;
    _miso_pin = miso;
    _sck_pin = sck;
    _cs_pin = cs;
    _int_pin = int_pin;
}

void CommLink::setup_spi(void)
{
    if ((_mosi_pin != NC) & (_miso_pin != NC) & (_sck_pin != NC)) {
        _spi = new SPI(_mosi_pin, _miso_pin, _sck_pin);    // DON'T FORGET TO DELETE IN DERIVED CLASS
        _spi->format(8, 0);
        _spi->frequency(5000000);
    }
}

void CommLink::setup_cs(void)
{
    if (_cs_pin != NC) {
        _cs = new DigitalOut(_cs_pin);    // DON'T FORGET TO DELETE IN DERIVED CLASS
        *_cs = 1;    // default to active low signal
    }
}

void CommLink::setup_interrupt(void)
{
    if (_int_pin != NC) {
        _int_in = new InterruptIn(_int_pin);    // DON'T FORGET TO DELETE IN DERIVED CLASS
        _int_in->mode(PullDown);
    }
}


// =================== TX/RX THREADS ===================
// Task operations for sending data over the hardware link when a new item is placed in the queue
void CommLink::txThread(void const *arg)
{
    //CommLink *inst = (CommLink*)arg;

    // Only continue past this point once the hardware link is initialized
    osSignalWait(COMM_LINK_SIGNAL_START_THREAD, osWaitForever);

    while (1) {
        // [X] - 1 - Wait until the CommModule class sends a signal to begin operation on new data being placed in its txQueue
        // =================
        osSignalWait(COMM_LINK_SIGNAL_TX_TRIGGER, osWaitForever);

        // [] - 2 - Copy the packet from the CommModule txQueue into the CommLink txQueue
        // =================
        //void *  osMailAlloc (osMailQId queue_id, uint32_t millisec);

        // [] - 3 - Call the method for sending the packet over a hardware communication link
        // =================

        // [] - 4 - Blink the TX LED for the hardware link
        // =================
    }
}


// Task operations for placing received data into the received data queue
void CommLink::rxThread(void const *arg)
{
    CommLink *inst = (CommLink *)arg;

    // Only continue past this point once the hardware link is initialized
    osSignalWait(COMM_LINK_SIGNAL_START_THREAD & COMM_LINK_SIGNAL_MODULE_LINKED, osWaitForever);

    // Set the function to call on an interrupt trigger
    inst->_int_in->rise(inst, &CommLink::ISR);

    while (1) {
        // [X] - 1 - Wait until new data has arrived - this is interrupt triggered by CommLink::ISR()
        // =================
        osSignalWait(COMM_LINK_SIGNAL_RX_TRIGGER, osWaitForever);

        // [X] - 2 - Get the received data from the external chip
        // =================
        uint8_t rec_bytes = RTP_MAX_DATA_SIZE;
        RTP_t p;

        if (inst->getData(p.raw, &rec_bytes)) {

            // force some numbers for testing always on port 8
            p.port = 8;
            p.subclass = 0;

            //p.port = p.data[0] >> 4;
            //p.subclass = p.data[0] & 0x0F;

            // [X] - 3 - Write the data to the CommModule object's rxQueue
            // =================
            inst->_comm_module->receive(p);

            // [~] - 4 - Blink the RX LED for the hardware link
            // =================

        }
    }
}


// Called by the derived class to begin thread operations
void CommLink::ready(void)
{
    //osSignalSet(_txID, COMM_LINK_SIGNAL_START_THREAD);
    osSignalSet(_rxID, COMM_LINK_SIGNAL_START_THREAD);
}


void CommLink::sendPacket(RTP_t *p)
{
    p->payload_size = p->total_size - 1; // fixup factor for headers. Exclude the `size` byte from being counted
    sendData(p->raw, p->total_size);
}


// Interrupt Service Routine - KEEP OPERATIONS TO ABOSOLUTE MINIMUM HERE AND IN ANY OVERRIDEN BASE CLASS IMPLEMENTATIONS OF THIS CLASS METHOD
void CommLink::ISR(void)
{
    osSignalSet(_rxID , COMM_LINK_SIGNAL_RX_TRIGGER);
}


void CommLink::toggle_cs(void)
{
    *_cs = !*_cs;
}


void CommLink::setModule(CommModule &com)
{
    _comm_module = &com;
    osSignalSet(_rxID , COMM_LINK_SIGNAL_MODULE_LINKED);
}


unsigned int CommLink::rxPackets(void)
{
    return _comm_module->NumRXPackets();
}

unsigned int CommLink::txPackets(void)
{
    return _comm_module->NumTXPackets();
}
