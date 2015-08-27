#include "CommLink.hpp"

#include "logger.hpp"


// Set the class's constants for streamlined use in other areas of the code
const int CommLink::RX_QUEUE_SIZE = COMM_LINK_RX_QUEUE_SIZE;

const char* COMM_ERR_STRING[] = { FOREACH_COMM_ERR(GENERATE_STRING) };

unsigned int CommLink::_nbr_links = 0;


// =================== MAIN CONSTRUCTOR ==============
CommLink::CommLink(PinName mosi, PinName miso, PinName sck, PinName cs, PinName int_pin)
    : _rxQueueHelper()
{
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

    // [X] - 2 - Define the thread task for controlling the RX queue
    // =================
    define_thread(_rxDef, &CommLink::rxThread);

    // [X] - 3 - Create the thread and pass it a pointer to the created object
    // =================
    _rxID = osThreadCreate(&_rxDef, (void*)this);
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
        _int_in->mode(PullUp);
    }
}


// =================== RX THREAD ===================
// Task operations for placing received data into the received data queue
void CommLink::rxThread(void const* arg)
{
    CommLink* inst = (CommLink*)arg;

    // Store our priority so we know what to reset it to if ever needed
    osPriority threadPriority;

    // Only continue past this point once the hardware link is initialized
    osSignalWait(COMM_LINK_SIGNAL_START_THREAD, osWaitForever);

    if (inst->_rxID != nullptr)
        threadPriority  = osThreadGetPriority(inst->_rxID);
    else
        threadPriority = osPriorityIdle;

    LOG(INIT, "RX communication link ready!\r\n    Thread ID:\t%u\r\n    Priority:\t%d", inst->_rxID, threadPriority);

    while ( true ) {
        osSignalWait(COMM_LINK_SIGNAL_RX_TRIGGER, osWaitForever);
        osDelay(1000);
    }

    // Set the function to call on an interrupt trigger
    // inst->_int_in->rise(inst, &CommLink::ISR);

    RTP_t p;

    while (true) {
        // [X] - 1 - Wait until new data has arrived - this is interrupt triggered by CommLink::ISR()
        // =================
        osSignalWait(COMM_LINK_SIGNAL_RX_TRIGGER, osWaitForever);

        // [X] - 2 - Get the received data from the external chip
        // =================
        uint8_t rec_bytes = RTP_MAX_DATA_SIZE;
        int32_t response = inst->getData(p.raw, &rec_bytes);

        LOG(INF1, "Interrupt manually disabled right now");

        if (response == COMM_SUCCESS) {

            // [X] - 3 - Write the data to the CommModule object's rxQueue
            // =================
            CommModule::receive(p);

            // [~] - 4 - Blink the RX LED for the hardware link
            // =================
        }
    }   // while()

    osThreadTerminate(inst->_rxID);
}


// Called by the derived class to begin thread operations
void CommLink::ready(void)
{
    osSignalSet(_rxID, COMM_LINK_SIGNAL_START_THREAD);
}


void CommLink::sendPacket(RTP_t* p)
{
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
