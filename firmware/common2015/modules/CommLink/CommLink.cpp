#include "CommLink.hpp"

#include "logger.hpp"


// Set the class's constants for streamlined use in other areas of the code
const int CommLink::RX_QUEUE_SIZE = COMM_LINK_RX_QUEUE_SIZE;

const char* COMM_ERR_STRING[] = { FOREACH_COMM_ERR(GENERATE_STRING) };


CommLink::CommLink(PinName mosi,
    PinName miso,
    PinName sck,
    PinName cs,
    PinName int_pin)
    : _cs(cs), _int_in(int_pin),
      _spi(mosi, miso, sck),
      _rxQueueHelper(), _miso_pin(miso)
{
    chip_deselect();

    // spi settings
    _spi.format(8, 0);
    _spi.frequency(5000000);

    // interrupt pin settings
    _int_in.mode(PullUp);

    // Define the thread task for controlling the RX queue
    define_thread(_rxDef, &CommLink::rxThread);

    // Create the thread and pass it a pointer to the created object
    _rxID = osThreadCreate(&_rxDef, (void*)this);
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

    // Set the function to call on an interrupt trigger
    inst->_int_in.rise(inst, &CommLink::ISR);

    rtp::packet p;

    while (true) {
        // [X] - 1 - Wait until new data has arrived - this is interrupt triggered by CommLink::ISR()
        // =================
        osSignalWait(COMM_LINK_SIGNAL_RX_TRIGGER, osWaitForever);

        // [X] - 2 - Get the received data from the external chip
        // =================
        uint8_t rec_bytes = rtp::MAX_DATA_SZ;
        int32_t response = inst->getData(p.raw, &rec_bytes);

        LOG(INF3, "RX interrupt triggered");

        if (response == COMM_SUCCESS) {

            // [X] - 3 - Write the data to the CommModule object's rxQueue
            // =================
            CommModule::receive(p);
        }
    }

    osThreadTerminate(inst->_rxID);
}


// Called by the derived class to begin thread operations
void CommLink::ready()
{
    osSignalSet(_rxID, COMM_LINK_SIGNAL_START_THREAD);
}


void CommLink::sendPacket(rtp::packet* p)
{
    sendData(p->raw, p->total_size);
}


// Interrupt Service Routine - KEEP OPERATIONS TO ABOSOLUTE MINIMUM HERE AND IN ANY OVERRIDEN BASE CLASS IMPLEMENTATIONS OF THIS CLASS METHOD
void CommLink::ISR()
{
    osSignalSet(_rxID, COMM_LINK_SIGNAL_RX_TRIGGER);
}
