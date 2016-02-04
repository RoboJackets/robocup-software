#include "CommLink.hpp"

#include "logger.hpp"
#include "assert.hpp"

#define COMM_LINK_SIGNAL_START_THREAD (1 << 0)
#define COMM_LINK_SIGNAL_RX_TRIGGER (1 << 1)

const char* COMM_ERR_STRING[] = {FOREACH_COMM_ERR(GENERATE_STRING)};

CommLink::CommLink(PinName mosi, PinName miso, PinName sck, PinName cs,
                   PinName int_pin)
    : _rxQueueHelper() {
    setup_pins(mosi, miso, sck, cs, int_pin);
    setup();
}

CommLink::~CommLink() { cleanup(); }

void CommLink::cleanup() {
    // release created pin objects if they exist
    if (_spi) delete _spi;
    if (_cs) delete _cs;
    if (_int_in) delete _int_in;

    // terminate the thread we created
    osThreadTerminate(_rxID);

    // release the previsouly allocated stack
    delete[](_rxDef.stack_pointer);
}

void CommLink::setup() {
    // Initialize the hardware for communication
    setup_spi();
    setup_cs();
    setup_interrupt();

    // Define the thread task for controlling the RX queue
    define_thread(_rxDef, &CommLink::rxThread, osPriorityNormal);

    // Create the thread and pass it a pointer to the created object
    _rxID = osThreadCreate(&_rxDef, (void*)this);
}

void CommLink::setup_pins(PinName mosi, PinName miso, PinName sck, PinName cs,
                          PinName int_pin) {
    _mosi_pin = mosi;
    _miso_pin = miso;
    _sck_pin = sck;
    _cs_pin = cs;
    _int_pin = int_pin;
}

void CommLink::setup_spi(int baudrate) {
    if ((_mosi_pin != NC) & (_miso_pin != NC) & (_sck_pin != NC)) {
        _spi = new SPI(_mosi_pin, _miso_pin, _sck_pin);
        _spi->format(8, 0);
        _spi->frequency(baudrate);
    }
}

void CommLink::setup_cs() {
    if (_cs_pin != NC) {
        _cs = new DigitalOut(_cs_pin);
        *_cs = 1;  // default to active low signal
    }
}

void CommLink::setup_interrupt() {
    if (_int_pin != NC) {
        _int_in = new InterruptIn(_int_pin);
        _int_in->mode(PullUp);
    }
}

// =================== RX THREAD ===================
// Task operations for placing received data into the received data queue
void CommLink::rxThread(void const* arg) {
    CommLink* inst =
        const_cast<CommLink*>(reinterpret_cast<const CommLink*>(arg));

    // Only continue past this point once the hardware link is initialized
    osSignalWait(COMM_LINK_SIGNAL_START_THREAD, osWaitForever);

    ASSERT(inst->_rxID != nullptr);
    // Store our priority so we know what to reset it to if ever needed
    osPriority threadPriority = osThreadGetPriority(inst->_rxID);

    LOG(INIT, "RX communication link ready!\r\n    Thread ID: %u, Priority: %d",
        inst->_rxID, threadPriority);

    // Set the function to call on an interrupt trigger
    inst->_int_in->rise(inst, &CommLink::ISR);

    rtp::packet p;
    std::vector<uint8_t> buf;
    buf.reserve(120);

    while (true) {
        // Wait until new data has arrived
        // this is triggered by CommLink::ISR()
        osSignalWait(COMM_LINK_SIGNAL_RX_TRIGGER, osWaitForever);

        // Get the received data from the external chip
        uint8_t rec_bytes = rtp::MAX_DATA_SZ;
        int32_t response = inst->getData(buf.data(), &rec_bytes);

        LOG(INF3, "RX interrupt triggered");

        if (response == COMM_SUCCESS) {
            // Write the data to the CommModule object's rxQueue
            p.recv(buf);
            CommModule::receive(p);
            buf.clear();
        }
    }

    osThreadTerminate(inst->_rxID);
}

// Called by the derived class to begin thread operations
void CommLink::ready() { osSignalSet(_rxID, COMM_LINK_SIGNAL_START_THREAD); }

void CommLink::sendPacket(rtp::packet* p) {
    std::vector<uint8_t> buffer;
    p->pack(&buffer);
    sendData(buffer.data(), buffer.size());
}

void CommLink::ISR() { osSignalSet(_rxID, COMM_LINK_SIGNAL_RX_TRIGGER); }

void CommLink::radio_select() { *_cs = 0; }

void CommLink::radio_deselect() { *_cs = 1; }

uint8_t CommLink::twos_compliment(uint8_t val) { return -(unsigned int)val; }
