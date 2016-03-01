#include "CommLink.hpp"

#include "logger.hpp"
#include "assert.hpp"

#define COMM_LINK_SIGNAL_START_THREAD (1 << 0)
#define COMM_LINK_SIGNAL_RX_TRIGGER (1 << 1)

const char* COMM_ERR_STRING[] = {FOREACH_COMM_ERR(GENERATE_STRING)};

CommLink::CommLink(PinName mosi, PinName miso, PinName sck, PinName cs,
                   PinName int_pin) {
    setup_pins(mosi, miso, sck, cs, int_pin);
    setup();
}

CommLink::~CommLink() {
    // release created pin objects if they exist
    if (_spi) delete _spi;
    if (_cs) delete _cs;
    if (_int_in) delete _int_in;

    // terminate the thread we created
    delete _rxThread;
}

void CommLink::setup() {
    // Initialize the hardware for communication
    setup_spi();
    setup_cs();
    setup_interrupt();

    // initialize and start thread
    _rxThread = new Thread(&CommLink::rxThreadHelper, this);
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
void CommLink::rxThread() {
    // Only continue past this point once the hardware link is initialized
    Thread::signal_wait(COMM_LINK_SIGNAL_START_THREAD);

    // Store our priority so we know what to reset it to if ever needed
    const osPriority threadPriority = _rxThread->get_priority();

    LOG(INIT, "RX communication link ready!\r\n    Thread ID: %u, Priority: %d",
        _rxThread->gettid(), threadPriority);

    // Set the function to call on an interrupt trigger
    _int_in->rise(this, &CommLink::ISR);

    rtp::packet p;
    std::vector<uint8_t> buf;
    buf.reserve(120);

    while (true) {
        // Wait until new data has arrived
        // this is triggered by CommLink::ISR()
        Thread::signal_wait(COMM_LINK_SIGNAL_RX_TRIGGER);

        // Get the received data from the external chip
        uint8_t rec_bytes = rtp::MAX_DATA_SZ;
        int32_t response = getData(buf.data(), &rec_bytes);

        LOG(INF3, "RX interrupt triggered");

        if (response == COMM_SUCCESS) {
            // Write the data to the CommModule object's rxQueue
            p.recv(buf);
            CommModule::Instance()->receive(p);
            buf.clear();
        }
    }
}

// Called by the derived class to begin thread operations
void CommLink::ready() { _rxThread->signal_set(COMM_LINK_SIGNAL_START_THREAD); }

void CommLink::sendPacket(rtp::packet* p) {
    std::vector<uint8_t> buffer;
    p->pack(&buffer);
    sendData(buffer.data(), buffer.size());
}

void CommLink::ISR() { _rxThread->signal_set(COMM_LINK_SIGNAL_RX_TRIGGER); }

void CommLink::radio_select() { *_cs = 0; }

void CommLink::radio_deselect() { *_cs = 1; }

uint8_t CommLink::twos_compliment(uint8_t val) { return 1 -(unsigned int)val; }
