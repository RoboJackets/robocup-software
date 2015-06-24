#include "CC1101.hpp"

// Default constructor
CC1101::CC1101() : CommLink() {}


// Main constructor
CC1101::CC1101(PinName mosi, PinName miso, PinName sck, PinName cs, PinName int_pin) :
    CommLink(mosi, miso, sck, cs, int_pin)
{
    // [X] - 1 - Setup the chip and only signal the base class if device is tested and confirmed to be electrically connected
    // =================
    if(powerUp()) {
        
        // [X] - 2a - Don't completely initialize if device fails to successfully start
        // =================
        log(LOG_LEVEL::ERROR, "CC1101", "CC1101 Failure");
    } else {
        
        // [X] - 2b - Call the base class method for beginning full class operation with threads
        // =================
        CommLink::ready();

        log(LOG_LEVEL::INFO, "CC1101", "CC1101 Ready!");
    }
}

// Deconstructor
CC1101::~CC1101()
{
    if (_spi)
        delete _spi;
    if (_cs)
        delete _cs;
    if (_int_in)
        delete _int_in;
}


// Set up everything for the hardware to begin communicating
int32_t CC1101::powerUp(void)
{
#if CCXXX1_DEBUG_MODE > 0
    EVENT("CC1101 RADIO TRANSCEIVER INITILIZATION");
#endif

    set_init_vars();
    freq(CCXXX1_BASE_FREQUENCY);
    interface_freq(CCXXX1_IF_FREQUENCY);
    assign_channel_spacing(199951);    // 200kHz
    datarate(249939);  // 250 kBaud
    set_rf_settings();
    init();

    return selfTest();
}


// Determines if the communication link is fully connected with a second device
bool CC1101::isConnected(void)
{
    // [] - 1 - Perform a check to ensure the CC1101 can provide communication with a secondary base station link
    // =================
    // Note: This does not necessarily mean the link must be reliable, this only needs to determine: `Can the perheripial provide communication with a 2nd initialized CC1101?`

    // [] - 2 - Return true/false for indicating a connected communication link
    // =================

    return true;
}


// Soft reset the hardware device
void CC1101::reset(void)
{
    // [X] - 1 - Perform a soft reset for the CC1101 transceiver
    // =================
    strobe(CCXXX1_SRES);
}


// Check if the hardware is physically connected and able to communicate on SPI bus
int32_t CC1101::selfTest(void)
{
    // [X] - 1 - Get the chip's version number and fail if different from what was expected.
    _chip_version = status(CCXXX1_VERSION);

    if (_chip_version != CCXXX1_EXPECTED_VERSION_NUMBER) {

        // [X] - 2 - Send message over serial port if version register is not what was expected
        // =================
        log(LOG_LEVEL::ERROR, "CC1101",
            "FATAL ERROR\r\n"
                "  Wrong version number returned from chip's 'VERSION' register (Addr: 0x%02X)\r\n"
                "\r\n"
                "  Expected: 0x%02X\r\n"
                "  Found:    0x%02X\r\n"
                "\r\n"
                "  Troubleshooting Tips:\r\n"
                "    - Check that the chip is fully connected with no soldering errors\r\n"
                "    - Determine if chip is newer version & update firmware\r\n", CCXXX1_VERSION,
            CCXXX1_EXPECTED_VERSION_NUMBER, _chip_version);

        return -1;  // Negative numbers mean error occurred
    }

    return 0;   // Success
}


// 2nd ighest level of initilization routines behind CC1101::powerUp();
void CC1101::init(void)
{

    power_on_reset();

    // Send all configuration values to the CC1101 registers
    put_rf_settings();

    uint8_t paTable[] = {0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};   // 10dB
    write_reg(CCXXX1_PATABLE, paTable[0]);

    // Flush TX and RX buffers before beginning
    flush_rx();
    flush_tx();

    // Set the initial offset frequency estimate
    log(LOG_LEVEL::INFO, "CC1101", "Configuring frequency offset estimate...");
    write_reg(CCXXX1_FSCTRL0, status(CCXXX1_FREQEST));
    log(LOG_LEVEL::INFO, "CC1101", "Frequency offset estimate configured");

    calibrate();

    rx_mode();
}


// Hard reset the device
void CC1101::power_on_reset(void)
{
    log(LOG_LEVEL::INFO, "CC1101", "Beginning Power-on-Reset routine...");

    delete _spi;

    // make sure chip is not selected
    *_cs = 1;

    DigitalOut *SI = new DigitalOut(_mosi_pin);
    DigitalOut *SCK = new DigitalOut(_sck_pin);
    DigitalIn *SO = new DigitalIn(_miso_pin);

    // bring SPI lines to a defined state. Reasons are outlined in CC1101 datasheet - section 11.3
    *SI = 0;
    *SCK = 1;

    // toggle chip select and remain in high state afterwards
    toggle_cs();
    toggle_cs();

    // wait at least 40us
    wait_us(45);

    // pull CSn low & wait for the serial out line to go low
    *_cs = 0;
    while(*SO);

    // cleanup everything before the mbed's SPI library calls take back over
    delete SI;
    delete SO;
    delete SCK;

    // reestablish the SPI bus and call the reset strobe
    setup_spi();
    reset();

    delete _spi;
    // wait for the SO line to go low again. Once low, reset is complete and CC1101 is in IDLE state
    DigitalIn *SO2 = new DigitalIn(_miso_pin);
    while(*SO2);

    // make sure chip is deselected before returning
    *_cs = 1;

    // reestablish the SPI bus for the final time after removing the DigitalIn object
    delete SO2;
    setup_spi();

    log(LOG_LEVEL::INFO, "CC1101", "CC1101 Power-on-Reset complete");
}

// Write to the CC1101's buffer and tell it to send the data over the air
int32_t CC1101::sendData(uint8_t *buf, uint8_t size)
{
    // [X] - 1 - Move all values down by 1 to make room for the packet's size value.
    // =================
    for (int i = size; i > 0; i--)
        buf[i] = buf[i-1];

    // [X] - 2 - Place the packet's size as the array's first value.
    // =================
    buf[0] = size;

    log(LOG_LEVEL::INFO, "CC1101", "PACKET TRANSMITTED\r\n  Bytes: %u", size);

    // [X] - 3 - Send the data to the CC1101. Increment the size value by 1 before doing so to account for the buffer's inserted value
    // =================
    write_reg(CCXXX1_TXFIFO, buf, ++size);

#if CCXXX1_DEBUG_MODE > 1
    Timer t1;
#endif

    // [X] - 4 - Enter the TX state.
    // =================
    strobe(CCXXX1_STX);

#if CCXXX1_DEBUG_MODE > 1
    /*  For the debug mode, this will determine how long the CC1101 takes to transmit everything in the TX buffer
        and enter or return to the RX state.
    */
    t1.start();
    float ti = t1.read();
#endif

    // [X] - 5 - Wait until radio enters back to the RX state.
    // =================
    // *Note: Takes very few cycles, so might as well wait before returning to elimate querky errors.
    //while(mode() != 0x0D);
#if CCXXX1_DEBUG_MODE > 1
    t1.stop();
    log(INF1, "CC1101", "Time:  %02.4f ms", (t1.read() - ti)*1000);
#endif

    // [] - 6 - Return any error codes if necessary.
    // =================
    return 0;   // success
}


// Read any received data from the CC1101
int32_t CC1101::getData(uint8_t *buf, uint8_t *length)
{
    // [X] - 1 - Update the frequency offset estimate.
    // =================
    write_reg(CCXXX1_FSCTRL0, status(CCXXX1_FREQEST));

    // [X] - 2 - Get the packet's size.
    // =================
    uint8_t rx_size;
    read_reg(CCXXX1_RXFIFO, &rx_size, 1);

    // [X] - 3 - Check if there are indeed bytes to be read.
    // =================
    if (rx_size & CCXXX1_RXFIFO_MASK) {

        // [X] - 3.1 - Check if the received data will fit in the provided buffer - fill the buffer if so.
        // =================
        if (rx_size <= *length) {

            // [X] - 3.1a - Read in received data from the CC1101 and put the contents in the provided buffer.
            *length = rx_size;  // set the correct length
            read_reg(CCXXX1_RXFIFO, buf, *length);

            /*  The RSSI value takes a bit to be determined by the CC1101, so having 2 separate SPI reads gives
                the CC1101 enough time to determine the correct value.
            */

            // [X] - 3.1b - Read the 2 appended status bytes.
            uint8_t status_bytes[2];    // status[0] = RSSI. status[1] = LQI.
            read_reg(CCXXX1_RXFIFO, status_bytes, 2);

            // Update the RSSI reading.
            rssi(status_bytes[0]);
            _lqi = status_bytes[1] & CCXXX1_RXFIFO_MASK; // MSB of LQI is the CRC_OK bit - The interrupt is only triggered if CRC is OK, so no need to check again.

#if CCXXX1_DEBUG_MODE > 0
            log(INF1, "CC1101", 
                "PACKET RECEIVED\r\n"
                "  Bytes: %u\r\n"
                "  RSSI: %ddBm\r\n"
                "  LQI:  %u"
                , *length, _rssi, _lqi
            );
#endif

            // [X] - 3.2c - Go back to the receiving state since CC1101 is configured for transitioning to IDLE on receiving a packet.
            rx_mode();
            return 0;   // success
        } else {
            *length = rx_size;
            flush_rx();
            return -2;  // passed buffer size is not large enough
        }
    }

    flush_rx();
    return -1;   // there is no new data to read
}


// Read a single register
uint8_t CC1101::read_reg(uint8_t addr)
{
    _spi->frequency(8000000);
    toggle_cs();
    _spi->write(addr | CCXXX1_READ_SINGLE);
    uint8_t x = _spi->write(0);
    toggle_cs();

#if CCXXX1_DEBUG_MODE > 1
    log(INF2, "CC1101", "== Single Register Read ==\r\n    Address: 0x%02X\r\n    Value:   0x%02X", addr, x);
#endif
    return x;
}


// Read register using BURST
void CC1101::read_reg(uint8_t addr, uint8_t *buffer, uint8_t count)
{
    _spi->frequency(5000000);
    toggle_cs();
    _spi->write(addr | CCXXX1_READ_BURST);
    for (uint8_t i = 0; i < count; i++) {
        buffer[i] = _spi->write(0);
    }
    toggle_cs();

#if CCXXX1_DEBUG_MODE > 1
    log(INF1, "CC1101", "== Burst Register Read ==\r\n    Address: 0x%02X\r\n    Bytes:   %u", addr, count);
#endif
}


// Write a single register
void CC1101::write_reg(uint8_t addr, uint8_t value)
{
    _spi->frequency(8000000);
    toggle_cs();
    _spi->write(addr);
    _spi->write(value);
    toggle_cs();

#if CCXXX1_DEBUG_MODE > 1
    log(INF2, "CC1101", "== Single Register Write ==\r\n    Address: 0x%02X\r\n    Value:   0x%02X", addr, value);
#endif
}


// Write registers using BURST
void CC1101::write_reg(uint8_t addr, uint8_t *buffer, uint8_t count)
{
    _spi->frequency(5000000);
    toggle_cs();
    _spi->write(addr | CCXXX1_WRITE_BURST);
    for (uint8_t i = 0; i < count; i++) {
        _spi->write(buffer[i]);
    }
    toggle_cs();

#if CCXXX1_DEBUG_MODE > 1
    log(INF2, "CC1101", "== Burst Register Write ==\r\n    Address: 0x%02X\r\n    Bytes:   %u", addr, count);
#endif
}


// Send a strobe command to the CC1101 (1 byte)
uint8_t CC1101::strobe(uint8_t addr)
{
    _spi->frequency(8000000);
    toggle_cs();
    uint8_t x = _spi->write(addr);
    toggle_cs();
    return x;
}


// Calibrate the frequency synthesizer - This takes a long time
void CC1101::calibrate(void)
{
#if DEBUG_MODE > 0
    log(INF1, "CC1101", "Calibrating frequency synthesizer");
#endif

    idle();

    // Send the calibration strobe
    strobe(CCXXX1_SCAL);

    // Wait for the radio to leave the calibration step
    while( (mode() == 0x04) | (mode() == 0x05) );

    // The radio is now is IDLE mode, so go to RX mode
    rx_mode();

#if DEBUG_MODE > 0
    log(INF1, "CC1101", "Frequency synthesizer calibrated");
#endif
}


// Set an address for packet filtering
void CC1101::address(uint8_t addr)
{
    _address = addr;
    // NOW, WRITE THE ADDRESS TO THE CC1101
}


// Read the "Link Quality Indicator" of the last received packet
uint8_t CC1101::lqi(void)
{
    return 0x3F - (_lqi & 0x3F);
}


// Returns the CC1101's VERSION register that specifices what exact chip version is being used
uint8_t CC1101::version(void)
{
    return _chip_version;
}


// Update the radio channel that communication occurs on
void CC1101::channel(uint16_t chan)
{
    if ( chan != _channel ) {
#if DEBUG_MODE > 0
        log(INF1, "CC1101", "Updating channel from %02u to %02u", _channel, chan);
#endif

        _channel = chan;
        write_reg(CCXXX1_CHANNR, _channel);

#if DEBUG_MODE > 0
        log(INF1, "CC1101", "Channel updated: %02u", _channel);
#endif
    }
}


// Get the "Received Signal Strength Indicator" of the last received packet
int16_t CC1101::rssi(void)
{
    return _rssi;
}


// Calculate the real RSSI value from the passed register value
void CC1101::rssi(uint8_t rssi_val)
{
    int8_t temp;

    if (rssi_val & 0x80) {
        temp = (rssi_val - 256)>>1;
    } else {
        temp = rssi_val>>1; // divide by 2
    }
    _rssi = temp - 74;
}   // rssi


// Clear the RX buffer
void CC1101::flush_rx(void)
{
#if DEBUG_MODE > 0
    log(INF1, "CC1101", "Clearing RX buffer...");
#endif

    // Make sure that the radio is in IDLE state before flushing the FIFO
    idle();

    // Flush RX FIFO
    strobe(CCXXX1_SFRX);

    // Enter back into a RX state
    rx_mode();

#if DEBUG_MODE > 0
    log(INF1, "CC1101", "RX buffer cleared");
#endif
}


// CLear the TX buffer
void CC1101::flush_tx(void)
{
#if DEBUG_MODE > 0
    log(INF1, "CC1101", "Clearing TX buffer...");
#endif

    // Make sure that the radio is in IDLE state before flushing the FIFO
    idle();

    // Flush TX FIFO
    strobe(CCXXX1_SFTX);

    // Enter back into a RX state
    rx_mode();

#if DEBUG_MODE > 0
    log(INF1, "CC1101", "TX buffer cleared");
#endif
}


// Set the CC1101 into RX mode
void CC1101::rx_mode(void)
{
#if DEBUG_MODE > 0
    log(INF1, "CC1101", "Sending RX_MODE strobe to CC1101");
#endif

    strobe(CCXXX1_SRX);

#if DEBUG_MODE > 0
    log(INF1, "CC1101", "RX_MODE strobe sent to CC1101");
#endif
}


// Set the CC1101 into TX mode
void CC1101::tx_mode(void)
{
#if DEBUG_MODE > 0
    log(INF1, "CC1101", "Sending TX_MODE strobe to CC1101");
#endif

    strobe(CCXXX1_STX);

#if DEBUG_MODE > 0
    log(INF1, "CC1101", "TX_MODE strobe sent to CC1101");
#endif
}


// Set the CC1101 into IDLE mode
void CC1101::idle(void)
{
#if DEBUG_MODE > 0
    log(INF1, "CC1101", "Sending IDLE strobe to CC1101");
#endif

    strobe(CCXXX1_SIDLE);

#if DEBUG_MODE > 0
    log(INF1, "CC1101", "IDLE strobe sent to CC1101");
#endif
}


// Read the STATUS byte from the CC1101
uint8_t CC1101::status(void)
{
    return strobe(CCXXX1_SNOP);
}


// Read from a READ ONLY register
uint8_t CC1101::status(uint8_t addr)
{
    _spi->frequency(8000000);
    toggle_cs();
    _spi->write(addr | CCXXX1_READ_BURST);
    uint8_t x = _spi->write(0);
    toggle_cs();
    return x;
}


// Returns the current mode that the CC1101 is operating in. It is a more detailed version of the STATUS byte
uint8_t CC1101::mode(void)
{
    return status(CCXXX1_MARCSTATE);
}


// Compute the register values for the frequency from a passed value
void CC1101::freq(uint32_t freq)
{
    /* calculate the value that is written to the register for settings the base frequency
     * that the CC1101 should use for sending/receiving over the air. Default value is equivalent
     * to 901.83 MHz.
     */

    // this is split into 3 bytes that are written to 3 different registers on the CC1101
    uint32_t reg_freq = freq/(CCXXX1_CRYSTAL_FREQUENCY>>16);

    rfSettings.FREQ2 = (reg_freq>>16) & 0xFF;   // high byte, bits 7..6 are always 0 for this register
    rfSettings.FREQ1 = (reg_freq>>8) & 0xFF;    // middle byte
    rfSettings.FREQ0 = reg_freq & 0xFF;         // low byte
}


// Compute the register value for the datarate from a passed value
void CC1101::datarate(uint32_t rate)
{
    // update the baud rate class member
    _datarate = rate;

    // have to be careful with bit shifting here since it requires a large amount of shifts
    uint32_t shift_val = 28 - (_modem.data_rate_exp & 0x0F);

    // compute the register value and assign it
    rfSettings.MDMCFG3 = ((_datarate)/(CCXXX1_CRYSTAL_FREQUENCY>>shift_val)) - 256;
}


// Initializes the class members
void CC1101::set_init_vars(void)
{
    // define the initial state of an unselected chip
    *_cs = 1;
    _channel = 1;
    _address = 0x00;

    // turn off address packet filtering and assign 0 (broadcast address) to the address value
    _pck_control.addr_check = ADDR_OFF;

    // these values determine how the CC1101 will handel a packet
    _pck_control.whitening_en = false;

    // enable CRC calculation in TX and CRC checking in RX
    _pck_control.crc_en = true;

    // enable automatically flushing the RX buffer on a bad CRC (only works if 1 packet is in the RX buffer)
    _pck_control.autoflush_en = true;

    // enable appending 2 status bytes to the end of every packet that includes the CRC and
    _pck_control.status_field_en = true;

    // normal packet mode uses RX and TX buffers
    _pck_control.format_type = FORMAT_DEFAULT;

    // setup how the payload of the packet is transmitted - default to a fixed length of 61 bytes
    _pck_control.length_type = PACKET_VARIABLE;
    //_pck_control.length_type = PACKET_FIXED;
    _pck_control.size = 61; // indicates max packet size when using variable packet types

    // this is a preamble threshold for determining when a packet should be accepted
    _pck_control.preamble_thresh = 2;

    // these values determine how the frequency bands and channels are distributed as well as defining the modulation type
    _modem.dc_filter_off_en = false;
    _modem.manchester_encode_en = false;
    _modem.fec_en = false;

    // bandwidth configurations
    _modem.channel_bw = 2;
    _modem.channel_bw_exp = 0;
    _modem.channel_space_exp = 2;
    _modem.data_rate_exp = 13;

    _modem.mod_type = MOD_GFSK;
    _modem.sync_mode = SYNC_HIGH_ALLOW_TWO;
    _modem.preamble_bytes = PREAM_FOUR;
}


// Compute the register values for the interface frequency from a passed value
void CC1101::interface_freq(uint32_t if_freq)
{
    // The desired IF frequency for RX. Subtracted from FS base frequency in RX.
    // bits 7..5 are always 0
    rfSettings.FSCTRL1 = (if_freq/(CCXXX1_CRYSTAL_FREQUENCY>>10)) & 0x1F;
    rfSettings.FSCTRL0 = 0x00;  // set the initial freq calibration to 0
}


// Setup the register values for channel spacing from the class's members
void CC1101::assign_channel_spacing(uint32_t spacing)
{
    // have to be careful with bit shifting here since it requires a large amount of shifts
    uint32_t shift_val = 18 - (_modem.channel_space_exp & 0x03);

    // compute the register value and assign it
    rfSettings.MDMCFG0 = (spacing/(CCXXX1_CRYSTAL_FREQUENCY>>shift_val)) - 256;
}


// Setup the register values for the modem parameters from the class's members
void CC1101::assign_modem_params()
{
    rfSettings.MDMCFG4 = (_modem.channel_bw_exp & 0x03)<<6 | (_modem.channel_bw & 0x03)<<4 | (_modem.data_rate_exp & 0x0F);
    rfSettings.MDMCFG2 = _modem.dc_filter_off_en<<7 | (_modem.mod_type & 0x07)<<4 | _modem.manchester_encode_en<<3 | (_modem.sync_mode & 0x07);
    rfSettings.MDMCFG1 = _modem.fec_en<<7 | (_modem.preamble_bytes & 0x07)<<4 | (_modem.channel_space_exp & 0x03);
}


// Setup the register values for packet handling from the class's members
void CC1101::assign_packet_params()
{
    rfSettings.PCKCTRL0 = _pck_control.whitening_en<<6 | (_pck_control.format_type & 0x3)<<4 | _pck_control.crc_en<<2 | (_pck_control.length_type & 0x3);
    rfSettings.PCKCTRL1 = (_pck_control.preamble_thresh & 0x07)<<5 | _pck_control.autoflush_en<<3 | _pck_control.status_field_en<<2 | (_pck_control.addr_check & 0x03);
    rfSettings.PCKLEN = _pck_control.size;
}


// Setup all register values from the class's members
void CC1101::set_rf_settings(void)
{
    // set the fields for packet controls
    assign_packet_params();

    // set the fields for the frequency limits of the modem
    assign_modem_params();

    // assign an address to the CC1101. This can be used to filter packets
    rfSettings.ADDR = _address;

    // there can be 16 different channel numbers. The bandwidth and spacing are defined in other registers
    rfSettings.CHANNR = _channel;

    // disable GDO0
    rfSettings.IOCFG0 = 0x2E;

    // setup for SPI
    rfSettings.IOCFG1 = 0x0C;

    // setup for going HIGH when packet received and CRC is ok
    rfSettings.IOCFG2 = 0x07;

    rfSettings.DEVIATN = 0x62;

    rfSettings.FREND1 = 0xB6;
    rfSettings.FREND0 = 0x10;


    bool RX_TIME_RSSI = false;
    bool RX_TIME_QUAL = false;
    uint8_t RX_TIME = 0x07;    // no timeout
    rfSettings.MCSM2 = (RX_TIME_RSSI<<4) | (RX_TIME_QUAL<<3) | (RX_TIME & 0x07);


    uint8_t CCA_MODE = 0x00;
    uint8_t RXOFF_MODE = 0x00;  // go directly to IDLE when existing RX
    //uint8_t RXOFF_MODE = 0x03;  // stay in RX when existing RX
    uint8_t TXOFF_MODE = 0x03;  // go directly to RX when existing TX
    // uint8_t TXOFF_MODE = 0x00;  // go directly to IDLE when existing TX
    rfSettings.MCSM1 = ((CCA_MODE & 0x03)<<4) | ((RXOFF_MODE & 0x03)<<2) | (TXOFF_MODE & 0x03);


    uint8_t FS_AUTOCAL = 0x01;  // calibrate when going from IDLE to RX or TX
    uint8_t PO_TIMEOUT = 0x02;
    bool PIN_CTRL_EN = false;
    bool XOSC_FORCE_ON = false;
    rfSettings.MCSM0 = ((FS_AUTOCAL & 0x03)<<4) | ((PO_TIMEOUT & 0x03)<<2) | (PIN_CTRL_EN<<1) | (XOSC_FORCE_ON);


    bool FOC_BS_CS_GATE = false;
    uint8_t FOC_PRE_K = 0x03;
    bool FOC_POST_K = true;
    uint8_t FOC_LIMIT = 0x01;
    rfSettings.FOCCFG = 0x40 | (FOC_BS_CS_GATE<<5) | ((FOC_PRE_K & 0x03)<<3) | (FOC_POST_K<<2) | (FOC_LIMIT & 0x03);

    rfSettings.BSCFG = 0x1C;

    rfSettings.AGCCTRL2 = 0xC7;
    rfSettings.AGCCTRL1 = 0x00;
    rfSettings.AGCCTRL0 = 0xB0;

    // 33 byte TX FIFO & 32 byte RX FIFO
    rfSettings.FIFOTHR = 0x07;
    // rfSettings.FIFOTHR = 0x0F;
}


// Write the setup register values to the CC1101 for configuration
void CC1101::put_rf_settings()
{
#if DEBUG_MODE > 0
    log(INF1, "CC1101", "Writing configuration registers...");
#endif
    write_reg(CCXXX1_IOCFG2,   rfSettings.IOCFG2);
    write_reg(CCXXX1_IOCFG1,   rfSettings.IOCFG1);
    write_reg(CCXXX1_IOCFG0,   rfSettings.IOCFG0);
    write_reg(CCXXX1_FIFOTHR,  rfSettings.FIFOTHR);
    // SYNC1
    // SYNC0
    write_reg(CCXXX1_PCKLEN,   rfSettings.PCKLEN);
    write_reg(CCXXX1_PCKCTRL1, rfSettings.PCKCTRL1);
    write_reg(CCXXX1_PCKCTRL0, rfSettings.PCKCTRL0);
    write_reg(CCXXX1_ADDR,     rfSettings.ADDR);

    write_reg(CCXXX1_CHANNR,   rfSettings.CHANNR);
    write_reg(CCXXX1_FSCTRL1,  rfSettings.FSCTRL1);
//    write_reg(CCXXX1_FSCTRL0,  rfSettings.FSCTRL0);
    write_reg(CCXXX1_FREQ2,    rfSettings.FREQ2);

    write_reg(CCXXX1_FREQ1,    rfSettings.FREQ1);
    write_reg(CCXXX1_FREQ0,    rfSettings.FREQ0);
    write_reg(CCXXX1_MDMCFG4,  rfSettings.MDMCFG4);
    write_reg(CCXXX1_MDMCFG3,  rfSettings.MDMCFG3);

    write_reg(CCXXX1_MDMCFG2,  rfSettings.MDMCFG2);
    write_reg(CCXXX1_MDMCFG1,  rfSettings.MDMCFG1);
    write_reg(CCXXX1_MDMCFG0,  rfSettings.MDMCFG0);
    write_reg(CCXXX1_DEVIATN,  rfSettings.DEVIATN);
    write_reg(CCXXX1_MCSM2 ,   rfSettings.MCSM2);
    write_reg(CCXXX1_MCSM1 ,   rfSettings.MCSM1);
    write_reg(CCXXX1_MCSM0 ,   rfSettings.MCSM0 );
    write_reg(CCXXX1_FOCCFG,   rfSettings.FOCCFG);
    write_reg(CCXXX1_BSCFG,    rfSettings.BSCFG);
    write_reg(CCXXX1_AGCCTRL2, rfSettings.AGCCTRL2);
    write_reg(CCXXX1_AGCCTRL1, rfSettings.AGCCTRL1);
    write_reg(CCXXX1_AGCCTRL0, rfSettings.AGCCTRL0);
    // WOREVT1
    // WOREVT0
    // WORCTRL
    write_reg(CCXXX1_FREND1,   rfSettings.FREND1);
    write_reg(CCXXX1_FREND0,   rfSettings.FREND0);
    //write_reg(CCXXX1_FSCAL3,   rfSettings.FSCAL3);
    //write_reg(CCXXX1_FSCAL2,   rfSettings.FSCAL2);
    //write_reg(CCXXX1_FSCAL1,   rfSettings.FSCAL1);
    //write_reg(CCXXX1_FSCAL0,   rfSettings.FSCAL0);
    // PCCTRL1
    // PCCTRL0
    // FSTEST
    // PTEST
    // AGCTEST
    //write_reg(CCXXX1_TEST2,    rfSettings.TEST2);
    //write_reg(CCXXX1_TEST1,    rfSettings.TEST1);
    //write_reg(CCXXX1_TEST0,    rfSettings.TEST0);
#if DEBUG_MODE > 0
    log(INF1, "CC1101", "Configurations registers ready");
#endif
}
