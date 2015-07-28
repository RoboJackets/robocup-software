#include "CC1201.hpp"

#define S1(x) #x
#define S2(x) S1(x)
#define LINE_INFO __FILE__ ":" S2(__LINE__)


// extern "C" void mbed_reset();


using namespace std;


CC1201::CC1201() : CommLink() {}


CC1201::CC1201(PinName mosi, PinName miso, PinName sck, PinName cs, PinName intPin) :
	CommLink(mosi, miso, sck, cs, intPin)
{
	//powerOnReset();res
	_offset_reg_written = false;
	reset();
	idle(); flush_rx(); flush_tx();
	selfTest();

	if (_isInit == true) {
		CommLink::ready();
		log(OK, "CC1201", "Ready!");
	}
}


CC1201::~CC1201()
{
	if (_spi)
		delete _spi;

	if (_cs)
		delete _cs;

	if (_int_in)
		delete _int_in;
}


/**
 *
 */
int32_t CC1201::sendData(uint8_t *buf, uint8_t size)
{
	//idle();
	//strobe(CC1201_STROBE_SFTX);
	if (_isInit == false)
		return 0;

	log(OK, "PACKET LENGTH", "%u", size);

	if ( size != (buf[0] + 1) ) {
		log(SEVERE, "CC1201 TX", "Packet size values are inconsistent. %u bytes requested vs %u bytes in packet.", size, buf[0]);
		return 1;
	}


	// [X] - 1 - Send the data to the CC1201.
	// =================
	uint8_t device_state = writeReg(CC1201_TX_FIFO, buf, size);


	// [X] - 4 - Enter the TX state.
	// =================
	if ( (device_state & CC1201_TX_FIFO_ERROR) == CC1201_TX_FIFO_ERROR ) {
		log(WARN, "CC1201 FIFO", "STATE AT TX ERROR: 0x%02X", device_state);
		flush_tx();	// flush the TX buffer & return if the FIFO is in a corrupt state

		// set in IDLE mode and strobe back into RX to ensure the states will fall through calibration then return
		idle();
		strobe(CC1201_STROBE_SRX);

		return 2;
	} else {
		strobe(CC1201_STROBE_STX);	// Enter TX mode
	}


	// [X] - 5 - Wait until radio enters into the TX state
	// =================
	while ( mode() != 0x13 );	// While not TX mode

	uint8_t bts = 1;

	do {
		bts = readReg(CC1201EXT_NUM_TXBYTES, EXT_FLAG_ON);
	} while (bts != 0);

	return 0;   // success
}

int32_t CC1201::getData(uint8_t *buf, uint8_t *len)
{
	osDelay(1);	//make sure the packet is ready. remove for production

	uint8_t device_state = freq_update();	// update frequency offset estimate & get the current state while at it

	uint8_t num_rx_bytes = readReg(CC1201EXT_NUM_RXBYTES, EXT_FLAG_ON);

	if ( ((*len) + 2) < num_rx_bytes ) {
		log(SEVERE, "CC1201 RX", "%u bytes in RX FIFO with passed buffer size of %u bytes. Unable to process request.", num_rx_bytes, *len);
		return 0x01;
	}

	if ( (device_state & CC1201_RX_FIFO_ERROR) == CC1201_RX_FIFO_ERROR ) {
		flush_rx();	// flush RX FIFO buffer and place back into RX state
		strobe(CC1201_STROBE_SRX);
		return 0x02;
	}

	if ( readReg(CC1201EXT_NUM_TXBYTES, EXT_FLAG_ON) > 0 ) {
		// This was a TX interrupt from the CC1201, not an RX
		return 0x03;
	}

	if ( num_rx_bytes > 0 ) {
		device_state = readReg(CC1201_RX_FIFO, buf, num_rx_bytes);
		*len = num_rx_bytes;

		log(INF2, "CC1201 RX", "Bytes in RX buffer: %u", num_rx_bytes);
		log(INF2, "CC1201 RX", "Payload bytes: %u", buf[0]);
	}

	update_rssi();

	// return back to RX mode
	strobe(CC1201_STROBE_SRX);

	return 1;	// success
}


/**
 * reads a standard register
 */
uint8_t CC1201::readReg(uint8_t addr, bool ext_flag)
{
	uint8_t returnVal;

	if ( (addr == 0x2F) & (ext_flag == EXT_FLAG_OFF) ) {
		log(WARN, LINE_INFO, "readReg invalid address: %02X", addr);
		return 0xFF;
	}

	if (ext_flag == EXT_FLAG_ON)
		return readRegExt(addr);

	addr &= 0xBF; // Should be redundant but leaving for security. We don't want to accidently do a burst read.

	toggle_cs();
	_spi->write(addr | CC1201_READ);
	returnVal = _spi->write(0x00);
	toggle_cs();

	return returnVal;
}
uint8_t CC1201::readReg(uint8_t addr, uint8_t *buffer, uint8_t len, bool ext_flag)
{
	uint8_t status_byte;

	if ( (addr >= 0x2F) & (ext_flag == EXT_FLAG_OFF) ) {
		log(WARN, LINE_INFO, "readReg invalid address: %02X", addr);
		return 0xFF;
	}

	if ( ext_flag == EXT_FLAG_ON )
		return readRegExt(addr, buffer, len);

	toggle_cs();
	status_byte = _spi->write(addr | CC1201_READ | CC1201_BURST);

	for (uint8_t i = 0; i < len; i++)
		buffer[i] = _spi->write(0x00);

	toggle_cs();

	return status_byte;
}


uint8_t CC1201::writeReg(uint8_t addr, uint8_t value, bool ext_flag)
{
	uint8_t status_byte;
	addr &= 0x3F; // Don't accidently do a burst or read

	if (ext_flag == EXT_FLAG_ON)
		return writeRegExt(addr, value);

	toggle_cs();
	status_byte = _spi->write(addr);
	_spi->write(value);
	toggle_cs();

	return status_byte;
}
uint8_t CC1201::writeReg(uint8_t addr, uint8_t *buffer, uint8_t len, bool ext_flag)
{
	uint8_t status_byte;
	addr &= 0x7F; // Don't accidently do a read

	if (ext_flag == EXT_FLAG_ON)
		return writeRegExt(addr, buffer, len);

	toggle_cs();
	status_byte = _spi->write(addr | CC1201_BURST);

	for (uint8_t i = 0; i < len; i++)
		_spi->write(buffer[i]);

	toggle_cs();

	return status_byte;
}


/**
 * reads an extended register
 */
uint8_t CC1201::readRegExt(uint8_t addr)
{
	// Only callable from readReg(), so no checks needed
	uint8_t returnVal;

	toggle_cs();
	_spi->write(CC1201_EXTENDED_ACCESS | CC1201_READ);
	_spi->write(addr);
	returnVal = _spi->write(0x00);
	toggle_cs();

	return returnVal;
}
uint8_t CC1201::readRegExt(uint8_t addr, uint8_t *buffer, uint8_t len)
{
	// Only callable from readReg(), so no checks needed
	uint8_t status_byte;

	toggle_cs();
	status_byte = _spi->write(CC1201_EXTENDED_ACCESS | CC1201_READ | CC1201_BURST);
	_spi->write(addr);

	for (int i = 0; i < len; i++)
		buffer[i] = _spi->write(0x00);

	toggle_cs();

	return status_byte;
}


uint8_t CC1201::writeRegExt(uint8_t addr, uint8_t value)
{
	// Only callable from writeReg(), so no checks needed
	uint8_t status_byte;

	toggle_cs();
	status_byte = _spi->write(CC1201_EXTENDED_ACCESS);
	_spi->write(addr);
	_spi->write(value);
	toggle_cs();

	return status_byte;
}
uint8_t CC1201::writeRegExt(uint8_t addr, uint8_t *buffer, uint8_t len)
{
	// Only callable from writeReg(), so no checks needed
	uint8_t status_byte;

	toggle_cs();
	status_byte = _spi->write(CC1201_EXTENDED_ACCESS | CC1201_BURST);
	_spi->write(addr);

	for (uint8_t i = 0; i < len; i++)
		_spi->write(buffer[i]);

	toggle_cs();

	return status_byte;
}


uint8_t CC1201::strobe(uint8_t addr)
{
	if (addr > 0x3d || addr < 0x30) {
		log(WARN, LINE_INFO, "Invalid address: %02X", addr);
		return -1;
	}

	toggle_cs();
	uint8_t ret = _spi->write(addr);
	toggle_cs();

	return ret;
}

uint8_t CC1201::mode(void)
{
	return 0x1F & readReg(CC1201EXT_MARCSTATE, EXT_FLAG_ON);
}


uint8_t CC1201::status(uint8_t addr)
{
	return strobe(addr);
}


uint8_t CC1201::status(void)
{
	return strobe(CC1201_STROBE_SNOP);
}


void CC1201::reset(void)
{
	idle();
	toggle_cs();
	_spi->write(CC1201_STROBE_SRES);
	//Thread::wait(200);
	toggle_cs();

	_isInit = false;
}

int32_t CC1201::selfTest(void)
{
	if (_isInit)
		return 0;

	_isInit = true;

	_chip_version = readReg(CC1201EXT_PARTNUMBER, EXT_FLAG_ON);

	if (_chip_version != CC1201_EXPECTED_PART_NUMBER) {
		log(FATAL, LINE_INFO,
		    "FATAL ERROR\r\n"
		    "  Wrong version number returned from chip's 'PARTNUMBER' register (Addr: 0x%02X)\r\n"
		    "  Expected: 0x%02X\r\n"
		    "  Found:    0x%02X\r\n"
		    "\r\n"
		    , CC1201EXT_PARTNUMBER, CC1201_EXPECTED_PART_NUMBER, _chip_version);

		return -1;
	}

	idle();

	return 0;
}

bool CC1201::isConnected(void)
{
	return _isInit;
}

uint8_t recurseCount = 0;
void CC1201::powerOnReset(void)
{
	if (_isInit == false)
		return;

	recurseCount++;

	if (recurseCount >= 10) {
		log(SEVERE, LINE_INFO, "cannot calibrate radio -> system reset");
		//Thread::wait(500);
		// mbed_reset();
		mbed_interface_reset();
	}

	log(INF1, LINE_INFO, "Beginning power on reset (POR)");

	log(INF2, LINE_INFO, "Strobe SIDLE");
	idle();
	log(INF2, LINE_INFO, "IDLE strobe OK.");

	log(INF2, LINE_INFO, "Force CS low");
	*_cs = 0;
	log(INF2, LINE_INFO, "Strobe SRES");
	_spi->write(CC1201_STROBE_SRES);
	log(INF2, LINE_INFO, "dealloc SPI");
	delete _spi;
	log(INF2, LINE_INFO, "(MI)SO alloc digIn");
	DigitalIn *SO = new DigitalIn(_miso_pin);


	log(INF2, LINE_INFO, "wait for olliscator assertion");
	uint8_t waitCycles = 20;

	while (*SO) {
		if (waitCycles == 0) {
			log(WARN, LINE_INFO, "calibration settled assertion timeout -> retry");
			//powerOnReset();	// There's absolutely no need to do this. If it doesn't calibrate in 20 cycles, it will never do it successfully.
			return;
		}

		//Thread::wait(100);
		waitCycles--;
	}

	recurseCount = 0;

	log(INF2, LINE_INFO, "dealloc digIn");
	delete SO;
	log(INF2, LINE_INFO, "force CSn high");
	*_cs = 1;
	log(INF2, LINE_INFO, "setup SPI");
	setup_spi();
	log(INF2, LINE_INFO, "POR COMPLETE!");

	_isInit = false;
}


void CC1201::flush_tx(void)
{
	idle();
	strobe(CC1201_STROBE_SFTX);
	log(WARN, "CC1201 RX FIFO", "%u bytes flushed from TX FIFO buffer.", readReg(CC1201EXT_NUM_TXBYTES, EXT_FLAG_ON));
}


void CC1201::flush_rx(void)
{
	idle();
	strobe(CC1201_STROBE_SFRX);
	log(WARN, "CC1201 RX FIFO", "%u bytes flushed from RX FIFO buffer.", readReg(CC1201EXT_NUM_RXBYTES, EXT_FLAG_ON));
}


void CC1201::calibrate(void)
{
	idle();
	strobe(CC1201_STROBE_SCAL);
}


void CC1201::update_rssi(void)
{
	uint8_t offset = 0;

	// Only use the top MSB for simplicity. 1 dBm resolution.
	if (_offset_reg_written) {
		offset = readReg(CC1201EXT_RSSI1, EXT_FLAG_ON);
		_rssi = static_cast<float>((int8_t)twos_compliment(offset));

		log(INF3, "CC1201 RSSI", "RSSI is from device.");
	} else {
		_rssi = 0.0;
	}

	log(INF3, LINE_INFO, "RSSI Register Val: 0x%02X", offset);
}

float CC1201::rssi(void)
{
	return _rssi;
}

uint8_t CC1201::idle(void)
{
	uint8_t status_byte = strobe(CC1201_STROBE_SIDLE);

	if (_isInit == false)
		return 0;

	while ( mode() != 0x01 );

	return status_byte;
}

uint8_t CC1201::rand(void)
{
	writeReg(CC1201EXT_RNDGEN, 0x80, EXT_FLAG_ON);
	return readReg(CC1201EXT_RNDGEN, EXT_FLAG_ON);
}

uint8_t CC1201::freq_update(void)
{
	return strobe(CC1201_STROBE_SAFC);
}

float CC1201::freq(void)
{
	uint8_t buf[5];
	uint16_t freq_offset;
	uint32_t freq_base;
	float freq;

	freq_update();

	readReg(CC1201EXT_FREQOFF1, buf, 5, EXT_FLAG_ON);
	/*
	buf[0] = readReg(CC1201EXT_FREQOFF1, EXT_FLAG_ON);
	buf[1] = readReg(CC1201EXT_FREQOFF0, EXT_FLAG_ON);
	buf[2] = readReg(CC1201EXT_FREQ2, EXT_FLAG_ON);
	buf[3] = readReg(CC1201EXT_FREQ1, EXT_FLAG_ON);
	buf[4] = readReg(CC1201EXT_FREQ0, EXT_FLAG_ON);
	*/

	freq_offset = (buf[0] << 8) | (buf[1]);
	freq_offset = (~freq_offset) + 1;
	freq_base = (buf[2] << 16) | (buf[3] << 8) | (buf[4]);

	freq = 40 * static_cast<float>((freq_base >> 16) + (freq_offset >> 18));
	freq /= 4;

	log(INF1, LINE_INFO, "Operating Frequency: %3.2f MHz", freq);

	return freq;
}

bool CC1201::isLocked(void)
{
	// This is only valid in RX, TX, & FSTXON
	return (readReg(CC1201EXT_FSCAL_CTRL, EXT_FLAG_ON) & 0x01);
}

void CC1201::set_rssi_offset(int8_t offset)
{
	// HAVING THIS MEANS OFFSET CAN ONLY BE SET ONCE FROM THE CLASS
	if (_offset_reg_written == true )
		return;

	_offset_reg_written = true;

	//uint8_t offset_c = offset;
	uint8_t offset_c = twos_compliment(offset);
	writeReg(CC1201_AGC_GAIN_ADJUST, offset_c);
}

uint8_t CC1201::twos_compliment(uint8_t val)
{
	return -(unsigned int)val;
}