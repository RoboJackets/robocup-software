#include "CC1201.hpp"

#include "CC1201Defines.hpp"
#include "logger.hpp"


CC1201::CC1201(PinName mosi, PinName miso, PinName sck, PinName cs, PinName intPin, int rssiOffset) :
	CommLink(mosi, miso, sck, cs, intPin), _isInit(false)
{
	_offset_reg_written = false;
	reset();
	set_rssi_offset(rssiOffset);
	selfTest();

	if (_isInit == true) {
		LOG(INIT, "CC1201 ready!");
		CommLink::ready();
	}
}


int32_t CC1201::sendData(const uint8_t* buf, uint8_t size)
{
	// Return if there's no functional radio transceiver - the system will lockup otherwise
	if (_isInit == false)
		return -1;

	if ( size != (buf[0] + 1) ) {
		LOG(SEVERE, "Packet size values are inconsistent. %u bytes requested vs %u bytes in packet.", size, buf[0]);
		return 1;
	}

	strobe(CC1201_STROBE_SFTX);

	// [X] - 1 - Send the data to the CC1201.
	// =================
	uint8_t device_state = writeReg(CC1201_TX_FIFO, buf, size);

	// [X] - 4 - Enter the TX state.
	// =================
	if ( (device_state & CC1201_TX_FIFO_ERROR) == CC1201_TX_FIFO_ERROR ) {
		LOG(WARN, "STATE AT TX ERROR: 0x%02X", device_state);
		flush_tx();	// flush the TX buffer & return if the FIFO is in a corrupt state

		// set in IDLE mode and strobe back into RX to ensure the states will fall through calibration then return
		idle();
		strobe(CC1201_STROBE_SRX);

		return 2;
	} else {
		strobe(CC1201_STROBE_STX);	// Enter TX mode
	}

	// [X] - 5 - Wait until radio's TX buffer is emptied
	uint8_t bts = 1;

	do {
		bts = readReg(CC1201EXT_NUM_TXBYTES, EXT_FLAG_ON);
		Thread::wait(2);

	} while (bts != 0);

	return 0;   // success
}


int32_t CC1201::getData(uint8_t* buf, uint8_t* len)
{
	uint8_t device_state = freqUpdate();	// update frequency offset estimate & get the current state while at it
	uint8_t num_rx_bytes = readReg(CC1201EXT_NUM_RXBYTES, EXT_FLAG_ON);

	if ( ((*len) + 2) < num_rx_bytes ) {
		LOG(SEVERE, "%u bytes in RX FIFO with passed buffer size of %u bytes.", num_rx_bytes, *len);

		return COMM_FUNC_BUF_ERR;
	}

	if ( (device_state & CC1201_RX_FIFO_ERROR) == CC1201_RX_FIFO_ERROR ) {
		flush_rx();	// flush RX FIFO buffer and place back into RX state
		strobe(CC1201_STROBE_SRX);

		return COMM_DEV_BUF_ERR;
	}

	// This is temporary
	if ( readReg(CC1201EXT_NUM_TXBYTES, EXT_FLAG_ON) > 0 ) {
		// This was a TX interrupt from the CC1201, not an RX
		return COMM_FALSE_TRIG;
	}

	if ( num_rx_bytes > 0 ) {
		device_state = readReg(CC1201_RX_FIFO, buf, num_rx_bytes);
		*len = num_rx_bytes;

		LOG(INF3, "Bytes in RX buffer: %u\r\nPayload bytes: %u", num_rx_bytes, buf[0]);
	} else {
		return COMM_NO_DATA;
	}

	update_rssi();

	// return back to RX mode
	strobe(CC1201_STROBE_SRX);

	return COMM_SUCCESS;	// success
}


/**
 * reads a standard register
 */
uint8_t CC1201::readReg(uint8_t addr, ext_flag_t ext_flag)
{
	uint8_t returnVal;

	if ( (addr == 0x2F) & (ext_flag == EXT_FLAG_OFF) ) {
		LOG(WARN, "Invalid address: %02X", addr);
		return 0xFF;
	}

	if (ext_flag == EXT_FLAG_ON)
		return readRegExt(addr);

	addr &= 0xBF; // Should be redundant but leaving for security. We don't want to accidently do a burst read.

	chip_select();
	_spi.write(addr | CC1201_READ);
	returnVal = _spi.write(0x00);
	chip_deselect();

	return returnVal;
}
uint8_t CC1201::readReg(uint8_t addr, uint8_t* buffer, uint8_t len, ext_flag_t ext_flag)
{
	uint8_t status_byte;

	if ( (addr >= 0x2F) & (ext_flag == EXT_FLAG_OFF) ) {
		LOG(WARN, "Invalid address: %02X", addr);
		return 0xFF;
	}

	if ( ext_flag == EXT_FLAG_ON )
		return readRegExt(addr, buffer, len);

	chip_select();
	status_byte = _spi.write(addr | CC1201_READ | CC1201_BURST);

	for (uint8_t i = 0; i < len; i++)
		buffer[i] = _spi.write(0x00);

	chip_deselect();

	return status_byte;
}


uint8_t CC1201::writeReg(uint8_t addr, uint8_t value, ext_flag_t ext_flag)
{
	uint8_t status_byte;
	addr &= 0x3F; // Don't accidently do a burst or read

	if (ext_flag == EXT_FLAG_ON)
		return writeRegExt(addr, value);

	chip_select();
	status_byte = _spi.write(addr);
	_spi.write(value);
	chip_deselect();

	return status_byte;
}
uint8_t CC1201::writeReg(uint8_t addr, const uint8_t* buffer, uint8_t len, ext_flag_t ext_flag)
{
	uint8_t status_byte;
	addr &= 0x7F; // Don't accidently do a read

	if (ext_flag == EXT_FLAG_ON)
		return writeRegExt(addr, buffer, len);

	chip_select();
	status_byte = _spi.write(addr | CC1201_BURST);

	for (uint8_t i = 0; i < len; i++)
		_spi.write(buffer[i]);

	chip_deselect();

	return status_byte;
}


/**
 * reads an extended register
 */
uint8_t CC1201::readRegExt(uint8_t addr)
{
	// Only callable from readReg(), so no checks needed
	uint8_t returnVal;

	chip_select();
	_spi.write(CC1201_EXTENDED_ACCESS | CC1201_READ);
	_spi.write(addr);
	returnVal = _spi.write(0x00);
	chip_deselect();

	return returnVal;
}
uint8_t CC1201::readRegExt(uint8_t addr, uint8_t* buffer, uint8_t len)
{
	// Only callable from readReg(), so no checks needed
	uint8_t status_byte;

	chip_select();
	status_byte = _spi.write(CC1201_EXTENDED_ACCESS | CC1201_READ | CC1201_BURST);
	_spi.write(addr);

	for (int i = 0; i < len; i++)
		buffer[i] = _spi.write(0x00);

	chip_deselect();

	return status_byte;
}


uint8_t CC1201::writeRegExt(uint8_t addr, uint8_t value)
{
	// Only callable from writeReg(), so no checks needed
	uint8_t status_byte;

	chip_select();
	status_byte = _spi.write(CC1201_EXTENDED_ACCESS);
	_spi.write(addr);
	_spi.write(value);
	chip_deselect();

	return status_byte;
}
uint8_t CC1201::writeRegExt(uint8_t addr, const uint8_t* buffer, uint8_t len)
{
	// Only callable from writeReg(), so no checks needed
	uint8_t status_byte;

	chip_select();
	status_byte = _spi.write(CC1201_EXTENDED_ACCESS | CC1201_BURST);
	_spi.write(addr);

	for (uint8_t i = 0; i < len; i++)
		_spi.write(buffer[i]);

	chip_deselect();

	return status_byte;
}


uint8_t CC1201::strobe(uint8_t addr)
{
	if (addr > 0x3d || addr < 0x30) {
		LOG(WARN, "Invalid address: %02X", addr);
		return -1;
	}

	chip_select();
	uint8_t ret = _spi.write(addr);
	chip_deselect();

	return ret;
}

uint8_t CC1201::mode()
{
	return 0x1F & readReg(CC1201EXT_MARCSTATE, EXT_FLAG_ON);
}


uint8_t CC1201::status(uint8_t addr)
{
	return strobe(addr);
}


uint8_t CC1201::status()
{
	return strobe(CC1201_STROBE_SNOP);
}


void CC1201::reset()
{
	idle();
	chip_select();
	_spi.write(CC1201_STROBE_SRES);
	chip_deselect();

	// Wait up to 300ms for the radio to do anything. Don't block everything else if it doesn't startup correctly
	for (int i = 0; i < 300; i++) {
		if (~(idle()) & 0x80)	// Chip is ready when status byte's MSB is 0
			break;
		else
			Thread::wait(1);
	}

	_isInit = false;
}

int32_t CC1201::selfTest()
{
	if (_isInit == true)
		return 0;

	_chip_version = readReg(CC1201EXT_PARTNUMBER, EXT_FLAG_ON);

	if (_chip_version != CC1201_EXPECTED_PART_NUMBER) {
		LOG(FATAL,
		    "CC1201 part number error:\r\n"
		    "    Found:\t0x%02X (expected 0x%02X)",
		    _chip_version,
		    CC1201_EXPECTED_PART_NUMBER
		   );

		return -1;
	} else {

		_isInit = true;
		idle();
		return 0;
	}
}

bool CC1201::isConnected()
{
	return _isInit;
}

void CC1201::powerOnReset()
{
	if (_isInit == false)
		return;

	LOG(INF1, "Beginning power on reset (POR)");
	LOG(INF2, "Strobe SIDLE");

	idle();

	LOG(INF2, "IDLE strobe OK.");

	LOG(INF2, "Force CS low");
	chip_select();
	LOG(INF2, "Strobe SRES");
	_spi.write(CC1201_STROBE_SRES);
	LOG(INF2, "dealloc SPI");
	// delete _spi;
	LOG(INF2, "(MI)SO alloc digIn");
	DigitalIn* SO = new DigitalIn(_miso_pin);


	LOG(INF2, "wait for olliscator assertion");
	uint8_t waitCycles = 20;

	while (*SO) {
		if (waitCycles == 0) {
			LOG(WARN, "calibration settled assertion timeout -> retry");
			return;
		}

		Thread::wait(5);
		waitCycles--;
	}

	LOG(INF2, "dealloc digIn");
	delete SO;
	LOG(INF2, "force CSn high");
	chip_deselect();
	LOG(INF2, "setup SPI");
	setup_spi();
	LOG(INF2, "POR COMPLETE!");

	_isInit = false;
}


void CC1201::flush_tx()
{
	idle();
	strobe(CC1201_STROBE_SFTX);
	LOG(WARN, "%u bytes flushed from TX FIFO buffer.", readReg(CC1201EXT_NUM_TXBYTES, EXT_FLAG_ON));
}


void CC1201::flush_rx()
{
	idle();
	strobe(CC1201_STROBE_SFRX);
	LOG(WARN, "%u bytes flushed from RX FIFO buffer.", readReg(CC1201EXT_NUM_RXBYTES, EXT_FLAG_ON));
}


void CC1201::calibrate()
{
	idle();
	strobe(CC1201_STROBE_SCAL);
}


void CC1201::update_rssi()
{
	uint8_t offset = 0;

	// Only use the top MSB for simplicity. 1 dBm resolution.
	if (_offset_reg_written) {
		offset = readReg(CC1201EXT_RSSI1, EXT_FLAG_ON);
		_rssi = static_cast<float>((int8_t)twos_compliment(offset));

		LOG(INF3, "RSSI is from device.");
	} else {
		_rssi = 0.0;
	}

	LOG(INF3, "RSSI Register Val: 0x%02X", offset);
}

float CC1201::rssi()
{
	return _rssi;
}

uint8_t CC1201::idle()
{
	uint8_t status_byte = strobe(CC1201_STROBE_SIDLE);

	if (_isInit == false)
		return status_byte;

	while ( mode() != 0x01 );

	return status_byte;
}

uint8_t CC1201::rand()
{
	writeReg(CC1201EXT_RNDGEN, 0x80, EXT_FLAG_ON);
	return readReg(CC1201EXT_RNDGEN, EXT_FLAG_ON);
}

uint8_t CC1201::freqUpdate()
{
	return strobe(CC1201_STROBE_SAFC);
}

float CC1201::freq()
{
	uint8_t buf[5];
	uint16_t freq_offset;
	uint32_t freq_base;
	float freq;

	freqUpdate();

	readReg(CC1201EXT_FREQOFF1, buf, 5, EXT_FLAG_ON);

	freq_offset = (buf[0] << 8) | (buf[1]);
	freq_offset = (~freq_offset) + 1;
	freq_base = (buf[2] << 16) | (buf[3] << 8) | (buf[4]);

	freq = 40 * static_cast<float>((freq_base >> 16) + (freq_offset >> 18));
	freq /= 4;

	LOG(INF2, "Operating Frequency: %3.2f MHz", freq);

	return freq;
}

bool CC1201::isLocked()
{
	// This is only valid in RX, TX, & FSTXON
	return (readReg(CC1201EXT_FSCAL_CTRL, EXT_FLAG_ON) & 0x01);
}

void CC1201::set_rssi_offset(int8_t offset)
{
	// HAVING THIS MEANS OFFSET MUST ONLY BE SET ONCE FOR THE CLASS
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
