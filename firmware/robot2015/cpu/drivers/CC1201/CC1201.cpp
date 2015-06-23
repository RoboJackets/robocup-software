#include "CC1201.hpp"

#define S1(x) #x
#define S2(x) S1(x)
#define LINE_INFO __FILE__ ":" S2(__LINE__)


extern "C" void mbed_reset();


using namespace std;


CC1201::CC1201() : CommLink() {}


CC1201::CC1201(PinName mosi, PinName miso, PinName sck, PinName cs, PinName intPin) :
CommLink(mosi, miso, sck, cs, intPin)
{
    //powerOnReset();
	reset();
	idle();
	flush_rx(); flush_tx();
	//Thread::wait(100);
	selfTest();
	CommLink::ready();
	log(OK, "CC1201", "Ready!");
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
 int32_t CC1201::sendData(uint8_t* buffer, uint8_t size)
 {
 	freq_update();
 	idle();
 	strobe(CC1201_STROBE_SFTX);
	// [X] - 1 - Move all values down by 1 to make room for the packet's size value.
	// =================
	//for (int i = size; i > 0; i--)
		//buffer[i] = buffer[i - 1];


	// [X] - 2 - Place the packet's size as the array's first value. Increment afterwards.
	// =================
	//buffer[0] = size++;


	// [X] - 3 - Send the data to the CC1101. Increment the size value by 1 
	//before doing so to account for the buffer's inserted value
	// =================
 	uint8_t device_state = writeReg(CC1201_TX_FIFO, buffer, size);

 	log(INF2, "CC1201 TX", "Bytes in TX buffer: %u", readReg(CC1201EXT_NUM_TXBYTES, EXT_FLAG_ON));
 	log(INF2, "CC1201 TX", "Payload bytes: %u", buffer[0]);


	// [X] - 4 - Enter the TX state.
	// =================
 	if( (device_state & CC1201_TX_FIFO_ERROR) == CC1201_TX_FIFO_ERROR )
 	{
 		log(WARN, "CC1201 FIFO", "STATE AT TX ERROR: 0x%02X", device_state);
		flush_tx();	// flush the TX buffer & return if the FIFO is in a corrupt state
		return 1;
	} else {
		strobe(CC1201_STROBE_STX);	// Enter TX mode
	}

	
	//while(mode() != 0x0D);
	//log(INF1, LINE_INFO, "MARCSTATE after TX: %02X", mode());

	//uint16_t freq_offset_est = readReg(CC1201EXT_FREQOFF1, EXT_FLAG_ON) << 8;
	//freq_offset_est |= readReg(CC1201EXT_FREQOFF0, EXT_FLAG_ON);

	//uint32_t freq_offset = freq_offset_est * 40000000;
	//freq_offset = freq_offset >> 20;
	
	//log(INF1, LINE_INFO, "Est. Freq. Offset:\t%u Hz", freq_offset);


	// [X] - 5 - Wait until radio enters into the TX state
	// =================
	while( mode() != 0x13 );	// While not TX mode

	uint8_t bts = 1;
	do {
		bts = readReg(CC1201EXT_NUM_TXBYTES, EXT_FLAG_ON);
	}
	while (bts != 0);

	strobe(CC1201_STROBE_SRX);

	return 0;   // success
}

int32_t CC1201::getData(uint8_t* buf, uint8_t* len)
{
	/*
	log(INF3, LINE_INFO, "RXFIRST: %02X, RXLAST: %02X",
			readReg(CC1201EXT_RXFIRST, EXT_FLAG_ON),
			readReg(CC1201EXT_RXLAST, EXT_FLAG_ON));
	*/

freq_update();

*len = 5;

for(int i=0; i<*len; i++)
	buf[i] = i;

buf[0] = 0x82;

update_rssi();

	/*
	if( xx & CC1201_TX_FIFO_ERROR )
	{
		flush_rx();
		return 0xFFFFFFFF;
	}
	*/
	flush_rx();
	return 1;	// success
} 


/**
 * reads a standard register
 */
 uint8_t CC1201::readReg(uint8_t addr, bool ext_flag)
 {
 	uint8_t returnVal;

 	if ( (addr == 0x2F) & (ext_flag == EXT_FLAG_OFF) )
 	{
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
uint8_t CC1201::readReg(uint8_t addr, uint8_t* buffer, uint8_t len, bool ext_flag)
{
	uint8_t status_byte;

	if ( (addr >= 0x2F) & (ext_flag == EXT_FLAG_OFF) )
	{
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
uint8_t CC1201::writeReg(uint8_t addr, uint8_t* buffer, uint8_t len, bool ext_flag)
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
 uint8_t CC1201::readRegExt(uint8_t addr, uint8_t* buffer, uint8_t len)
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
 uint8_t CC1201::writeRegExt(uint8_t addr, uint8_t* buffer, uint8_t len)
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
 	if (addr > 0x3d || addr < 0x30)
 	{
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

 	if (_chip_version != CC1201_EXPECTED_PART_NUMBER)
 	{
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
 	recurseCount++;
 	if (recurseCount >= 10)
 	{
 		log(SEVERE, LINE_INFO, "cannot calibrate radio -> system reset");
        //Thread::wait(500);
 		mbed_reset();
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
 	DigitalIn* SO = new DigitalIn(_miso_pin);


 	log(INF2, LINE_INFO, "wait for olliscator assertion");
 	uint8_t waitCycles = 20;    
 	while (*SO)
 	{
 		if (waitCycles == 0)
 		{
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
 	log(WARN, "CC1201 FIFO", "TX FIFO flushed");
 }


 void CC1201::flush_rx(void)
 {
 	idle();
 	strobe(CC1201_STROBE_SFRX);
 	log(WARN, "CC1201 FIFO", "RX FIFO flushed");
 }


 void CC1201::calibrate(void)
 {
 	idle();
 	strobe(CC1201_STROBE_SCAL);
 }


 void CC1201::update_rssi(void)
 {
 	uint8_t rssi_reg[2];
	rssi_reg[0] = readReg(CC1201EXT_RSSI0, EXT_FLAG_ON);
	rssi_reg[1] = readReg(CC1201EXT_RSSI1, EXT_FLAG_ON);

 	uint16_t rssi_val = ( (rssi_reg[1] << 4) | (rssi_reg[0] & 0x78) >> 3);
 	rssi_val = ~(rssi_val) + 1;
 	rssi_val = rssi_val << 4;
 	int16_t rssi_val_s = (int16_t)rssi_val;
 	rssi_val_s = rssi_val_s >> 4;
 	_rssi = rssi_val_s * 0.0625 - 81;

	//log(INF2, LINE_INFO, "RSSI Register Val: 0x%03X", rssi_val);

/*
	if (rssi_val & 0x400)	// if negative (this is a 2's compliment register value)
	{
		_rssi = (float) ((2048 - rssi_val) >> 1);
	} else
	{
		_rssi = (float) (rssi_val >> 1);
	}
	*/
	/*
	if (rssi_val == 0x800)
	{
		_rssi = -999.0;
	} else
	{
		_rssi = static_cast<float>(rssi_val - 81) / 16;	// RSSI offset value. RSSI now stored as value of dBm. 0.0625 resolution
	}
	*/
	
}

float CC1201::rssi(void)
{
	return _rssi;
}

uint8_t CC1201::idle(void)
{
	uint8_t status_byte = strobe(CC1201_STROBE_SIDLE);
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

float CC1201::frequency(void)
{
	uint8_t buf[5];
	uint16_t freq_offset;
	uint32_t freq_base;
	float freq;
	
	freq_update();
	
	//readReg(CC1201EXT_FREQOFF1, buf, 5, EXT_FLAG_ON);
	buf[0] = readReg(CC1201EXT_FREQOFF1, EXT_FLAG_ON);
	buf[1] = readReg(CC1201EXT_FREQOFF0, EXT_FLAG_ON);
	buf[2] = readReg(CC1201EXT_FREQ2, EXT_FLAG_ON);
	buf[3] = readReg(CC1201EXT_FREQ1, EXT_FLAG_ON);
	buf[4] = readReg(CC1201EXT_FREQ0, EXT_FLAG_ON);

	freq_offset = (buf[0] << 8) | (buf[1]);
	freq_offset = (~freq_offset) + 1;
	freq_base = (buf[2] << 16) | (buf[3] << 8) | (buf[4]);

	freq = 40 * static_cast<float>((freq_base >> 16) + (freq_offset >> 18));
	freq /= 4;

	log(OK, LINE_INFO, "Operating Frequency: %3.2f MHz", freq);

	return freq;
}

bool CC1201::isLocked(void)
{
	// This is only valid in RX, TX, & FSTXON
	return (readReg(CC1201EXT_FSCAL_CTRL, EXT_FLAG_ON) & 0x01);
}