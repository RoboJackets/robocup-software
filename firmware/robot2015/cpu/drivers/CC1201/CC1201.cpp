#include "CC1201Radio.hpp"
#include "mbed.h"
#include "rtos.h"

extern "C" void mbed_reset();

using namespace std;

CC1201::CC1201() : CommLink() {}

CC1201::CC1201(PinName mosi, PinName miso, PinName sck, PinName cs, PinName intPin) :
	CommLink(mosi, miso, sck, cs, intPin)
{
    //powerOnReset();
	reset();
	strobe(CC1201_STROBE_SIDLE);
	flush_rx(); flushtx();
	Thread::wait(100);
	selfTest();
	//CommLink::ready();
	log(OK, "CC1201", "CC1201 Ready!");
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
	// [X] - 1 - Move all values down by 1 to make room for the packet's size value.
	// =================
	for (int i = size; i > 0; i--)
		buffer[i] = buffer[i - 1];

	// [X] - 2 - Place the packet's size as the array's first value.
	// =================
	buffer[0] = size;

	//log(INF2, "CC1201", "PACKET TRANSMITTED\r\n  Bytes: %u", size);

	log(WARN, "CC1201::sendData", "MARCSTATE before TX: %02X", mode());

	// [X] - 3 - Send the data to the CC1101. Increment the size value by 1 
	//before doing so to account for the buffer's inserted value
	// =================
	writeReg(CC1201_TX_FIFO, buffer, ++size);

	// [X] - 4 - Enter the TX state.
	// =================
	strobe(CC1201_STROBE_STX);
	//strobe(CC1201_STROBE_SIDLE);
	//strobe(CC1201_STROBE_SFTX);

	//if (decodeState(strobe(CC1201_STROBE_SNOP)) == 7)
	//	strobe(CC1201_STROBE_SFTX); 

	// [X] - 5 - Wait until radio enters back to the RX state.
	// =================
	// *Note: Takes very few cycles, so might as well wait before returning 
	//to elimate querky errors.
	//while(mode() != 0x0D);

	// [] - 6 - Return any error codes if necessary.
	// =================
	return 0;   // success
}

int32_t CC1201::getData(uint8_t* paramOne, uint8_t* paramTwo)
{
	//log(WARN, "CC1201::getData() (virtual CommLink::getData())", "DATA GET NOT IMPLEMENTED");
	log(INF3, "CC1201::getData", "RXFIRST: %02X, RXLAST: %02X",
			readReg(CC1201EXT_RXFIRST, EXT_FLAG_ON),
			readReg(CC1201EXT_RXLAST, EXT_FLAG_ON));

	strobe(CC1201_STROBE_SFRX);
	
	return 0xFF;
} 


/**
 * reads a standard register
 */
uint8_t CC1201::readReg(uint8_t addr, bool ext_flag)
{
	uint8_t returnVal;

	if ( (addr == 0x2F) & (ext_flag == EXT_FLAG_OFF) )
	{
		log(WARN, "CC1201::readReg", "readReg invalid address: %02X", addr);
		return 0xFF;
	}

	if (ext_flag == EXT_FLAG_ON)
		return readRegExt(addr);

	addr &= 0xBF; // Should be redundant but leaving for security. We don't want to accidently do a burst read.

	toggle_cs();
	_spi->write(addr | CC1201_READ_SINGLE);
	returnVal = _spi->write(0x00);
	toggle_cs();

	return returnVal;
}
void CC1201::readReg(uint8_t addr, uint8_t* buffer, uint8_t len, bool ext_flag)
{
	if ( (addr >= 0x2F) & (ext_flag == EXT_FLAG_OFF) )
		return log(WARN, "CC1201::readReg", "readReg invalid address: %02X", addr);

	if ( ext_flag == EXT_FLAG_ON )
		return readRegExt(addr, buffer, len);

	toggle_cs();
	_spi->write(addr | CC1201_READ_SINGLE | CC1201_BURST);
	for (uint8_t i = 0; i < len; i++)
		buffer[i] = _spi->write(0x00);

	toggle_cs();
}


void CC1201::writeReg(uint8_t addr, uint8_t value, bool ext_flag)
{
	addr &= 0x3F; // Don't accidently do a burst or read

	if (ext_flag == EXT_FLAG_ON)
		return writeRegExt(addr, value);

	toggle_cs();
	_spi->write(addr);
	_spi->write(value);
	toggle_cs();
}
void CC1201::writeReg(uint8_t addr, uint8_t* buffer, uint8_t len, bool ext_flag)
{
	addr &= 0x7F; // Don't accidently do a read

	if (ext_flag == EXT_FLAG_ON)
		return writeRegExt(addr, buffer, len);

	toggle_cs();
	_spi->write(addr | CC1201_BURST);
	for (uint8_t i = 0; i < len; i++)
		_spi->write(buffer[i]);

	toggle_cs();
}


/**
 * reads an extended register
 */
uint8_t CC1201::readRegExt(uint8_t addr)
{
	// Only callable from readReg(), so no checks needed
	uint8_t returnVal;

	toggle_cs();
	_spi->write(CC1201_EXTENDED_ACCESS_READ);
	_spi->write(addr);
	returnVal = _spi->write(0x00);
	toggle_cs();

	return returnVal;
}
void CC1201::readRegExt(uint8_t addr, uint8_t* buffer, uint8_t len)
{
	// Only callable from readReg(), so no checks needed
	toggle_cs();
	_spi->write(CC1201_EXTENDED_ACCESS_READ | CC1201_BURST);
	_spi->write(addr);
	for (uint8_t i = 0; i < len; i++)
		buffer[i] = _spi->write(0x00);

	toggle_cs();
}


void CC1201::writeRegExt(uint8_t addr, uint8_t value)
{
	// Only callable from writeReg(), so no checks needed
	toggle_cs();
	_spi->write(CC1201_EXTENDED_ACCESS_WRITE);
	_spi->write(addr);
	_spi->write(value);
	toggle_cs();
}
void CC1201::writeRegExt(uint8_t addr, uint8_t* buffer, uint8_t len)
{
	// Only callable from writeReg(), so no checks needed
	toggle_cs();
	_spi->write(CC1201_EXTENDED_ACCESS_WRITE | CC1201_BURST);
	_spi->write(addr);
	for (uint8_t i = 0; i < len; i++)
		_spi->write(buffer[i]);

	toggle_cs();
}


uint8_t CC1201::strobe(uint8_t addr)
{
	if (addr > 0x3d || addr < 0x30)
	{
		log(WARN, "CC1201::strobe", "Invalid address: %02X", addr);
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
    strobe(CC1201_STROBE_SIDLE);
    Thread::wait(100);

	toggle_cs();
	_spi->write(CC1201_STROBE_SRES);
	Thread::wait(600);
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
		log(FATAL, "CC1201",
		    "FATAL ERROR\r\n"
		    "  Wrong version number returned from chip's 'PARTNUMBER' register (Addr: 0x%02X)\r\n"
		    "  Expected: 0x%02X\r\n"
		    "  Found:    0x%02X\r\n"
			"\r\n"
		    , CC1201EXT_PARTNUMBER, CC1201_EXPECTED_PART_NUMBER, _chip_version);

		return -1;
	}

	strobe(CC1201_STROBE_SIDLE);

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
        log(SEVERE, "CC1201::reset", "cannot calibrate radio -> system reset");
        Thread::wait(500);
        mbed_reset();
    }

    log(INF1, "CC1201::reset", "Beginning power on reset (POR)");

    log(INF2, "  CC1201::reset", "Strobe SIDLE");
    strobe(CC1201_STROBE_SIDLE);
    log(INF2, "  CC1201::reset", "IDLE strobe OK.");
    
    log(INF2, "  CC1201::reset", "Force CS low");
    *_cs = 0;
    log(INF2, "  CC1201::reset", "Strobe SRES");
    _spi->write(CC1201_STROBE_SRES);
    log(INF2, "  CC1201::reset", "dealloc SPI");
    delete _spi;
    log(INF2, "  CC1201::reset", "(MI)SO alloc digIn");
    DigitalIn* SO = new DigitalIn(_miso_pin);


    log(INF2, "  CC1201::reset", "wait for olliscator assertion");
    uint8_t waitCycles = 20;    
    while (*SO)
    {
        if (waitCycles == 0)
        {
            log(WARN, "  CC1201::reset", "calibration settled assertion timeout -> retry");
            //powerOnReset();	// There's absolutely no need to do this. If it doesn't calibrate in 20 cycles, it will never do it successfully.
            return;
        }
        Thread::wait(100);
        waitCycles--;
    }
    recurseCount = 0;

    log(INF2, "  CC1201::reset", "dealloc digIn");
    delete SO;
    log(INF2, "  CC1201::reset", "force CSn high");
    *_cs = 1;
    log(INF2, "  CC1201::reset", "setup SPI");
    setup_spi();
    log(INF2, "  CC1201::reset", "POR COMPLETE!");

    _isInit = false;

/*
    log(INF1, "CC1201", "Beginning Power-on-Reset routine...");

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
    //while(*SO2);

    // make sure chip is deselected before returning
    *_cs = 1;

    // reestablish the SPI bus for the final time after removing the DigitalIn object
    delete SO2;
    setup_spi();

    log(INF1, "CC1201", "CC1201 Power-on-Reset complete");
*/
}

/*
void CC1201::ready(void)
{
	CommLink::ready();
}
*/

uint8_t CC1201::decodeState(uint8_t nopRet)
{
	return (nopRet >> 4) & 0x07;
}

string CC1201::modeToStr(uint8_t mode)
{
	switch (mode)
	{
		default: return "unk";
	}
}


void flush_tx(void)
{
	strobe(CC1201_STROBE_SFTX);
}


void flush_rx(void)
{
	strobe(CC1201_STROBE_FRX);
}


void calibrate(void)
{
	strobe(CC1201_STROBE_SCAL);
}


void rssi(void)
{
	uint8_t rssi_val = readReg(CC1201EXT_RSSI1);
}