#include "CC1201Radio.hpp"
#include "mbed.h"
#include "rtos.h"

using namespace std;

CC1201::CC1201() : CommLink() {}

CC1201::CC1201(PinName mosi, PinName miso, PinName sck, PinName cs, PinName intPin) :
	CommLink(mosi, miso, sck, cs, intPin)
{
	strobe(CC1201_STROBE_SRES);
	Thread::wait(300);
	strobe(CC1201_STROBE_SIDLE);
	Thread::wait(400);
}

CC1201::~CC1201() {}

void CC1201::sendGarbage(void)
{
	strobe(CC1201_STROBE_SIDLE);
	Thread::wait(50);
	writeReg(CC1201_PKT_LEN, 0x1E);
	writeReg(CC1201_PKT_CFG2, 0x00 | (1 << 1));
	writeReg(CC1201EXT_TXFIRST, 0x00);
	writeReg(CC1201EXT_TXLAST, 0x08);

	//if (decodeState(strobe(CC1201_STROBE_SNOP)) == 0x07)
	//	strobe(CC1201_STROBE_SFTX);

	strobe(CC1201_STROBE_STX);
}

/**
 *
 */
int32_t CC1201::sendData(uint8_t* buffer, uint8_t size)
{
	// [X] - 1 - Move all values down by 1 to make room for the packet's size value.
	// =================
	for (int i = size - 1; i > 0; i--)
		buffer[i] = buffer[i - 1];

	// [X] - 2 - Place the packet's size as the array's first value.
	// =================
	buffer[0] = size;

	log(INF2, "CC1201", "PACKET TRANSMITTED\r\n  Bytes: %u", size);

	//strobe(CC1201_STROBE

	// [X] - 3 - Send the data to the CC1101. Increment the size value by 1 
	//before doing so to account for the buffer's inserted value
	// =================
	writeReg(CC1201_TX_FIFO, buffer, size);

	// [X] - 4 - Enter the TX state.
	// =================
	strobe(CC1201_STROBE_STX);

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
	return -1;
} 

/**
 * reads a standard register
 */
uint8_t CC1201::readReg(uint8_t addr)
{
	const uint8_t DATA_BYTE = 0x00;
	uint8_t returnVal;

	if (addr >= 0x2F)
	{
		log(WARN, "CC1201::readReg", "readReg invalid address: %02X", addr);
		return -1;
	}

	//addr |=   1 << 7;
	addr &= ~(1 << 6); //should be redundant but leaving for security 
	_spi->write(addr | CC1201_READ_SINGLE);
	returnVal = _spi->write(DATA_BYTE);
	return returnVal;
}

void CC1201::readReg(uint8_t addr, uint8_t* buffer, uint8_t len)
{
	return;
} 

/**
 * reads an extended register
 */
uint8_t CC1201::readRegExt(uint8_t addr)
{
	const uint8_t DATA_BYTE = 0x00;
	uint8_t returnVal;

	toggle_cs();
	_spi->write(CC1201_EXTENDED_ACCESS_READ);
	_spi->write(addr);
	returnVal = _spi->write(DATA_BYTE);
	toggle_cs();
	return returnVal;
}

void CC1201::readRegExt(uint8_t addr, uint8_t* buffer, uint8_t len)
{
	return;
}

void CC1201::writeReg(uint8_t addr, uint8_t value)
{
	toggle_cs();
	_spi->write(addr);
	_spi->write(value);
	toggle_cs();
}

void CC1201::writeReg(uint8_t addr, uint8_t* buffer, uint8_t len)
{
	toggle_cs();

	_spi->write(addr | CC1201_WRITE_BURST);
	for (uint8_t index = 0; index < len; index++)
	{
		_spi->write(buffer[index]);
	}
	
	toggle_cs();
}

void CC1201::writeRegExt(uint8_t addr, uint8_t value)
{
	toggle_cs();
	_spi->write(CC1201_EXTENDED_ACCESS_WRITE);
	_spi->write(addr);
	_spi->write(value);
	toggle_cs();
}

void CC1201::writeRegExt(uint8_t addr, uint8_t* buffer, uint8_t value)
{
	return;
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
    return readRegExt(CC1201EXT_MARCSTATE);
}

uint8_t CC1201::status(uint8_t addr)
{
	return strobe(CC1201_STROBE_SNOP);
}

void CC1201::reset(void)
{
    strobe(CC1201_STROBE_SRES);
}

int32_t CC1201::selfTest(void)
{
	_chip_version = readRegExt(CC1201EXT_PARTNUMBER);
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

    return 0;
}

bool CC1201::isConnected(void)
{
	return true;
}

uint8_t CC1201::decodeState(uint8_t nopRet)
{
	nopRet &= ~(1 << 7);
	nopRet >>= 4;
	return nopRet;
}

string CC1201::modeToStr(uint8_t mode)
{
	switch (mode)
	{
		default: return "unk";
	}
}
