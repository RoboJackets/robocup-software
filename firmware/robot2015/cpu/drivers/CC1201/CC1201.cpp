#include "CC1201.hpp"
#include "CC1101-Defines.hpp"
#include "logger.hpp"

CC1201::CC1201() : CommLink() {}

CC1201::CC1201(PinName mosi, PinName miso, PinName sck, PinName cs, PinName intPin) :
	CommLink(mosi, miso, sck, cs, intPin)
{
	strobe(CCXXX1_SRES);
}

CC1201::~CC1201() {}

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

	log(INF2, "CC1201", "PACKET TRANSMITTED\r\n  Bytes: %u", size);

	// [X] - 3 - Send the data to the CC1101. Increment the size value by 1 
	//before doing so to account for the buffer's inserted value
	// =================
	writeReg(CCXXX1_TXFIFO, buffer, ++size);

	// [X] - 4 - Enter the TX state.
	// =================
	strobe(CCXXX1_STX);

	// [X] - 5 - Wait until radio enters back to the RX state.
	// =================
	// *Note: Takes very few cycles, so might as well wait before returning 
	//to elimate querky errors.
	while(mode() != 0x0D);

	// [] - 6 - Return any error codes if necessary.
	// =================
	return 0;   // success
}

int32_t CC1201::getData(uint8_t* paramOne, uint8_t* paramTwo)
{
	return -1;
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

	_spi->write(addr | CCXXX1_WRITE_BURST);
	for (uint8_t index = 0; index < len; index++)
	{
		_spi->write(buffer[index]);
	}
	
	toggle_cs();
}

uint8_t CC1201::strobe(uint8_t addr)
{
	toggle_cs();
	uint8_t ret = _spi->write(addr);
	toggle_cs();
	return ret;
}

uint8_t CC1201::mode(void)
{
    return status(CCXXX1_MARCSTATE);
}

uint8_t CC1201::status(uint8_t addr)
{
    toggle_cs();
    _spi->write(addr | CCXXX1_READ_BURST);
    uint8_t ret = _spi->write(0);
    toggle_cs();
    return ret;
}

void CC1201::reset(void)
{
    strobe(CCXXX1_SRES);
}

int32_t CC1201::selfTest(void)
{
	return -1;
}

bool CC1201::isConnected(void)
{
	return true;
}
