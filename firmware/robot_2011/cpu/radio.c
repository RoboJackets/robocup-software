#include "radio.h"
#include "timer.h"
#include "status.h"
#include "radio_config.h"

uint8_t radio_command(uint8_t cmd)
{
	radio_select();
	uint8_t status = spi_xfer(cmd);
	radio_deselect();
	
	return status;
}

uint8_t radio_read(uint8_t addr)
{
	radio_select();
	spi_xfer(addr | CC_READ);
	uint8_t value = spi_xfer(SNOP);
	radio_deselect();
	
	return value;
}

uint8_t radio_write(uint8_t addr, uint8_t value)
{
	radio_select();
	uint8_t status = spi_xfer(addr);
	spi_xfer(value);
	radio_deselect();
	
	return status;
}

// Waits for the radio's version byte to have the expected value.
// This verifies that the radio crystal oscillator and SPI interface are working.
static int radio_check_version()
{
	// It may take some time for the radio's oscillator to start up
	for (int i = 0; i < 10; ++i)
	{
		if (radio_read(VERSION) == 0x04)
		{
			return 1;
		}
		
		delay_ms(10);
	}
	return 0;
}

int radio_init()
{
	// Reset
	delay_ms(50);
	radio_command(SRES);
	delay_ms(50);

	if (!radio_check_version())
	{
		failures |= Fail_Radio_Interface;
		return 0;
	}
	
	// Test GDO2 high
	radio_write(IOCFG2, 0x2f | GDOx_INVERT);
	if (!radio_gdo2())
	{
		// Interrupt line is stuck low
		failures |= Fail_Radio_Int_Low;
		return 0;
	}
	
	// Test GDO2 low
	radio_write(IOCFG2, 0x2f);
	if (radio_gdo2())
	{
		// Interrupt line is stuck high
		failures |= Fail_Radio_Int_High;
		return 0;
	}
	
	return 1;
}

void radio_configure()
{
	radio_command(SIDLE);
	
	radio_select();
	for (int i = 0; i < sizeof(cc1101_regs); ++i)
	{
		spi_xfer(cc1101_regs[i]);
	}
	radio_deselect();
	
	radio_write(IOCFG2, 6 | GDOx_INVERT);
	
	radio_write(PKTLEN, Forward_Size);
	radio_command(SFRX);
	radio_command(SRX);
}
