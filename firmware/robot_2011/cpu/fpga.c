#include <board.h>

#include "fpga.h"
#include "status.h"
#include "spi.h"
#include "timer.h"

int fpga_init()
{
	int ret;
	
	failures &= ~Fail_FPGA;
	
	// Disable SPI drivers so the FPGA can reconfigure
	spi_shutdown();
	
	// Release PROGB so the FPGA can start configuring
	AT91C_BASE_PIOA->PIO_SODR = MCU_PROGB;
	
	// Wait for the FPGA to start configuring
	delay_ms(5);
	if (!(AT91C_BASE_PIOA->PIO_PDSR & FLASH_NCS))
	{
		// FLASH_NCS is low: the FPGA is reading
		//
		// Wait for the FPGA to finish configuring
		for (int i = 0; i < 100; ++i)
		{
			delay_ms(10);
			if (AT91C_BASE_PIOA->PIO_PDSR & FLASH_NCS)
			{
				// FLASH_NCS is high: the FPGA is done
				//FIXME - Read version register
				ret = 1;
				goto good;
			}
		}
		
		// The FPGA took too long to configure.
		// Configuration memory is probably empty/corrupt.
		// Shut down the FPGA, since the MCU needs to become the SPI master
		AT91C_BASE_PIOA->PIO_CODR = MCU_PROGB;
		
		failures |= Fail_FPGA_Config;
		
		// Become the SPI master
		spi_init();
		
		return 0;
	} else {
		// FPGA did not start reading SPI flash - already configured?
		ret = 2;
	}
good:
	// Become the SPI master
	spi_init();

	//FIXME - Read version
	spi_select(NPCS_FPGA);
	uint8_t s0 = spi_xfer(0);
	uint8_t s1 = spi_xfer(0);
	spi_deselect();
	
	if (s0 != 0xc9 || s1 != 0xa5)
	{
		failures |= Fail_FPGA_Logic;
		failures |= Fail_FPGA_Version;
	}
	
	return ret;
}
