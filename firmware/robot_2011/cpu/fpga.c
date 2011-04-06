#include <board.h>

#include "fpga.h"
#include "status.h"
#include "spi.h"
#include "timer.h"

uint_fast16_t encoder[4];
int_fast16_t encoder_delta[4];

int_fast8_t wheel_out[4];
int_fast8_t dribble_out;

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

	// Verify interface version
	spi_select(NPCS_FPGA);
	uint8_t version = spi_xfer(0);
	spi_deselect();
	
	if (version != 0x02)
	{
		failures |= Fail_FPGA_Version;
	}
	
	return ret;
}

void fpga_update()
{
	uint8_t tx[10] = {0}, rx[10];
	
	// Save old encoder counts
	int_fast16_t last_encoder[4];
	for (int i = 0; i < 4; ++i)
	{
		last_encoder[i] = encoder[i];
	}
	
	//FIXME - Select 2008/2010 mechanical base with switch DP0
	for (int i = 0; i < 4; ++i)
	{
		int_fast8_t cmd = wheel_out[i];
		if (cmd < 0)
		{
			cmd = -cmd;
			tx[i * 2] = cmd << 1;
			tx[i * 2 + 1] = cmd >> 7;
			tx[i * 2 + 1] |= 2;
		} else {
			tx[i * 2] = cmd << 1;
			tx[i * 2 + 1] = cmd >> 7;
		}
	}
	tx[8] = dribble_out << 1;
	tx[9] = (dribble_out >> 7) | 2;
	
	// Swap data with the FPGA
	spi_select(NPCS_FPGA);
	for (int i = 0; i < sizeof(tx); ++i)
	{
		rx[i] = spi_xfer(tx[i]);
	}
	spi_deselect();
	
	// Unpack data from the FPGA's response
	encoder[0] = rx[1] | (rx[2] << 8);
	encoder[1] = rx[3] | (rx[4] << 8);
	encoder[2] = rx[5] | (rx[6] << 8);
	encoder[3] = rx[7] | (rx[8] << 8);
	motor_faults = rx[9];
	
	for (int i = 0; i < 4; ++i)
	{
		encoder_delta[i] = (int16_t)(encoder[i] - last_encoder[i]);
	}
}
