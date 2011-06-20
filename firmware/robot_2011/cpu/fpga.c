#include <board.h>

#include "fpga.h"
#include "status.h"
#include "spi.h"
#include "timer.h"

uint_fast16_t encoder[4];
int_fast16_t encoder_delta[4];

int_fast8_t wheel_out[4];
int_fast8_t dribble_out;

uint_fast8_t kick_strength;
uint_fast8_t use_chipper;

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
				ret = 1;
				goto good;
			}
		}
		
		// The FPGA took too long to configure.
		// Configuration memory is probably empty/corrupt.
		// It's also possible that the board is on a bench power supply
		// with a low current limit and the supply voltage rose very slowly
		// due to inrush current to the motor driver capacitors.
		//
		// Shut down the FPGA, since the MCU needs to become the SPI master.
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
	
	if (version != 0x03)
	{
		failures |= Fail_FPGA_Version;
	}
	
	return ret;
}

void fpga_read_status()
{
	if (failures & Fail_FPGA)
	{
		return;
	}
	
	uint8_t rx[12];

	// Save old encoder counts
	int_fast16_t last_encoder[4];
	for (int i = 0; i < 4; ++i)
	{
		last_encoder[i] = encoder[i];
	}

	// Swap data with the FPGA
	spi_select(NPCS_FPGA);
	for (int i = 0; i < sizeof(rx); ++i)
	{
		rx[i] = spi_xfer(0);
	}
	spi_deselect();
	
	// Unpack data from the FPGA's response
	encoder[0] = rx[1] | (rx[2] << 8);
	encoder[1] = rx[3] | (rx[4] << 8);
	encoder[2] = rx[5] | (rx[6] << 8);
	encoder[3] = rx[7] | (rx[8] << 8);
	current_motor_faults = rx[9];
	motor_faults |= current_motor_faults;
	kicker_status = rx[10];
	
	for (int i = 0; i < 4; ++i)
	{
		encoder_delta[i] = (int16_t)(encoder[i] - last_encoder[i]);
	}
}

void fpga_send_commands()
{
	if (failures & Fail_FPGA)
	{
		return;
	}
	
	uint8_t tx[12] = {0};

	//FIXME - Select 2008/2010 mechanical base with switch DP0
        tx[0] = 0x01;   // Command: set motor speeds
	for (int i = 0; i < 4; ++i)
	{
		int_fast8_t cmd = wheel_out[i];
		if (cmd < 0)
		{
			cmd = -cmd;
			tx[i * 2 + 1] = cmd << 2;
			tx[i * 2 + 2] = cmd >> 6;
			tx[i * 2 + 2] |= 2;
		} else {
			tx[i * 2 + 1] = cmd << 2;
			tx[i * 2 + 2] = cmd >> 6;
		}
	}
	tx[9] = dribble_out << 1;
	tx[10] = (dribble_out >> 7) | 2;
	tx[10] |= 0x80;	// Always enable kicker charging
	if (use_chipper)
	{
		tx[10] |= 0x40;	// Select chipper
	}
	tx[11] = kick_strength;
	
	// Swap data with the FPGA
	spi_select(NPCS_FPGA);
	for (int i = 0; i < sizeof(tx); ++i)
	{
		spi_xfer(tx[i]);
	}
	spi_deselect();
}
