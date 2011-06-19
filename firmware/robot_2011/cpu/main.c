#include <board.h>
#include <stdio.h>

#include "timer.h"
#include "console.h"
#include "sound.h"
#include "status.h"
#include "radio.h"
#include "adc.h"
#include "power.h"
#include "control.h"
#include "ball_sense.h"
#include "fpga.h"
#include "i2c.h"
#include "stall.h"
#include "imu.h"

// Last forward packet
uint8_t forward_packet[Forward_Size];

// Last reverse packet
uint8_t reverse_packet[Reverse_Size];

int_fast8_t wheel_command[4];
uint_fast8_t dribble_command;
uint_fast8_t kick_command;

uint32_t rx_lost_time;
int_fast8_t last_rssi;

int in_reverse;

// Last time the 5ms periodic code was executed
unsigned int update_time;

void (*debug_update)(void) = 0;

static int forward_packet_received()
{
	uint8_t bytes = radio_read(RXBYTES);
	
	if (bytes != (Forward_Size + 2))
	{
		// Bad CRC, so the packet was flushed (or the radio got misconfigured).
		radio_command(SFRX);
		radio_command(SRX);
		return 0;
	}
	
	// Read the packet from the radio
	radio_select();
	spi_xfer(RXFIFO | CC_READ | CC_BURST);
	for (int i = 0; i < Forward_Size; ++i)
	{
		forward_packet[i] = spi_xfer(SNOP);
	}
	
	// Read status bytes
	last_rssi = (int8_t)spi_xfer(SNOP);
	uint8_t status = spi_xfer(SNOP);
	radio_deselect();
	
	if (!(status & 0x80))
	{
		// Bad CRC
		//
		// Autoflush is supposed to be on so this should never happen.
		// If we get here and autoflush is on, this means some bytes have been lost
		// and the status byte isn't really the status byte.
		radio_command(SFRX);
		radio_command(SRX);
		return 0;
	}
	
	rx_lost_time = current_time;
	LED_TOGGLE(LED_RG);
	LED_OFF(LED_RR);
	
	uint8_t reverse_id = forward_packet[0] & 15;
	
	// Clear motor commands in case this robot's ID does not appear in the packet
	for (int i = 0; i < 4; ++i)
	{
		wheel_command[i] = 0;
	}
	dribble_command = 0;

	// Get motor commands from the packet
	int offset = 1;
	for (int slot = 0; slot < 5; ++slot)
	{
		if ((forward_packet[offset + 4] & 0x0f) == robot_id)
		{
			for (int i = 0; i < 4; ++i)
			{
				wheel_command[i] = (int8_t)forward_packet[offset + i];
			}
			
			// Convert the dribbler speed from the top four bits in a byte to seven bits
			dribble_command = (forward_packet[offset + 4] & 0xf0) >> 1;
			dribble_command |= dribble_command >> 4;
			
			kick_command = forward_packet[offset + 5];
		}
		offset += 6;
	}
	
	//FIXME - Testing chipper with only one 2011 robot
	if (forward_packet[0] & 0x80)
	{
		use_chipper = 1;
	} else {
		use_chipper = 0;
	}

	if (reverse_id == robot_id)
	{
		// Build and send a reverse packet
		radio_write(PKTLEN, Reverse_Size);
		radio_command(SFTX);
		
		reverse_packet[0] = robot_id;
		reverse_packet[1] = last_rssi;
		reverse_packet[2] = 0x00;
		reverse_packet[3] = 0; //FIXME - Battery
		reverse_packet[4] = kicker_status;
		reverse_packet[5] = motor_faults;
		
		if (have_ball)
		{
			reverse_packet[5] |= 1 << 5;
		}
		
		if (failures & Fail_Ball)
		{
			reverse_packet[5] |= 1 << 6;
		}
		
		reverse_packet[10] = 0;
		for (int i = 0; i < 4; ++i)
		{
			reverse_packet[6 + i] = encoder[i];
			reverse_packet[10] |= (encoder[i] & 0x300) >> (8 - i * 2);
		}
		
		radio_select();
		spi_xfer(TXFIFO | CC_BURST);
		for (int i = 0; i < Reverse_Size; ++i)
		{
			spi_xfer(reverse_packet[i]);
		}
		radio_deselect();
		
		// Start transmitting.  When this finishes, the radio will automatically switch to RX
		// without calibrating (because it doesn't go through IDLE).
		radio_command(STX);
		
		in_reverse = 1;
	} else {
		// Get ready to receive another forward packet
		radio_command(SRX);
	}
	
	return 1;
}

void reverse_packet_sent()
{
	LED_TOGGLE(LED_RY);
	radio_write(PKTLEN, Forward_Size);
	radio_command(SRX);

	in_reverse = 0;
}

#if 0
// Use this main() to debug startup code, IRQ, or linker script problems.
// You can change SConstruct to build for SRAM because linker garbage collection
// will remove all the large code.
int main()
{
	// If no LEDs turn on, we didn't reach this point.
	// Either startup code is broken, built for the wrong memory (flash but running in SRAM?),
	// or LowLevelInit failed (failed to return properly do to interworking?).
	
	// Turn on all LEDs
	AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOA;
	AT91C_BASE_PIOA->PIO_CODR = LED_ALL;
	AT91C_BASE_PIOA->PIO_OER = LED_ALL;
	
	// Start PIT.  An IRQ will occur in about 1ms.
	timer_init();
	delay_ms(500);
	
	// Turn off all LEDs
	AT91C_BASE_PIOA->PIO_SODR = LED_ALL;
	
	// If the LEDs turn on and stay on, the delay failed (IRQ crashed or did not trigger).
	// If the LEDs turn off, IRQs and startup code are working.
	
	while (1);
}
#else

int main()
{
	// Set up watchdog timer
	AT91C_BASE_WDTC->WDTC_WDCR = 0xa5000001;
	AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDRSTEN | AT91C_WDTC_WDDBGHLT | (0xfff << 16) | 0x0ff;
	
	// Enable user reset (reset button)
	AT91C_BASE_SYS->RSTC_RMR = 0xa5000000 | AT91C_RSTC_URSTEN;
	
	// Set up PIOs
	// Initially, FLASH_NCS is a PIO because the FPGA will be driving it.
	// After the FPGA is configured (or we give up on it), FLASH_NCS is assigned to SPI.  This happens later in spi_init().
	AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOA;	// Turn on PIO clock
	// Connect some pins to the PIO controller
	AT91C_BASE_PIOA->PIO_PER = LED_ALL | MCU_PROGB | FLASH_NCS | RADIO_INT | VBUS | BALL_LED;
	AT91C_BASE_PIOA->PIO_ODR = ~0;					// Disable all outputs (FIXME - unnecessary?)
	AT91C_BASE_PIOA->PIO_OWER = LED_ALL;			// Allow LED states to be written directly
	AT91C_BASE_PIOA->PIO_CODR = LED_ALL;			// Turn on all LEDs
	AT91C_BASE_PIOA->PIO_SODR = BALL_LED;			// Turn off ball sensor LED
	AT91C_BASE_PIOA->PIO_OER = LED_ALL | BALL_LED;	// Enable outputs
	// Enable and disable pullups
	AT91C_BASE_PIOA->PIO_PPUER = RADIO_INT | RADIO_NCS | FPGA_NCS | FLASH_NCS | MISO | ID0 | ID1 | ID2 | ID3 | DP1 | DP2 | DP4;
	AT91C_BASE_PIOA->PIO_PPUDR = VBUS | M2DIV | M3DIV | M5DIV | BALL_LED;
	
	// Set up MCU_PROGB as an open-drain output, initially high
	AT91C_BASE_PIOA->PIO_SODR = MCU_PROGB;
	AT91C_BASE_PIOA->PIO_MDER = MCU_PROGB;
	AT91C_BASE_PIOA->PIO_OER = MCU_PROGB;
	
	timer_init();
	
	// At this point, the FPGA is presumed to be the SPI master.
	// Wait for it to configure and determine if it works.
	// If not, it must be disabled.
	// This calls spi_init.
	fpga_init();
	
	// Find out if the radio works.
	// This tests SPI communications with the radio and tests if
	// the interrupt line is working.
	radio_init();
	
	// Set up the ADC
	adc_init();
	
	// Check for low/high supply voltage
	power_init();
	
	// Set up I2C
	i2c_init();
	
	// Set up the IMU
	imu_init();
	
	// Turn off LEDs
	LED_OFF(LED_ALL);
	
	if (failures == 0)
	{
		music_start(song_startup);
		
		// Enough hardware is working, so act like a robot
		controller = DEFAULT_CONTROLLER;
	} else if (failures & Fail_Power)
	{
		// Dead battery (probably)
		power_fail_music();
	} else {
		// We're a paperweight
		music_start(song_failure);
	}
	
	// Set up the radio.  After this, it will be able to transmit and receive.
	//FIXME - Multiple channels
	if (!(failures & Fail_Radio))
	{
		radio_configure();
	}
	
	rx_lost_time = current_time;
	
	if (controller && controller->init)
	{
		controller->init(0, 0);
	}
	
	stall_init();
	
	// Main loop
	in_reverse = 0;
	int lost_radio_count = 0;
	while (1)
	{
		// Reset the watchdog timer
		AT91C_BASE_WDTC->WDTC_WDCR = 0xa5000001;
		
		// Handle USB connect/disconnect
		check_usb_connection();
		
		// Read robot ID
		uint32_t inputs = AT91C_BASE_PIOA->PIO_PDSR;
		robot_id = 0;
		if (!(inputs & ID0))
		{
			robot_id = 1;
		}
		if (!(inputs & ID1))
		{
			robot_id |= 2;
		}
		if (!(inputs & ID2))
		{
			robot_id |= 4;
		}
		if (!(inputs & ID3))
		{
			robot_id |= 8;
		}
		
		if (!(failures & Fail_Radio))
		{
			// Flash LED and recalibrate radio when signal is lost
			if ((current_time - rx_lost_time) > 250)
			{
				rx_lost_time = current_time;
				LED_OFF(LED_RY);
				LED_OFF(LED_RG);
				LED_TOGGLE(LED_RR);
				
				++lost_radio_count;
				if (lost_radio_count == 10)
				{
					lost_radio_count = 0;
					radio_configure();
				} else {
					radio_command(SIDLE);
					radio_command(SFRX);
					radio_command(SRX);
				}
				
				// Clear drive commands
				for (int i = 0; i < 4; ++i)
				{
					wheel_command[i] = 0;
				}
				dribble_command = 0;
			}
			
			// Check for radio packets
			if (AT91C_BASE_PIOA->PIO_ISR & RADIO_INT && AT91C_BASE_PIOA->PIO_PDSR & RADIO_INT)
			{
				if (!in_reverse)
				{
					if (forward_packet_received() && controller && controller->received)
					{
						controller->received();
					}
				} else {
					reverse_packet_sent();
				}
			}
		}
		
		// Periodic activities
		if ((current_time - update_time) >= 5)
		{
			update_time = current_time;
			
			// Read ADC results
			adc_update();
			
			// Check things that depend on ADC results
			power_update();
			update_ball_sensor();
			
			// Read encoders
			if (!(failures & Fail_FPGA))
			{
				fpga_read_status();
			}
			
			// Detect stalled motors
			// This must be done before clearing motor outputs because it uses the old values
			stall_update();
			
			// Reset motor outputs in case the controller is broken
			for (int i = 0; i < 4; ++i)
			{
				wheel_out[i] = 0;
			}
			dribble_out = 0;
			
			// Allow kicking if we have the ball
			if (have_ball)
			{
				kick_strength = kick_command;
			} else {
				kick_strength = 0;
			}
			
			// Run the controller, if there is one
			if (controller && controller->update)
			{
				controller->update();
			}
			
			// Clear the commands for unusable motors
			uint8_t bad_motors = motor_faults | motor_stall;
			for (int i = 0; i < 4; ++i)
			{
				if (bad_motors & (1 << i))
				{
					wheel_out[i] = 0;
				}
			}
			if (bad_motors & (1 << Motor_Dribbler))
			{
				dribble_out = 0;
			}
			
			// Send commands to and read status from the FPGA
			if (!(failures & Fail_FPGA))
			{
				fpga_send_commands();
			}
			
			if (kicker_status & Kicker_Charged)
			{
				LED_ON(LED_LR);
			} else {
				LED_OFF(LED_LR);
			}
			
			if (usb_is_connected() && debug_update)
			{
				debug_update();
			}
		}
		
		// Keep power failure music playing continuously
		power_fail_music();
	}
}

#endif
