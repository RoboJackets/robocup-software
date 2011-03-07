#include <board.h>

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

// Last forward packet
uint8_t forward_packet[Forward_Size];

// Last reverse packet
uint8_t reverse_packet[Reverse_Size];

int8_t wheel_command[4];
uint8_t dribble;

uint32_t rx_lost_time;
int8_t last_rssi;

// Last time the 5ms periodic code was executed
unsigned int update_time;

// Most recent wheel encoder values
uint16_t encoder[4];
uint16_t last_encoder[4];

int8_t wheel_out[4];
int integral[4];

static int forward_packet_received()
{
	uint8_t bytes = radio_read(RXBYTES);
	LED_TOGGLE(LED_RY);
	
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
	last_rssi = spi_xfer(SNOP);
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
	
// 	uint8_t reverse_id = forward_packet[0] & 15;
	
	// Update sequence number history
// 	uint8_t sequence = forward_packet[0] >> 4;
	
	// Kicking
	uint8_t kick_id = forward_packet[1] & 15;
	if (kick_id == robot_id && forward_packet[2])
	{
// 		LED_ON(LED_RY);
	} else {
// 		LED_OFF(LED_RY);
	}
	
	// Kick/chip selection
/*	kicker_control &= ~0x20;
	if (forward_packet[1] & 0x10)
	{
	    kicker_control |= 0x20;
	}
	fpga_write(FPGA_Kicker_Status, kicker_control);*/

#if 0
	// Clear history bits for missed packets
	for (int i = (last_sequence + 1) & 15; i != sequence; i = (i + 1) & 15)
	{
		sequence_history &= ~(1 << i);
	}
	
	// Set the history bit for this packet
	sequence_history |= 1 << sequence;
	
	// Save this packet's sequence number for next time
	last_sequence = sequence;
	
	// Count lost packets
	int lost_packets = 0;
	for (int i = 0; i < 16; ++i)
	{
		if (!(sequence_history & (1 << i)))
		{
			++lost_packets;
		}
	}
#endif
	
	// Clear motor commands in case this robot's ID does not appear in the packet
	for (int i = 0; i < 4; ++i)
	{
		wheel_command[i] = 0;
	}
	dribble = 0;

	// Get motor commands from the packet
	int offset = 3;
	for (int slot = 0; slot < 5; ++slot)
	{
		if ((forward_packet[offset + 4] & 0x0f) == robot_id)
		{
			for (int i = 0; i < 4; ++i)
			{
				//FIXME - Select 2008/2010 mechanical base with switch DP0
				wheel_command[i] = -(int8_t)forward_packet[offset + i];
			}
			dribble = forward_packet[offset + 4] >> 4;
		}
		offset += 5;
	}

#if 0
	if (reverse_id == board_id)
	{
		// Build and send a reverse packet
		radio_write(PKTLEN, Reverse_Size);
		radio_command(SFTX);
		
		uint8_t fault = fpga_read(FPGA_Fault) & 0x1f;
		
		// Clear fault bits
		fpga_write(FPGA_Fault, fault);
		
		reverse_packet[0] = (lost_packets << 4) | board_id;
		reverse_packet[1] = last_rssi;
		reverse_packet[2] = 0x00;
		reverse_packet[3] = battery >> 2;
		reverse_packet[4] = fpga_read(FPGA_Kicker_Status);
		
		// Fault bits
		reverse_packet[5] = fault;
		
		if (ball_present)
		{
			reverse_packet[5] |= 1 << 5;
		}
		
		if (ball_sensor_fail)
		{
			reverse_packet[5] |= 1 << 6;
		}
		
		reverse_packet[10] = 0;
		for (int i = 0; i < 4; ++i)
		{
		    reverse_packet[6 + i] = last_tick[i];
		    reverse_packet[10] |= (last_tick[i] & 0x300) >> (8 - i * 2);
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
#else
	radio_command(SRX);
#endif
	
	return 1;
}

void update_fpga()
{
	uint8_t tx[10] = {0}, rx[10];
	
	if (controller)
	{
		// Calculate new motor commands
		for (int i = 0; i < 4; ++i)
		{
			// Wheel speed in ticks/s, opposite of joystick direction
			int last_speed = encoder[i] - last_encoder[i];
			int error = ((int)wheel_command[i] * 4) - last_speed;
			integral[i] += error;
			
			int cmd = integral[i] / 40;
			
			// Stop oscillating
			if ((cmd < 0 && wheel_command[i] > 0) || (cmd > 0 && wheel_command[i] < 0))
			{
				cmd = 0;
				integral[i] = 0;
			}
			
			wheel_out[i] = cmd;
			
			// Convert from 2's-complement to sign-magnitude for FPGA
			uint8_t out;
			if (cmd < 0)
			{
				out = -cmd | 0x80;
			} else {
				out = cmd;
			}
			tx[i] = out;
		}
		tx[4] = 0x80 | (dribble << 3);
	} else {
		for (int i = 0; i < 4; ++i)
		{
			integral[i] = 0;
		}
	}
	
	// Save old encoder counts
	for (int i = 0; i < 4; ++i)
	{
		last_encoder[i] = encoder[i];
	}
	
	// Swap data with the FPGA
	spi_select(NPCS_FPGA);
	spi_xfer(0x01);
	for (int i = 0; i < sizeof(tx); ++i)
	{
		rx[i] = spi_xfer(tx[i]);
	}
	spi_deselect();
	
	// Unpack data from the FPGA's response
	encoder[0] = rx[0] | (rx[1] << 8);
	encoder[1] = rx[2] | (rx[3] << 8);
	encoder[2] = rx[4] | (rx[5] << 8);
	encoder[3] = rx[6] | (rx[7] << 8);
	motor_faults = rx[8];
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
	AT91C_BASE_PIOA->PIO_PPUER = RADIO_INT | FLASH_NCS | MISO | ID0 | ID1 | ID2 | ID3 | DP0 | DP1 | DP2;
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
	
	// Main loop
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
				LED_OFF(LED_RG);
				LED_TOGGLE(LED_RR);
				radio_command(SIDLE);
				radio_command(SFRX);
				radio_command(SRX);
			}
			
			//FIXME - Clean this up
			
			// Check for radiio packets
			if (radio_gdo2())
			{
				forward_packet_received();
			}
			//int have_forward = radio_gdo2() && forward_packet_received();
			
			// Run robot operations
			if (controller)
			{
				controller();
			} else {
				for (int i = 0; i < 4; ++i)
				{
					wheel_command[i] = 0;
				}
				dribble = 0;
			}
		}
		
		// Periodic activities: motor and ADC updates
		if ((current_time - update_time) >= 5)
		{
			adc_update();
			
			// Check things that depend on ADC results
			power_update();
			update_ball_sensor();
			
			// Send commands to and read status from the FPGA
			update_fpga();
			
			update_time = current_time;
		}
		
		// Keep power failure music playing continuously
		power_fail_music();
	}
}

#endif
