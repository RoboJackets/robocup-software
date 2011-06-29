#include "radio_protocol.h"
#include "radio.h"
#include "timer.h"
#include "status.h"
#include "fpga.h"
#include "ball_sense.h"

#include <string.h>

// Last forward packet
uint8_t forward_packet[Forward_Size];

// Last reverse packet
uint8_t reverse_packet[Reverse_Size];

int wheel_command[4];
int dribble_command;
int kick_command;

uint32_t rx_lost_time;

//FIXME - The protocol needs to be totally redesigned

int handle_forward_packet()
{
	if (radio_rx_len != Forward_Size)
	{
		radio_command(SFRX);
		radio_command(SRX);
		return 0;
	}
	
	memcpy(forward_packet, radio_rx_buf, Forward_Size);
	
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
				// Convert from seven bits to nine bits (signed)
				int8_t byte = forward_packet[offset + i];
				wheel_command[i] = byte;
// 				wheel_command[i] = (byte << 2) | ((byte >> 5) & 3); //FIXME - scale for dumb control
			}
			
			// Convert the dribbler speed from the top four bits in a byte to nine bits
			uint8_t four_bits = forward_packet[offset + 4] & 0xf0;
			dribble_command = (four_bits << 1) | (four_bits >> 3) | (four_bits >> 7);
			
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
			reverse_packet[6 + i] = encoder_delta[i];
			reverse_packet[10] |= (encoder_delta[i] & 0x300) >> (8 - i * 2);
		}
		
		radio_transmit(reverse_packet, sizeof(reverse_packet));
	} else {
		// Get ready to receive another forward packet
		radio_command(SRX);
	}
	
	return 1;
}
