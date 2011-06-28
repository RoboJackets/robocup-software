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

int_fast8_t wheel_command[4];
int_fast8_t dribble_command;
uint_fast8_t kick_command;

uint32_t rx_lost_time;

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
			reverse_packet[6 + i] = encoder_count[i];
			reverse_packet[10] |= (encoder_count[i] & 0x300) >> (8 - i * 2);
		}
		
		radio_transmit(reverse_packet, sizeof(reverse_packet));
	} else {
		// Get ready to receive another forward packet
		radio_command(SRX);
	}
	
	return 1;
}
