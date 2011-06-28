//FIXME - Use hall counters instead so this works with the dribbler and 2008 bases
//	Does the dribbler need different parameters?

#include <stdio.h>
#include <stdlib.h>

#include "stall.h"
#include "fpga.h"
#include "status.h"

// Stall detection uses a rough estimate of the temperature of the motor windings.
// For each motor there is a stall counter.  The counter increases with motor command
// (representing applied voltage), decreases with speed (representing back-EMF, airflow, and
// energy delivered to the mechanical load), and decreases at a constant rate
// (representing loss of heat to the environment).
//
// When the stall counter for a motor reaches a threshold, that motor is considered to have
// stalled until the stall flag is manually reset (or the board is reset).
//
// There is a possible wiring failure mode that is very important to detect:
// If a single hall effect sensor phase is disconnected, the apparent hall state may be valid but
// incorrect, resulting in the motor being "electrically stalled".  This will lead to the
// motor windings burning out if speed control tries to drive the motor at high power.

// Coefficient for speed
static const int Stall_kSpeed = 80;

// Coefficient for command
static const int Stall_kCommand = 1;

// If the counter is above the threshold, the motor has stalled
static const int Stall_Threshold = 127*200;

// The counter decays by this constant amount each cycle
static const int Stall_Decay = 80;

// Commands below this limit do not contribute to the counter
static const int Stall_Deadband = 60;

uint8_t motor_stall;
int stall_counter[5];

void stall_init(void)
{
	motor_stall = 0;
	for (int i = 0; i < 5; ++i)
	{
		stall_counter[i] = 0;
	}
}

void stall_update(void)
{
	for (int i = 0; i < 5; ++i)
	{
		// Don't look for stalls on motors that we already know can't be driven
		if (!(motor_faults & (1 << i)))
		{
			int speed = abs(hall_delta[i]);
			int command = abs(motor_out[i]);
			if (command < Stall_Deadband)
			{
				command = 0;
			}
			
			stall_counter[i] += command * Stall_kCommand - speed * Stall_kSpeed - Stall_Decay;
			if (stall_counter[i] < 0)
			{
				stall_counter[i] = 0;
			}
			
			if (stall_counter[i] >= Stall_Threshold)
			{
				// Prevent overflow and let the fail command reset the flag
				stall_counter[i] = 0;
				
				// Mark this motor as stalled
				motor_stall |= 1 << i;
			}
		}
	}
}
