// FIXME - Use hall counters instead so this works with the dribbler and 2008
// bases
//	Does the dribbler need different parameters?

#include <stdio.h>
#include <stdlib.h>

#include "stall.h"
#include "fpga.h"
#include "status.h"

// Coefficient for speed
static const int Stall_kSpeed = 80;

// Coefficient for command
static const int Stall_kCommand = 1;

// If the counter is above the threshold, the motor has stalled
static const int Stall_Threshold = MOTOR_MAX * 200;

// The counter decays by this constant amount each cycle
static const int Stall_Decay = 80;

// Commands below this limit do not contribute to the counter
static const int Stall_Deadband = 60;

uint8_t motor_stall;
int stall_counter[5];

void stall_init(void) {
    motor_stall = 0;
    for (int i = 0; i < 5; ++i) {
        stall_counter[i] = 0;
    }
}

void stall_update(void) {
    for (int i = 0; i < 5; ++i) {
        int speed = abs(hall_delta[i]);
        int command = abs(motor_out[i]);
        if (command < Stall_Deadband) {
            command = 0;
        }

        stall_counter[i] +=
            command * Stall_kCommand - speed * Stall_kSpeed - Stall_Decay;
        if (stall_counter[i] < 0) {
            stall_counter[i] = 0;
        }

        if (stall_counter[i] >= Stall_Threshold) {
            // Prevent overflow and let the fail command reset the flag
            stall_counter[i] = 0;

            // Mark this motor as stalled
            motor_stall |= 1 << i;
        }
    }
}
