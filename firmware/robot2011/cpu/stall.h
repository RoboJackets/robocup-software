#pragma once

#include <stdint.h>

/**
 * Stall detection overview
 *
 * Stall detection uses a rough estimate of the temperature of the motor
 * windings.
 * For each motor there is a stall counter.  The counter increases with motor
 * command
 * (representing applied voltage), decreases with speed (representing back-EMF,
 * airflow, and
 * energy delivered to the mechanical load), and decreases at a constant rate
 * (representing loss of heat to the environment).
 *
 * When the stall counter for a motor reaches a threshold, that motor is
 * considered to have
 * stalled until the stall flag is manually reset (or the board is reset).
 *
 * There is a possible wiring failure mode that is very important to detect:
 * If a single hall effect sensor phase is disconnected, the apparent hall state
 * may be valid but
 * incorrect, resulting in the motor being "electrically stalled".  This will
 * lead to the
 * motor windings burning out if speed control tries to drive the motor at high
 * power.
 */

// Bits 0-4 indicate a stall on the corresponding motor
// FIXME - The dribbler has no stall detection.  Need to put hall counters back
// in the FPGA.
extern uint8_t motor_stall;

extern int stall_counter[5];

void stall_init(void);
void stall_update(void);
