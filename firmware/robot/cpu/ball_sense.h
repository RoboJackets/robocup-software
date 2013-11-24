#pragma once

// Ball sensor result:
// Nonzero if we are confident that we have the ball.
// Zero if we don't have the ball or the ball sensor is suspect.
extern int have_ball;

// Most recent detector readings for LED on and off.
// The minimum value is zero (no light or broken detector wires).
// The maximum value is 0x3ff (very bright light or shorted detector wires).
extern int ball_sense_light, ball_sense_dark;

void update_ball_sensor(void);
