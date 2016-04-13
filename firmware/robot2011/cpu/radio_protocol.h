#pragma once

#include <stdint.h>
#include <board.h>

#define reply_timer_done() (AT91C_BASE_TC0->TC_SR & AT91C_TC_CPCS)

// Speeds given in the most recent forward packet
extern int cmd_body_x;  // -511 to 511
extern int cmd_body_y;
extern int cmd_body_w;
extern int dribble_command;  // -511 to 511
extern int kick_command;     // 0 to 255
extern int kick_immediate;
extern int accel_limit;
extern int decel_limit;
extern int sing;
extern int anthem;

extern uint32_t rx_lost_time;

// To send a radio reply, put it in here and call reply_timer_start().
extern uint8_t reply_buf[64];
extern uint8_t reply_len;

void reply_timer_init(void);

// Starts the reply timer.  Poll reply_timer_done() to see when it expires.
// <time> is the time to wait, 1500 counts/millisecond.
void reply_timer_start(uint16_t time);

// Sends a reply packet if the reply timer has expired
void radio_reply(void);

int handle_forward_packet(void);
