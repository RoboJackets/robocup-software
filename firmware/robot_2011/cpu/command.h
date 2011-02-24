#pragma once

#include <stdint.h>

typedef struct
{
	const char *name;
	void (*handler)(int argc, const char *argv[], void *arg);
	void *arg;
} command_t;

// Defined in main.c
extern const command_t commands[];

extern unsigned char usb_rx_buffer[64];
extern volatile int usb_rx_len;

void usb_rx_start();

void putchar_raw(uint8_t ch);
void flush_stdout();

void command_init(void);

// Handles command input from USB.
// Returns 1 to continue running robot operations or 0 to stop (user break).
int command_run(void);

void subcommand(int argc, const char *argv[], void *arg);

// arg points to a write_int_t
void cmd_write_int(int argc, const char *argv[], void *arg);

uint32_t parse_uint32(const char *str);
