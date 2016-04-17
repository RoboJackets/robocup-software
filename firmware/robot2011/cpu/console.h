#pragma once

#include <stdint.h>

#define MAX_COMMAND_SIZE 64
#define MAX_COMMAND_ARGS 8

typedef struct {
    const char* name;
    void (*handler)(int argc, const char* argv[], void* arg);
    void* arg;
} command_t;

// Defined in main.c
extern const command_t commands[];

extern unsigned char usb_rx_buffer[64];
extern volatile int usb_rx_len;

void usb_rx_start();

void putchar_raw(uint8_t ch);
void flush_stdout();

void console_init(void);

// Handles command input from USB.
// Returns 1 to continue running robot operations or 0 to stop (user break).
int console_run(void);

void subcommand(int argc, const char* argv[], void* arg);

// Parses a 32-bit unsigned integer from a string.
// If the string starts with "0x", it is interpreted as hexadecimal.
// Otherwise, it is interpreted as decimal.
uint32_t parse_uint32(const char* str);

int parse_int(const char* str);

void check_usb_connection(void);

// Returns nonzero if the USB console is usable (for printf, etc.)
int usb_is_connected(void);
