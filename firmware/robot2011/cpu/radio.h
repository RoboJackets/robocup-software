#pragma once

#include <stdint.h>

#include "spi.h"
#include "cc1101.h"

extern uint8_t radio_rx_len;
extern uint8_t radio_rx_buf[];
extern int_fast8_t last_rssi;

// True if we are waiting for a transmission to finish
extern volatile int radio_in_tx;

static inline void radio_select() { spi_select(NPCS_RADIO); }

#define radio_deselect spi_deselect

static inline int radio_gdo2() { return AT91C_BASE_PIOA->PIO_PDSR & RADIO_INT; }

int radio_init();
void radio_configure();
uint8_t radio_command(uint8_t cmd);
uint8_t radio_read(uint8_t addr);
uint8_t radio_write(uint8_t addr, uint8_t value);

// Sets the radio channel number (CHANNR register).
// Only certain values are permissible, so check the radio configuraion and
// CC1101 datasheet.
void radio_channel(int n);

void radio_transmit(uint8_t* buf, int len);

// Checks the radio for received data.
// Returns nonzero and sets radio_rx_len and radio_rx_buf if a packet was
// successfully received.
int radio_poll();
