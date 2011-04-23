#pragma once

#include <stdint.h>

#include "spi.h"
#include "cc1101.h"

#define Forward_Size    31
#define Reverse_Size    11

static inline void radio_select()
{
	spi_select(NPCS_RADIO);
}

#define radio_deselect spi_deselect

static inline int radio_gdo2()
{
    return AT91C_BASE_PIOA->PIO_PDSR & RADIO_INT;
}

int radio_init();
void radio_configure();
uint8_t radio_command(uint8_t cmd);
uint8_t radio_read(uint8_t addr);
uint8_t radio_write(uint8_t addr, uint8_t value);

// Disables the RX timeout timer
void radio_timeout_disable();

// Resets and enables the RX timeout timer
void radio_timeout_enable();
