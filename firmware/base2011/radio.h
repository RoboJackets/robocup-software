#ifndef _RADIO_H_
#define _RADIO_H_

#include <stdint.h>

#include "bits.h"

#define Forward_Size 31

static inline void radio_select() { clear_bit(PORTD, 4); }

static inline void radio_deselect() { set_bit(PORTD, 4); }

uint8_t spi_write(uint8_t data);

void radio_init();
uint8_t radio_command(uint8_t cmd);
uint8_t radio_read(uint8_t addr);
uint8_t radio_write(uint8_t addr, uint8_t value);

// Disables the RX timeout timer
void radio_timeout_disable();

// Resets and enables the RX timeout timer
void radio_timeout_enable();

#endif  // _RADIO_H_
