#ifndef _RADIO_H_
#define _RADIO_H_

#include <stdint.h>

#include "fpga.h"

#define Forward_Size    28
#define Reverse_Size    11

static inline void radio_select()
{
    fpga_write(FPGA_Radio_Config, FPGA_Radio_Select);
}

static inline void radio_deselect()
{
    fpga_write(FPGA_Radio_Config, 0);
}

static inline uint8_t radio_gdo2()
{
    return fpga_read(FPGA_Radio_Config) & FPGA_Radio_GDO2;
}

static inline uint8_t radio_int_gdo2()
{
    return fpga_read(FPGA_Interrupt) & FPGA_Interrupt_GDO2;
}

static inline void radio_clear_gdo2()
{
    fpga_write(FPGA_Interrupt, FPGA_Interrupt_GDO2);
}

uint8_t spi_write(uint8_t data);

void radio_init();
uint8_t radio_command(uint8_t cmd);
uint8_t radio_read(uint8_t addr);
uint8_t radio_write(uint8_t addr, uint8_t value);

// Disables the RX timeout timer
void radio_timeout_disable();

// Resets and enables the RX timeout timer
void radio_timeout_enable();

#endif // _RADIO_H_
