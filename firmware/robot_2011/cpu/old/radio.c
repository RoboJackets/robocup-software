#include "radio.h"
#include "cc1101.h"
#include "timer.h"
#include "robot.h"
#include "pins.h"
#include "lpc2103.h"

uint8_t spi_write(uint8_t data)
{
    fpga_write(FPGA_Radio_Data, data);
    
    while (!(fpga_read(FPGA_Radio_Config) & FPGA_Radio_Idle))
    {
    }
    
    return fpga_read(FPGA_Radio_Data);
}

void fail(int flash)
{
    timer_t t = {flash, flash, flash_red, 0};
    timer_register(&t);
    
    while (1)
    {
    }
}

void radio_init()
{
    radio_clear_gdo2();

#if 0
    // Reset
    delay_ms(50);
    radio_command(SRES);
    delay_ms(50);
#endif

    // Waits for the radio's version byte to have the expected value.
    // This verifies that the radio crystal oscillator and SPI interface are working.
    uint8_t version;
    do
    {
        version = radio_read(VERSION);
    } while (version != 0x04);
    
#if 0
    // Test GDO0 high
    radio_write(IOCFG0, 0x2f | GDOx_INVERT);
    if (!bit_is_set(PIND, 1))
    {
        fail(0);
    }
    
    // Test GDO0 low
    radio_write(IOCFG0, 0x2f);
    if (bit_is_set(PIND, 1))
    {
        fail(1);
    }
#endif
    
    // Test GDO2 high
    radio_write(IOCFG2, 0x2f | GDOx_INVERT);
    if (!radio_gdo2())
    {
        fail(250);
    }
    
    // Test GDO2 low
    radio_write(IOCFG2, 0x2f);
    if (radio_gdo2())
    {
        fail(500);
    }
}

uint8_t radio_command(uint8_t cmd)
{
    radio_select();
    uint8_t status = spi_write(cmd);
    radio_deselect();
    
    return status;
}

uint8_t radio_read(uint8_t addr)
{
    radio_select();
    spi_write(addr | CC_READ);
    uint8_t value = spi_write(SNOP);
    radio_deselect();
    
    return value;
}

uint8_t radio_write(uint8_t addr, uint8_t value)
{
    radio_select();
    uint8_t status = spi_write(addr);
    spi_write(value);
    radio_deselect();
    
    return status;
}
