#include "lpc2103.h"
#include "fpga.h"
#include "pins.h"

volatile int bad_fpga_write = 0xdddddddd;
volatile int bad_fpga_read = 0xeeeeeeee;

void fpga_init()
{
    // Wait for the FPGA to configure
    while (fpga_read(FPGA_Test_Fixed) != 0xc9);

    // Test FPGA bus
    bad_fpga_write = 0;
    fpga_write(FPGA_Test_RW, 0x00);
    bad_fpga_read = fpga_read(FPGA_Test_RW);
    while (bad_fpga_read != 0x00);
    for (int i = 0; i < 8; ++i)
    {
        bad_fpga_write = 1 << i;
        
        fpga_write(FPGA_Test_RW, bad_fpga_write);
        bad_fpga_read = fpga_read(FPGA_Test_RW);
        while (bad_fpga_read != bad_fpga_write);
        
        bad_fpga_write = bad_fpga_write ^ 0xff;
        fpga_write(FPGA_Test_RW, bad_fpga_write);
        bad_fpga_read = fpga_read(FPGA_Test_RW);
        while (bad_fpga_read != bad_fpga_write);
    }
}

uint8_t fpga_read(uint8_t addr)
{
    // Address
    FIOCLR = IO_DATA;
    FIOSET = addr << IO_DATA_SHIFT;
    FIODIR |= IO_DATA;
    
    // ALE
    FIOCLR = IO_ALE;
    FIOSET = IO_ALE;
    
    // Data
    FIODIR &= ~IO_DATA;
    FIOCLR = IO_RD;
    uint8_t value = FIOPIN >> IO_DATA_SHIFT;
    FIOSET = IO_RD;
    
    return value;
}

void fpga_write(uint8_t addr, uint8_t data)
{
    // Address
    FIOCLR = IO_DATA;
    FIOSET = addr << IO_DATA_SHIFT;
    FIODIR |= IO_DATA;
    
    // ALE
    FIOCLR = IO_ALE;
    FIOSET = IO_ALE;
    
    // Data
    FIOCLR = IO_DATA;
    FIOSET = data << IO_DATA_SHIFT;
    FIOCLR = IO_WR;
    FIOSET = IO_WR;
    FIODIR &= ~IO_DATA;
}
