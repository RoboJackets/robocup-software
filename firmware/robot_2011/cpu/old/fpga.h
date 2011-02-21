#ifndef _FPGA_H_
#define _FPGA_H_

#include <stdint.h>

enum
{
    IO_WR           = 1 << 6,
    IO_RD           = 1 << 7,
    IO_ALE          = 1 << 13,
    IO_DATA         = 0xff << 15,
    IO_DATA_SHIFT   = 15,
};

// Registers
enum
{
    FPGA_Fault          = 0x08,
    FPGA_Radio_Data     = 0x10,
    FPGA_Interrupt      = 0x11,
    FPGA_Radio_Config   = 0x12,
    FPGA_Switches       = 0x13,
    FPGA_Kick           = 0x20,
    FPGA_Kicker_Status  = 0x21,
    FPGA_Encoder_Latch  = 0x30, // Write only
    FPGA_Encoder_Low	= 0x30, // First encoder low byte
    FPGA_Encoder_High	= 0x31, // First encoder high byte
    FPGA_Test_Fixed     = 0xfe,
    FPGA_Test_RW        = 0xff
};

// 0x12: Radio config/status bits
enum
{
    FPGA_Radio_Select   = 1,
    FPGA_Radio_GDO0     = 2,    // Read only
    FPGA_Radio_GDO2     = 4,    // Read only
    FPGA_Radio_Idle     = 8     // Read only
};

// 0x15: Kicker status
enum
{
    FPGA_Kicker_Done    = 1,    // Read only
    FPGA_Kicker_Lockout = 2,    // Read only
    FPGA_Kicker_Charge  = 4,    // Read only
    FPGA_Kicker_Enable  = 8,
    FPGA_Kicker_Override = 16   // Read only
};

// Interrupt bits
enum
{
    FPGA_Interrupt_GDO2 = 2,
    FPGA_Interrupt_GDO0 = 1
};

void fpga_init(void);
uint8_t fpga_read(uint8_t addr);
void fpga_write(uint8_t addr, uint8_t data);

#endif /* _FPGA_H_ */

