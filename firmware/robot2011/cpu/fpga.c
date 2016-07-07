#include <board.h>

#include "fpga.h"
#include "status.h"
#include "spi.h"
#include "timer.h"

// This must match LOGIC_VERSION in robocup.v
#define LOGIC_VERSION 4

int encoder_count[4];
int encoder_delta[4];

int hall_count[5];
int hall_delta[5];

int_fast8_t motor_out[5];
int drive_mode[5];

uint_fast8_t kick_strength;
uint_fast8_t use_chipper;
uint_fast8_t kicker_charge;

uint8_t kicker_status;
int kicker_voltage;

int fpga_init() {
    int ret;

    failures &= ~Fail_FPGA;

    // Disable SPI drivers so the FPGA can reconfigure
    spi_shutdown();

    // Release PROGB so the FPGA can start configuring
    AT91C_BASE_PIOA->PIO_SODR = MCU_PROGB;

    // Wait for the FPGA to start configuring
    delay_ms(5);
    if (!(AT91C_BASE_PIOA->PIO_PDSR & FLASH_NCS)) {
        // FLASH_NCS is low: the FPGA is reading
        //
        // Wait for the FPGA to finish configuring
        for (int i = 0; i < 100; ++i) {
            delay_ms(10);
            if (AT91C_BASE_PIOA->PIO_PDSR & FLASH_NCS) {
                // FLASH_NCS is high: the FPGA is done
                ret = 1;
                goto good;
            }
        }

        // The FPGA took too long to configure.
        // Configuration memory is probably empty/corrupt.
        // It's also possible that the board is on a bench power supply
        // with a low current limit and the supply voltage rose very slowly
        // due to inrush current to the motor driver capacitors.
        //
        // Shut down the FPGA, since the MCU needs to become the SPI master.
        AT91C_BASE_PIOA->PIO_CODR = MCU_PROGB;

        failures |= Fail_FPGA_Config;

        // Become the SPI master
        spi_init();

        return 0;
    } else {
        // FPGA did not start reading SPI flash - already configured?
        ret = 2;
    }
good:
    // Become the SPI master
    spi_init();

    // Verify interface version
    spi_select(NPCS_FPGA);
    uint8_t version = spi_xfer(0);
    spi_deselect();

    if (version != LOGIC_VERSION) {
        failures |= Fail_FPGA_Version;
    }

    return ret;
}

void fpga_read_status() {
    if (failures & Fail_FPGA) {
        return;
    }

    uint8_t rx[17];

    // Save old encoder counts
    int last_encoder[4];
    for (int i = 0; i < 4; ++i) {
        last_encoder[i] = encoder_count[i];
    }

    int last_hall[5];
    for (int i = 0; i < 5; ++i) {
        last_hall[i] = hall_count[i];
    }

    // Swap data with the FPGA
    spi_select(NPCS_FPGA);
    for (int i = 0; i < sizeof(rx); ++i) {
        rx[i] = spi_xfer(0);
    }
    spi_deselect();

    // Unpack data from the FPGA's response
    if (rx[0] != LOGIC_VERSION) {
        failures |= Fail_FPGA_Version;
    }
    encoder_count[0] = rx[1] | (rx[2] << 8);
    encoder_count[1] = rx[3] | (rx[4] << 8);
    encoder_count[2] = rx[5] | (rx[6] << 8);
    encoder_count[3] = rx[7] | (rx[8] << 8);
    current_motor_faults = rx[9];
    motor_faults |= current_motor_faults;
    kicker_status = rx[10];
    kicker_voltage = (int)rx[11] * 33 * 101 / 2550;
    hall_count[0] = rx[12];
    hall_count[1] = rx[13];
    hall_count[2] = rx[14];
    hall_count[3] = rx[15];
    hall_count[4] = rx[16];

    if (kicker_status & 0x40) {
        failures &= ~Fail_Kicker_I2C;
    } else if (!base2008) {
        failures |= Fail_Kicker_I2C;
    }

    for (int i = 0; i < 4; ++i) {
        encoder_delta[i] = (int16_t)(encoder_count[i] - last_encoder[i]);
    }

    for (int i = 0; i < 5; ++i) {
        hall_delta[i] = (int8_t)(hall_count[i] - last_hall[i]);

        if (base2008) {
            // The motor commands are reverse before being sent to the FPGA,
            // so flip the hall speeds to match.
            hall_delta[i] = -hall_delta[i];
        }
    }
}

void fpga_send_commands() {
    if (failures & Fail_FPGA) {
        return;
    }

    uint8_t tx[12] = {0};

    tx[0] = 0x01;  // Command: set motor speeds
    for (int i = 0; i < 5; ++i) {
        int_fast8_t cmd = motor_out[i];

        // 2008 bases use outside gears so wheels spin the other way
        if (base2008) {
            cmd = -cmd;
        }

        // Convert to sign-magnitude
        if (cmd < 0) {
            cmd = -cmd;
            tx[i * 2 + 1] = cmd;
            tx[i * 2 + 2] = (cmd >> 8) & 1;
            tx[i * 2 + 2] |= 2 | ((drive_mode[i] & 3) << 2);
        } else {
            tx[i * 2 + 1] = cmd;
            tx[i * 2 + 2] = (cmd >> 8) & 1;
            tx[i * 2 + 2] |= (drive_mode[i] & 3) << 2;
        }
    }
    if (kicker_charge) {
        tx[10] |= 0x80;
    }
    if (use_chipper) {
        tx[10] |= 0x40;  // Select chipper
    }
    tx[11] = kick_strength;

    // Swap data with the FPGA
    spi_select(NPCS_FPGA);
    for (int i = 0; i < sizeof(tx); ++i) {
        spi_xfer(tx[i]);
    }
    spi_deselect();
}
