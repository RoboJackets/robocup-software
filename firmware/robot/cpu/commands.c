#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <board.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <math.h>

#include "timer.h"
#include "console.h"
#include "sound.h"
#include "write.h"
#include "spi.h"
#include "tools/reflash.h"
#include "cc1101.h"
#include "status.h"
#include "radio.h"
#include "control.h"
#include "power.h"
#include "main.h"
#include "ball_sense.h"
#include "adc.h"
#include "fpga.h"
#include "stall.h"
#include "encoder_monitor.h"
#include "kicker.h"
#include "radio_protocol.h"
#include "imu.h"
#include "invensense/imuMlsl.h"
#include "invensense/imuFIFO.h"
#include "invensense/imuSetup.h"

static void cmd_help(int argc, const char* argv[], void* arg) {
    printf("Commands:\n");
    for (int i = 0; commands[i].name; ++i) {
        printf("  %s\n", commands[i].name);
    }
}

static void cmd_reflash(int argc, const char* argv[], void* arg) {
    unsigned int len;

    if (argc != 1) {
        printf("Use the host-side reflash script\n");
        return;
    }

    music_stop();

    len = parse_uint32(argv[0]);

    // Set FMR for timing and auto-erase
    AT91C_BASE_MC->MC_FMR = 0x00340100;

    printf("GO %08x\n", len);

    // Disable interrupts
    AT91C_BASE_AIC->AIC_IDCR = ~0;

    // After this point, we're committed to running the reflasher.
    // RAM contents are forfeit.  Only the stack should be considered usable.

    // FIXME - Copy reflash to its final location in SRAM.
    //  For now, it's always there thanks to the relocate section, but this
    //  wastes memory.
    //  This would be unnecessary if we are running from SRAM to begin with.

    // This never returns
    reflash_main(len);
}

// FIXME - This can be smaller
const char* const motor_names[5] = {"BL", "FL", "FR", "BR", "DR"};

// Prints a power supply measurement with a label
static void print_supply(const char* label, int raw) {
    int supply_mv = raw * VBATT_NUM / VBATT_DIV;
    printf("%s: %d.%03dV\n", label, supply_mv / 1000, supply_mv % 1000);
}

static void print_motor_bits(uint8_t bits) {
    if (bits) {
        for (int i = 0; i < 5; ++i) {
            if (bits & (1 << i)) {
                putchar(' ');
                printf(motor_names[i]);
            }
        }
    } else {
        printf(" None");
    }
}

static void cmd_status(int argc, const char* argv[], void* arg) {
    if (base2008) {
        printf("2008");
    } else {
        printf("2011");
    }
    printf(" mechanical base\n");

    if (controller) {
        printf("Controller: %s\n", controller->name);
    } else {
        printf("No controller\n");
    }

    printf("Robot ID %d\n", robot_id);
    printf("Reset type %x\n", (AT91C_BASE_SYS->RSTC_RSR >> 8) & 7);
    printf("Current time: %d\n", current_time);

    printf("Failures: 0x%08x", failures);
    if (failures & Fail_FPGA) {
        printf(" FPGA");
    }
    if (failures & Fail_Radio) {
        printf(" Radio");
    }
    if (failures & Fail_Power) {
        printf(" Power");
    }
    if (failures & Fail_Ball) {
        printf(" BallSense");
    }
    if (failures & Fail_IMU) {
        printf(" IMU");
    }
    if (failures & Fail_Kicker) {
        printf(" Kicker");
    }
    putchar('\n');

    printf("Power:\n");
    print_supply("  Now", supply_raw);
    print_supply("  Min", supply_min);
    print_supply("  Max", supply_max);

    printf("Motor faults: 0x%02x", motor_faults);
    print_motor_bits(motor_faults);
    putchar('\n');

    printf("Encoder faults: 0x%02x", encoder_faults);
    print_motor_bits(encoder_faults);
    putchar('\n');

    printf("Motor stalls: 0x%02x", motor_stall);
    print_motor_bits(motor_stall);
    putchar('\n');

    printf("Command: %4d %4d %4d %4d\n", cmd_body_x, cmd_body_y, cmd_body_w,
           dribble_command);

    printf("Motor out:");
    for (int i = 0; i < 5; ++i) {
        printf(" %6d", motor_out[i]);
    }
    printf("\n");

    printf("     Mode:");
    for (int i = 0; i < 5; ++i) {
        printf(" %6d", drive_mode[i]);
    }
    printf("\n");

    printf("    Stall:");
    for (int i = 0; i < 5; ++i) {
        printf(" %6d", stall_counter[i]);
    }
    printf("\n");

    printf("     Hall:");
    for (int i = 0; i < 5; ++i) {
        printf(" 0x%04x", hall_count[i]);
    }
    printf("\n");

    printf("    dHall:");
    for (int i = 0; i < 5; ++i) {
        printf(" 0x%04x", hall_delta[i]);
    }
    printf("\n");

    printf(" Encoders:");
    for (int i = 0; i < 4; ++i) {
        printf(" 0x%04x", encoder_count[i]);
    }
    printf("\n");
    printf("    Delta:");
    for (int i = 0; i < 4; ++i) {
        printf(" %6d", encoder_delta[i]);
    }
    printf("\n");
    printf("   Output:");
    for (int i = 0; i < 5; ++i) {
        printf(" %6d", motor_out[i]);
    }
    printf("\n");

    printf("Ball sensor:\n");
    printf("  Light: 0x%03x\n", ball_sense_light);
    printf("  Dark:  0x%03x\n", ball_sense_dark);
    printf("  Delta: %5d\n", ball_sense_light - ball_sense_dark);

    printf("Kicker: status 0x%02x %3dV\n", kicker_status, kicker_voltage);
    if (!(kicker_status & 0x40)) {
        printf("Voltage ADC failed\n");
    }

    printf("Encoder monitor:\n");
    for (int i = 0; i < 5; ++i) {
        printf("%d %2d %3d %4d\n", i, em_err_hall[i], em_err_enc[i],
               em_err_out[i]);
    }
}

static void cmd_timers(int argc, const char* argv[], void* arg) {
    if (first_timer) {
        printf("Timer    time       period\n");
        //      0x01234567 0x01234567 0x01234567
        for (Timer* t = first_timer; t; t = t->next) {
            printf("%p 0x%08x 0x%08x\n", t, t->time, t->period);
        }
    } else {
        printf("(none)\n");
    }
}

static void cmd_print_uint32(int argc, const char* argv[], void* arg) {
    printf("0x%08x\n", *(unsigned int*)arg);
}

static void cmd_spi_test(int argc, const char* argv[], void* arg) {
    spi_select(NPCS_FLASH);

    spi_xfer(0xab);
    spi_xfer(0);
    spi_xfer(0);
    spi_xfer(0);
    printf("Signature: 0x%02x\n", spi_xfer(0));
    spi_deselect();

    spi_xfer(0x05);
    printf("Status:    0x%02x\n", spi_xfer(0));
    spi_deselect();
}

static int spi_wait(int max) {
    spi_xfer(0x05);
    for (unsigned int start_time = current_time;
         (current_time - start_time) < max;) {
        // Reset the watchdog timer
        AT91C_BASE_WDTC->WDTC_WDCR = 0xa5000001;

        uint8_t status = spi_xfer(0);
        if (!(status & 1)) {
            spi_deselect();
            return 1;
        }
    }
    printf("*** Timeout!\n");
    spi_deselect();
    return 0;
}

static void cmd_spi_erase(int argc, const char* argv[], void* arg) {
    spi_select(NPCS_FLASH);

    // Bulk erase
    // WREN
    spi_xfer(0x06);
    spi_deselect();

    // BE
    spi_xfer(0xc7);
    spi_deselect();

    // Wait for completion
    spi_wait(4000);
}

static void cmd_spi_write(int argc, const char* argv[], void* arg) {
    uint32_t addr, len;
    int rx_pos;

    if (argc != 2) {
        printf("spi_write <address> <length>\n");
        return;
    }

    addr = parse_uint32(argv[0]);
    len = parse_uint32(argv[1]);

    usb_rx_len = 0;
    rx_pos = 0;

    // Program one page at a time
    spi_select(NPCS_FLASH);
    while (len) {
        // WREN
        spi_xfer(0x06);
        spi_deselect();

        // PP
        spi_xfer(0x02);
        spi_xfer(addr >> 16);
        spi_xfer(addr >> 8);
        spi_xfer(addr);

        // Program up to the end of the page or the end of the data
        do {
            // Wait for more data if necessary
            if (rx_pos == usb_rx_len) {
                usb_rx_start();
                while (!usb_rx_len)
                    ;
                rx_pos = 0;
            }

            spi_xfer(usb_rx_buffer[rx_pos++]);
            ++addr;
            --len;
        } while ((addr & 0x7f) && len);

        spi_deselect();

        if (!spi_wait(5)) {
            return;
        }
    }
    // FIXME - I don't like how this fucntion handles input.
    //  There should be an input buffer.  Currently we depend on there not being
    //  extra input.
    usb_rx_len = 0;
    printf("OK\n");
}

static void cmd_spi_read(int argc, const char* argv[], void* arg) {
    uint32_t addr, len;

    if (argc != 2) {
        printf("spi_read <address> <length>\n");
        return;
    }

    addr = parse_uint32(argv[0]);
    len = parse_uint32(argv[1]);

    spi_select(NPCS_FLASH);
    spi_xfer(0x03);
    spi_xfer(addr >> 16);
    spi_xfer(addr >> 8);
    spi_xfer(addr);

    for (; len; --len) {
        putchar_raw(spi_xfer(0));
    }
    flush_stdout();

    spi_deselect();
}

static void cmd_fpga_test(int argc, const char* argv[], void* arg) {
    spi_select(NPCS_FPGA);

    for (int i = 0; i < argc; ++i) {
        uint8_t byte = parse_uint32(argv[i]);
        printf("0x%02x\n", spi_xfer(byte));
    }

    spi_deselect();
}

static void cmd_fpga_on(int argc, const char* argv[], void* arg) {
    switch (fpga_init()) {
        case 0:
            music_start(song_failure);
            printf("*** Failed\n");
            break;

        case 1:
            music_start(song_startup);
            printf("Configured\n");
            break;

        case 2:
            printf("Already configured\n");
            break;
    }
}

static void cmd_fpga_reset(int argc, const char* argv[], void* arg) {
    AT91C_BASE_PIOA->PIO_CODR = MCU_PROGB;
    delay_ms(1);
    cmd_fpga_on(0, 0, 0);
}

static void cmd_read_word(int argc, const char* argv[], void* arg) {
    if (argc != 1) {
        printf("rw <addr>\n");
        return;
    }

    uint32_t addr = parse_uint32(argv[0]);
    printf("0x%08x\n", *(unsigned int*)addr);
}

static void cmd_write_word(int argc, const char* argv[], void* arg) {
    if (argc != 2) {
        printf("ww <addr> <value>\n");
        return;
    }

    uint32_t addr = parse_uint32(argv[0]);
    uint32_t value = parse_uint32(argv[1]);
    *(uint32_t*)addr = value;
}

static void cmd_radio_start(int argc, const char* argv[], void* arg) {
    radio_command(SIDLE);
    radio_command(SFRX);
    radio_command(SRX);
}

static void cmd_stfu(int argc, const char* argv[], void* arg) {
    debug_update = 0;

    if (argc) {
        power_music_disable = parse_uint32(argv[0]);
    } else {
        power_music_disable = 1;
    }

    // Stop any music that may be playing
    music_stop();
}

static const note_t* const test_songs[] = {
    song_startup,    song_failure, song_overvoltage, song_undervoltage,
    song_fuse_blown, song_victory, song_still_alive};
#define NUM_TEST_SONGS (sizeof(test_songs) / sizeof(test_songs[0]))

static void cmd_music(int argc, const char* argv[], void* arg) {
    int n;
    if (argc != 1 || (n = parse_uint32(argv[0])) >= NUM_TEST_SONGS) {
        printf("music <0-%d>\n", (int)NUM_TEST_SONGS - 1);
        return;
    }

    music_start(test_songs[n]);
}

static void cmd_tone(int argc, const char* argv[], void* arg) {
    AT91C_BASE_PWMC->PWMC_DIS = 1 << 3;
    if (argc == 1) {
        int period = PERIOD(parse_uint32(argv[0]));
        AT91C_BASE_PWMC->PWMC_CH[3].PWMC_CPRDR = period;
        AT91C_BASE_PWMC->PWMC_CH[3].PWMC_CDTYR = period / 2;
        AT91C_BASE_PWMC->PWMC_ENA = 1 << 3;
        printf("%d\n", period);
    }
}

static void cmd_fail(int argc, const char* argv[], void* arg) {
    if (argc < 1 || argc > 4) {
        printf(
            "fail <flags> [<motor faults>] [<motor stalls>] "
            "[<encoder_faults>]\n");
        return;
    }

    failures = parse_uint32(argv[0]);
    if (argc > 1) {
        motor_faults = current_motor_faults | parse_uint32(argv[1]);
    }
    if (argc > 2) {
        motor_stall = parse_uint32(argv[2]);
    }
    if (argc > 3) {
        encoder_faults = parse_uint32(argv[3]);
    }
}

static void cmd_adc(int argc, const char* argv[], void* arg) {
    printf("0x%08x\n", AT91C_BASE_ADC->ADC_CHSR);
    printf("0x%08x\n", AT91C_BASE_ADC->ADC_MR);
    printf("0x%08x\n", AT91C_BASE_ADC->ADC_SR);
    for (int i = 0; i < 8; ++i) {
        if (i == 3) {
            // PA20 is not assigned to AD3
            continue;
        }
        printf("%d: 0x%03x\n", i, adc[i]);
    }

    if (argc) {
        AT91C_BASE_ADC->ADC_CR = AT91C_ADC_START;
    }
}

static void cmd_run(int argc, const char* argv[], void* arg) {
    if (argc) {
        // Start the named controller
        for (int i = 0; controllers[i].name; ++i) {
            if (!strcmp(argv[0], controllers[i].name)) {
                controller = &controllers[i];
                if (controller->init) {
                    controller->init(argc - 1, argv + 1);
                }
                return;
            }
        }

        printf("Not found\n");
    } else {
        // Start the default controller
        controller = default_controller;
        if (controller && controller->init) {
            controller->init(0, 0);
        }
    }
}

static void cmd_i2c_read(int argc, const char* argv[], void* arg) {
    if (argc != 2) {
        printf("i2c_read <device> <reg>\n");
        return;
    }

#if 0
	AT91C_BASE_TWI->TWI_MMR = (parse_uint32(argv[0]) << 16) | AT91C_TWI_MREAD | AT91C_TWI_IADRSZ_1_BYTE;
	AT91C_BASE_TWI->TWI_IADR = parse_uint32(argv[1]);
	AT91C_BASE_TWI->TWI_CR = AT91C_TWI_START | AT91C_TWI_STOP;
	
	//FIXME - Can't spin on SR (see errata chapter 40.18.7.4).  Must use interrupt.
	uint32_t status;
	while (1)
	{
		status = AT91C_BASE_TWI->TWI_SR;
		if (status & (AT91C_TWI_RXRDY | AT91C_TWI_NACK))
		{
			break;
		}
	}
	uint8_t result = AT91C_BASE_TWI->TWI_RHR;
	while (!(AT91C_BASE_TWI->TWI_SR & AT91C_TWI_TXCOMP));
	if (status & AT91C_TWI_NACK)
	{
		printf("NACK\n");
	} else {
		printf("0x%02x\n", result);
	}
#else
    uint8_t result = 0;
    if (MLSLSerialReadBurst(parse_uint32(argv[0]), parse_uint32(argv[1]), 1,
                            &result) != ML_SUCCESS) {
        printf("NACK\n");
    } else {
        printf("0x%02x\n", result);
    }
#endif
}

static void cmd_i2c_write(int argc, const char* argv[], void* arg) {
    if (argc != 3) {
        printf("i2c_read <device> <reg> <data>\n");
        return;
    }

    // 	if (MLSLSerialWriteSingle(parse_uint32(argv[0]), parse_uint32(argv[1]),
    // parse_uint32(argv[2])) != ML_SUCCESS)
    uint8_t data = parse_uint32(argv[2]);
    if (MLSLSerialWriteBurst(parse_uint32(argv[0]), parse_uint32(argv[1]), 1,
                             &data) != ML_SUCCESS) {
        printf("NACK\n");
    } else {
        printf("OK\n");
    }
}

void cmd_write_uint(int argc, const char* argv[], void* arg) {
    const write_uint_t* w = (const write_uint_t*)arg;
    if (argc >= 1) {
        *w->ptr = parse_int(argv[0]);
    } else {
        *w->ptr = w->value;
    }
}

void cmd_read(int argc, const char* argv[], void* arg) {
    if (argc != 2) {
        printf("read <address> <len>\n");
        return;
    }

    uint32_t addr = parse_uint32(argv[0]);
    uint32_t len = parse_uint32(argv[1]);
    for (uint32_t i = 0; i < len; ++i) {
        // Reset the watchdog timer
        AT91C_BASE_WDTC->WDTC_WDCR = 0xa5000001;

        putchar(*(uint8_t*)addr);
        ++addr;
    }
}

void cmd_rx_test(int argc, const char* argv[], void* arg) {
    radio_rx_len = 0;
    usb_rx_start();
    while (!usb_rx_len) {
        // Reset the watchdog timer
        AT91C_BASE_WDTC->WDTC_WDCR = 0xa5000001;

        if (radio_poll()) {
            printf("got %d\n", radio_rx_len);
            // printf("RSSI %d dBm\n", (int)last_rssi / 2 - 74);
            for (int i = 0; i < radio_rx_len; ++i) {
                printf("%02x ", radio_rx_buf[i]);
                if ((i & 7) == 7) {
                    printf("\n");
                }
            }
            if (radio_rx_len & 7) {
                printf("\n");
            }
            printf("\n");
            radio_rx_len = 0;
        }
    }
    usb_rx_start();
}

void cmd_kicker_test(int argc, const char* argv[], void* arg) {
    // Print the results of the power-on kicker test
    printf("%d %d\n", kicker_test_v1, kicker_test_v2);
}

void cmd_drive_mode(int argc, const char* argv[], void* arg) {
    if (argc != 2) {
        printf("drive_mode <motor 0..4> <mode 0..3>\n");
        return;
    }

    int n = parse_uint32(argv[0]);
    if (n <= 4) {
        drive_mode[n] = parse_uint32(argv[1]);
    }
}

void cmd_imu_test(int argc, const char* argv[], void* arg) {
    int max = 0, min = 0;

    radio_rx_len = 0;
    usb_rx_start();
    while (!usb_rx_len) {
        delay_ms(5);

        IMUupdateData();

        // Erases the last print statements
        int numOfLines = 6;
        for (int i = 0; i < numOfLines; i++) printf("\033[F\033[J");

        float q[4] = {0};
        IMUgetQuaternionFloat(q);
        printf("Quat (x100): %6d %6d %6d %6d\n", (int)(100 * q[0]),
               (int)(100 * q[1]), (int)(100 * q[2]), (int)(100 * q[3]));

        long int gyro[3] = {0};
        IMUgetGyro(gyro);
        printf("Gyro: (%d, %d, %d)\n", gyro[0], gyro[1], gyro[2]);

        // float a = 2.0f*(q[0]*q[3] + q[1]*q[2]);
        // float b = 1.0f - 2.0f * ( powf(q[2], 2.0f) * powf(q[3], 2.0f));
        // double z = tan((float)(a/b));
        // printf("Z? (x100): %d", z*100);
        // printf("size(int)=%d, size(long)=%d, size(short)=%d\n", sizeof(int),
        // sizeof(long), sizeof(short));

        long acc[3] = {0};
        IMUgetLinearAccelWorld(acc);

        // float accFloat[3];
        // accFloat[0] = (float)acc[0] / LONG_MAX * 2.0f;
        // accFloat[1] = (float)acc[1] / LONG_MAX * 2.0f;
        // accFloat[2] = (float)acc[2] / LONG_MAX * 2.0f;

        for (int i = 0; i < 3; i++) {
            if (acc[i] < min) min = acc[i];
            if (acc[i] > max) max = acc[i];
        }

        const int SCALE_FACTOR = 65536;

        printf("AccWorld (x100): (%6d, %6d, %6d)\n",
               (acc[0] * 100) / SCALE_FACTOR, (acc[1] * 100) / SCALE_FACTOR,
               (acc[2] * 100) / SCALE_FACTOR);

        IMUgetLinearAccel(acc);
        printf("Acc (x100): (%6d, %6d, %6d)\n", (acc[0] * 100) / SCALE_FACTOR,
               (acc[1] * 100) / SCALE_FACTOR, (acc[2] * 100) / SCALE_FACTOR);

        printf("Acc min=%6d, max=%6d\n", min, max);
    }
    usb_rx_start();
}

static void debug_faults() {
    printf("0x%02x 0x%08x\n", current_motor_faults, failures);
}
static const write_uint_t write_monitor_faults = {(unsigned int*)&debug_update,
                                                  (unsigned int)debug_faults};

static void debug_halls() {
    printf("%3d %3d %3d %3d %3d\n", hall_count[0], hall_count[1], hall_count[2],
           hall_count[3], hall_count[4]);
}
static const write_uint_t write_monitor_halls = {(unsigned int*)&debug_update,
                                                 (unsigned int)debug_halls};

static void debug_charge() {
    printf("%02x %3d\n", kicker_status, kicker_voltage);
}
static const write_uint_t write_monitor_charge = {(unsigned int*)&debug_update,
                                                  (unsigned int)debug_charge};

static void debug_ball() {
    printf("0x%03x 0x%03x %4d\n", ball_sense_light, ball_sense_dark,
           ball_sense_light - ball_sense_dark);
}
static const write_uint_t write_monitor_ball = {(unsigned int*)&debug_update,
                                                (unsigned int)debug_ball};

static const write_uint_t write_fpga_off = {&AT91C_BASE_PIOA->PIO_CODR,
                                            MCU_PROGB};
static const write_uint_t write_reset = {AT91C_RSTC_RCR, 0xa5000005};

const command_t commands[] = {
    {"help", cmd_help},
    {"status", cmd_status},
    {"s", cmd_status},
    {"reflash", cmd_reflash},
    {"reset", cmd_write_uint, (void*) & write_reset},
    {"rw", cmd_read_word},
    {"ww", cmd_write_word},
    {"stfu", cmd_stfu},
    {"run", cmd_run},
    {"inputs", cmd_print_uint32, (void*) & AT91C_BASE_PIOA->PIO_PDSR},
    {"timers", cmd_timers},
    {"fpga_reset", cmd_fpga_reset},
    {"fpga_off", cmd_write_uint, (void*) & write_fpga_off},
    {"fpga_on", cmd_fpga_on},
    {"fpga_test", cmd_fpga_test},
    {"spi_test", cmd_spi_test},
    {"spi_erase", cmd_spi_erase},
    {"spi_write", cmd_spi_write},
    {"spi_read", cmd_spi_read},
    {"radio_configure", (void*)radio_configure},
    {"radio_start", cmd_radio_start},
    {"music", cmd_music},
    {"tone", cmd_tone},
    {"fail", cmd_fail},
    {"adc", cmd_adc},
    {"i2c_read", cmd_i2c_read},
    {"i2c_write", cmd_i2c_write},
    {"monitor_faults", cmd_write_uint, (void*) & write_monitor_faults},
    {"monitor_halls", cmd_write_uint, (void*) & write_monitor_halls},
    {"read", cmd_read},
    {"rx_test", cmd_rx_test},
    {"kicker_test", cmd_kicker_test},
    {"drive_mode", cmd_drive_mode},
    {"monitor_charge", cmd_write_uint, (void*) & write_monitor_charge},
    {"imu_test", cmd_imu_test},
    {"monitor_ball", cmd_write_uint, (void*) & write_monitor_ball},

    // End of list placeholder
    {0, 0}};
