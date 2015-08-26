// MCK is 48MHz.

// FIXME - Ball sense status LED flashes briefly on reset when radio_configure
// takes a long time

#include <board.h>
#include <stdio.h>
#include <string.h>

#include "timer.h"
#include "console.h"
#include "sound.h"
#include "status.h"
#include "radio.h"
#include "adc.h"
#include "power.h"
#include "control.h"
#include "ball_sense.h"
#include "fpga.h"
#include "i2c.h"
#include "stall.h"
#include "imu.h"
#include "ota_update.h"
#include "main.h"
#include "encoder_monitor.h"
#include "kicker.h"
#include "radio_protocol.h"

#include "invensense/imuSetup.h"
// #include "invensense/imuMlsl.h"
// #include "invensense/imuFIFO.h"
// #include "invensense/imuMldl.h"
// #include "invensense/mpuregs.h"

// Last time the 5ms periodic code was executed
unsigned int update_time;
unsigned int imu_update_time;

#define ENABLE_IMU 0

void (*debug_update)(void) = 0;

void flash(int led, int count) {
    // This cycles at approximately 16Hz
    int phase = (current_time >> 7) & 7;

    if ((phase & 1) == 0) {
        // Even phase
        if ((phase >> 1) < count) {
            LED_ON(led);
        } else {
            LED_OFF(led);
        }
    } else {
        // Odd phase
        LED_OFF(led);
    }
}

void update_leds() {
    // Ball sensor
    if (failures & Fail_Ball_Dazzled) {
        flash(LED_LY, 1);
    } else if (failures & (Fail_Ball_Det_Open | Fail_Ball_Det_Short)) {
        flash(LED_LY, 2);
    } else if (failures & Fail_Ball_LED_Open) {
        flash(LED_LY, 3);
    } else {
        if (have_ball) {
            LED_ON(LED_LY);
        } else {
            LED_OFF(LED_LY);
        }
    }

    // Kicker
    if (failures & Fail_Kicker_Charge) {
        flash(LED_LR, 2);
    } else if (failures & Fail_Kicker_I2C) {
        // 2008 bases have no kicker ADC
        flash(LED_LR, 3);
    } else if (!(kicker_status & Kicker_Charging) ||
               (kicker_status & Kicker_Override)) {
        LED_OFF(LED_LR);
    } else if (kicker_status & Kicker_Charged) {
        LED_ON(LED_LR);
    } else if (kicker_status & Kicker_Charging) {
        flash(LED_LR, 1);
    }
}

#if 0
// Use this main() to debug startup code, IRQ, or linker script problems.
// You can change SConstruct to build for SRAM because linker garbage collection
// will remove all the large code.
int main()
{
	// If no LEDs turn on, we didn't reach this point.
	// Either startup code is broken, built for the wrong memory (flash but running in SRAM?),
	// or LowLevelInit failed (failed to return properly do to interworking?).

	// Turn on all LEDs
	AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOA;
	AT91C_BASE_PIOA->PIO_CODR = LED_ALL;
	AT91C_BASE_PIOA->PIO_OER = LED_ALL;

	// Start PIT.  An IRQ will occur in about 1ms.
	timer_init();
	delay_ms(500);

	// Turn off all LEDs
	AT91C_BASE_PIOA->PIO_SODR = LED_ALL;

	// If the LEDs turn on and stay on, the delay failed (IRQ crashed or did not trigger).
	// If the LEDs turn off, IRQs and startup code are working.

	while (1);
}
#else

int main() {
    // Set up watchdog timer
    AT91C_BASE_WDTC->WDTC_WDCR = 0xa5000001;
    AT91C_BASE_WDTC->WDTC_WDMR =
        AT91C_WDTC_WDRSTEN | AT91C_WDTC_WDDBGHLT | (0xfff << 16) | 0x0ff;

    // Enable user reset (reset button)
    AT91C_BASE_SYS->RSTC_RMR = 0xa5000000 | AT91C_RSTC_URSTEN;

    // Set up PIOs
    // Initially, FLASH_NCS is a PIO because the FPGA will be driving it.
    // After the FPGA is configured (or we give up on it), FLASH_NCS is assigned
    // to SPI.  This happens later in spi_init().
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOA;  // Turn on PIO clock
    // Connect some pins to the PIO controller
    AT91C_BASE_PIOA->PIO_PER =
        LED_ALL | MCU_PROGB | FLASH_NCS | RADIO_INT | VBUS | BALL_LED;
    AT91C_BASE_PIOA->PIO_ODR =
        ~0;  // Disable all outputs (FIXME - unnecessary?)
    AT91C_BASE_PIOA->PIO_OWER =
        LED_ALL;  // Allow LED states to be written directly
    AT91C_BASE_PIOA->PIO_CODR = LED_ALL;            // Turn on all LEDs
    AT91C_BASE_PIOA->PIO_SODR = BALL_LED;           // Turn off ball sensor LED
    AT91C_BASE_PIOA->PIO_OER = LED_ALL | BALL_LED;  // Enable outputs
    // Enable and disable pullups
    AT91C_BASE_PIOA->PIO_PPUER = RADIO_INT | RADIO_NCS | FPGA_NCS | FLASH_NCS |
                                 MISO | ID0 | ID1 | ID2 | ID3 | DP1 | DP2 | DP4;
    AT91C_BASE_PIOA->PIO_PPUDR = VBUS | M2DIV | M3DIV | M5DIV | BALL_LED;

    // Set up MCU_PROGB as an open-drain output, initially high
    AT91C_BASE_PIOA->PIO_SODR = MCU_PROGB;
    AT91C_BASE_PIOA->PIO_MDER = MCU_PROGB;
    AT91C_BASE_PIOA->PIO_OER = MCU_PROGB;

    base2008 = SWITCHES & DP1;

    // More internal peripherals
    timer_init();
    reply_timer_init();

    // At this point, the FPGA is presumed to be the SPI master.
    // Wait for it to configure and determine if it works.
    // If not, it must be disabled.
    // This calls spi_init.
    fpga_init();

    // Find out if the radio works.
    // This tests SPI communications with the radio and tests if
    // the interrupt line is working.
    radio_init();

    // Set up the ADC
    adc_init();

    // Check for low/high supply voltage
    power_init();

    // Set up I2C
    i2c_init();

// FIXME: Enabling this may cause the "all-the-lights"/"no-beep" bug
// (is it timing related?).
#if ENABLE_IMU
    // Set up the IMU
    if (!imu_init()) {
        // FIXME - Test each chip individually
        failures |= Fail_IMU;
    }
#endif

    // Test if the kicker works
    kicker_test();

    // Turn off LEDs
    LED_OFF(LED_ALL);

    // Play startup music
    if (failures == 0) {
        // Everything's good
        music_start(song_startup);
    } else if (failures & Fail_Power) {
        // Dead battery (probably)
        power_fail_music();
    } else {
        // Something's wrong
        music_start(song_failure);
    }

    // Select the default motion controller if enough hardware is working.
    // Some failures are acceptable at this point.
    int showstoppers = failures & ~Fail_Kicker & ~Fail_IMU;
    if (showstoppers == 0) {
        // DP1 on => PD controller, off => dumb controller
        if (base2008) {
            default_controller = &controllers[0];
        } else {
            // default_controller = &controllers[1];
            default_controller = &controllers[0];
        }
        if (!(SWITCHES & DP2)) {
            controller = default_controller;
        }
    }

    // Set up the radio.  After this, it will be able to transmit and receive.
    if (!(failures & Fail_Radio)) {
        radio_configure();

        if (SWITCHES & DP4) {
            // Secondary channel
            radio_channel(10);
        } else {
            // Primary channel
            radio_channel(0);
        }
    }

    rx_lost_time = current_time;

    // Start the controller if one was selected
    if (controller && controller->init) {
        controller->init(0, 0);
    }

    // Read the encoders once so that the first encoder_delta values are all
    // zero
    fpga_read_status();

    stall_init();

    // Main loop
    int lost_radio_count = 0;
    while (1) {
        // Reset the watchdog timer
        AT91C_BASE_WDTC->WDTC_WDCR = 0xa5000001;

        // Handle USB connect/disconnect
        check_usb_connection();

        // Read robot ID
        uint32_t inputs = SWITCHES;
        robot_id = 0;
        if (inputs & ID0) {
            robot_id = 1;
        }
        if (inputs & ID1) {
            robot_id |= 2;
        }
        if (inputs & ID2) {
            robot_id |= 4;
        }
        if (inputs & ID3) {
            robot_id |= 8;
        }

        if (!(failures & Fail_Radio)) {
            // Flash LED and recalibrate radio when signal is lost
            if ((current_time - rx_lost_time) > 250) {
                rx_lost_time = current_time;
                LED_OFF(LED_RG);
                LED_TOGGLE(LED_RR);

                ++lost_radio_count;
                if (lost_radio_count == 10) {
                    lost_radio_count = 0;
                    radio_configure();
                } else {
                    radio_command(SIDLE);
                    radio_command(SFRX);
                    radio_command(SRX);
                }

                // Clear drive commands
                cmd_body_x = 0;
                cmd_body_y = 0;
                cmd_body_w = 0;
                dribble_command = 0;
                kick_command = 0;
                use_chipper = 0;
                accel_limit = 0;
                decel_limit = 0;
            }

            // Check for radio packets
            if (radio_poll()) {
                if (!ota_start() && handle_forward_packet() && controller &&
                    controller->received) {
                    controller->received();
                }
            }

            // Send a reply packet if the reply timer has expired
            radio_reply();
        }

#if ENABLE_IMU
        //	update IMU
        if ((current_time - imu_update_time) >= 1) {
            imu_update_time = current_time;
            // Check for new IMU data
            imu_update();
        }
#endif

        // Periodic activities (every 5ms)
        if ((current_time - update_time) >= 5) {
            update_time = current_time;

            // Read ADC results
            adc_update();

            // Check things that depend on ADC results
            power_update();
            update_ball_sensor();

            // Read encoders
            fpga_read_status();

            // Detect faulty or miswired encoders
            encoder_monitor();

            kicker_monitor();

            // Detect stalled motors
            // This must be done before clearing motor outputs because it uses
            // the old values
            stall_update();

            // Reset motor outputs in case the controller is broken
            for (int i = 0; i < 5; ++i) {
                motor_out[i] = 0;
                drive_mode[i] = DRIVE_OFF;
            }

            // Allow kicking if we have the ball
            if (have_ball || kick_immediate) {
                kick_strength = kick_command;
                kick_immediate = 0;
                kick_command = 0;
            } else {
                kick_strength = 0;
            }

            // Run the controller, if there is one
            if (controller && controller->update) {
                controller->update();
            }

            // Clear the commands for unusable motors
            uint8_t bad_motors = motor_stall | encoder_faults;
            for (int i = 0; i < 5; ++i) {
                if (bad_motors & (1 << i)) {
                    motor_out[i] = 0;
                    drive_mode[i] = DRIVE_OFF;
                }
            }

            // Send commands to and read status from the FPGA
            fpga_send_commands();

            if (usb_is_connected() && debug_update) {
                debug_update();
            }
        }

        // Keep power failure music playing continuously
        power_fail_music();

        update_leds();

        if (sing) {
            music_start(song_victory);
            sing = 0;
        }

        if (anthem) {
            music_start(song_national_anthem);
            anthem = 0;
        }
    }
}

#endif
