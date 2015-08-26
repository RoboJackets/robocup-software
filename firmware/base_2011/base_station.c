#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#include "bits.h"
#include "device.h"
#include "host.h"
#include "radio.h"

uint8_t device_mode;

// Radio GDO0
ISR(INT4_vect) {
    if (device_mode) {
        device_gdo0_vect();
    } else {
        host_gdo0_vect();
    }
}

// Radio GDO2
ISR(INT5_vect) {
    if (device_mode) {
        device_gdo2_vect();
    } else {
        host_gdo2_vect();
    }
}

ISR(USB_GEN_vect) {
    if (device_mode) {
        device_gen_vect();
    } else {
        host_gen_vect();
    }
}

ISR(USB_COM_vect) {
    if (device_mode) {
        device_com_vect();
    } else {
        host_com_vect();
    }
}

void led_test() {
    uint8_t output;
    uint8_t leds;

    TCCR1B = (1 << WGM12) | 3;
    OCR1A = 21833;

    output = PORTB;
    leds = 0x10;
    do {
        PORTB = output | leds;
        loop_until_bit_is_set(TIFR1, OCF1A);
        set_bit(TIFR1, OCF1A);
        leds <<= 1;
    } while (leds);
    PORTB = output;

    TCCR1B = 0;
}

int main() {
    // Disable the watchdog timer.
    // This must be done in case we are started from the bootloader,
    // which uses the WDT to reset.
    wdt_reset();
    clear_bit(MCUSR, WDRF);
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    WDTCSR = 0;

    // Run at full clock speed
    CLKPR = 1 << CLKPCE;
    CLKPR = 0;

    // Setup I/O
    PORTB = 0x00;
    DDRB = 0xf7;  // LEDs and SPI outputs.  PB0 (/SS) must be an output in SPI
                  // master mode.

    PORTD = 0x10;
    DDRD = 0x10;  // Radio /CS

    PORTE = (1 << 2) |  // Pullups: HWB button
            (1 << 4) |  //          GDO0
            (1 << 5);   //          GDO2
    DDRE = 0x80;        // UVCON output

    // Start the 48MHz PLL
    PLLCSR = (3 << PLLP0) | (1 << PLLE);
    loop_until_bit_is_set(PLLCSR, PLOCK);

    // Set up USB:
    //
    // Enable USB core
    set_bit(USBCON, USBE);

    // Enable USB pad regulator
    set_bit(UHWCON, UVREGE);

    // Enable Vbus pad (required for Vbus detection)
    set_bit(USBCON, OTGPADE);

    // Enable USB clock
    clear_bit(USBCON, FRZCLK);

    // Set SPI mode 0 (clock idles low, data sampled on rising edge)
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);

    // Initialize radio
    radio_init();

    // Set up external interrupts
    // CC1101 GDO0 is INT4 (PE4)
    //        GDO2 is INT5 (PE5)
    EICRB = 0x0f;                 // Rising edge on INT4 and INT5
    EIFR = (1 << 4) | (1 << 5);   // Clear any stale interrupt
    EIMSK = (1 << 4) | (1 << 5);  // Enable interrupts

    led_test();

    sei();

    while (1) {
        // Host/device detection
        if (bit_is_set(USBSTA, VBUS)) {
            // Vbus present: device
            device_mode = 1;
            device_main();
        } else {
            // Vbus not present: host
            device_mode = 0;
            host_main();
        }
    }
}
