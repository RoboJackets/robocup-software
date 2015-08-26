#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "bits.h"
#include "radio.h"
#include "cc1101.h"

ISR(TIMER1_OVF_vect) {
    // RX timeout: recalibrate
    set_bit(PORTB, 5);

    // Go to IDLE.  This only takes 2 radio clocks.
    radio_command(SIDLE);

    // Flush the RX FIFO in case we were receiving a packet
    // and cut it off.
    radio_read(RXFIFO);
    radio_command(SFRX);

    // Calibrate and go to RX.
    radio_command(SRX);

    // Reset FREQOFF
    radio_write(FSCTRL0, 0x00);

    clear_bit(PORTB, 5);
}

uint8_t spi_write(uint8_t data) {
    SPDR = data;
    loop_until_bit_is_set(SPSR, SPIF);
    return SPDR;
}

void fail(uint8_t flash) {
    TCCR1B = (1 << WGM12) | 3;
    OCR1A = 20833;

    while (1) {
        loop_until_bit_is_set(TIFR1, OCF1A);
        set_bit(TIFR1, OCF1A);
        PORTB ^= flash;
    }
}

void radio_init() {
    // Waits for the radio's status and version bytes to have expected values.
    // This verifies that the radio crystal oscillator and SPI interface are
    // working.
    // Checking these two registers isn't strictly necessary but it reduces the
    // chances
    // of a false positive.
    uint8_t version;
    PORTB |= 0xf0;
    do {
        radio_select();
        spi_write(VERSION | CC_READ);
        version = spi_write(SNOP);
        radio_deselect();
    } while (version != 0x04);
    PORTB &= ~0xf0;

    // Test GDO0 high
    radio_write(IOCFG0, 0x2f | GDOx_INVERT);
    if (!bit_is_set(PINE, 4)) {
        PORTB |= 0x20;
        fail(0x10);
    }

    // Test GDO0 low
    radio_write(IOCFG0, 0x2f);
    if (bit_is_set(PINE, 4)) {
        PORTB |= 0x40;
        fail(0x10);
    }

    // Test GDO2 high
    radio_write(IOCFG2, 0x2f | GDOx_INVERT);
    if (!bit_is_set(PINE, 5)) {
        PORTB |= 0x60;
        fail(0x10);
    }

    // Test GDO2 low
    radio_write(IOCFG2, 0x2f);
    if (bit_is_set(PINE, 5)) {
        PORTB |= 0xe0;
        fail(0x10);
    }
}

uint8_t radio_command(uint8_t cmd) {
    radio_select();
    uint8_t status = spi_write(cmd);
    radio_deselect();

    return status;
}

uint8_t radio_read(uint8_t addr) {
    radio_select();
    spi_write(addr | CC_READ);
    uint8_t value = spi_write(SNOP);
    radio_deselect();

    return value;
}

uint8_t radio_write(uint8_t addr, uint8_t value) {
    radio_select();
    uint8_t status = spi_write(addr);
    spi_write(value);
    radio_deselect();

    return status;
}

void radio_timeout_enable() {
    // Reset and enable the RX timeout timer
    TCCR1B = 0;
    TCNT1H = 0;
    TCNT1L = 0;
    TCCR1B = 3;

    // Enable timer1 overflow interrupt
    set_bit(TIFR1, TOV1);
    set_bit(TIMSK1, TOIE1);
}

void radio_timeout_disable() {
    // Stop the timer.
    // It will be reset in radio_enable_timeout().
    TCCR1B = 0;
}
