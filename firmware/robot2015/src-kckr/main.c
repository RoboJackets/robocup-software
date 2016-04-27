#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "pins.h"

#define PARSE_CMD(P) (P >> 6)
#define PARSE_TIME(P) (P & 0x3F)

typedef enum { OFF, ON, ACTING } state_type;
volatile state_type state = OFF;
volatile unsigned millis_left = 0;

// always up-to-date voltage so we don't have to get_voltage() inside interupts
uint8_t last_voltage;

uint8_t get_voltage();

void main() {
    /* Port direction - setting outputs */
    DDRA |= _BV(KICK) | _BV(CHIP) | _BV(DO);
    DDRB |= _BV(LED) | _BV(CHARGE);

    DDRA &= ~_BV(N_KICK_CS);  // ensure N_KICK_CS is input

    /* Chip Select Interrupts */
    GIMSK |= _BV(PCIE0);  // Enable pin interrupts
    // Following is equivalent to SET_BIT(PCMSK0, PCINT7)
    PCMSK0 |= _BV(N_KICK_CS);  // Enable on N_KICK_CS

    /* SPI init - Pg. 120 */
    USICR |= _BV(USIWM0)     // 3 Wire Mode DO, DI, USCK - Pg. 124
             | _BV(USICS1)   // External, negative edge clock - Pg. 125
             | _BV(USISIE);  // Enable SPI start interrupt

    // Interrupt on TIMER 0
    TIMSK0 |= _BV(OCIE0A);
    // CTC - Clear Timer on Compare Match
    TCCR0A |= _BV(WGM01);
    // OCR0A is max val of timer before reset
    // we need 1000 clocks at 1 Mhz to get 1 millisecond
    // if we prescale by 8, then we need 125 on timer to get 1 ms exactly
    OCR0A = 125;

    /* ADC Initialization */
    PRR &= ~_BV(PRADC); // disable power reduction Pg. 133
    ADCSRA |= _BV(ADEN);   // enable the ADC - Pg. 133
    ADCSRB |= _BV(ADLAR);  // present left adjusted
    // because we left adjusted and only need
    // 8 bit precision, we can now read ADCH directly
    DDRA |= _BV(DO);
    state = ON;

    // Enable Global Interrupts
    sei();

    // We handle voltage readings here
    while (1) {
        last_voltage = get_voltage();
        _delay_ms(100);
    }
}

/* SPI Starting Interrupt */
/* NOTE: This is intentionally not done with an overflow interrupt,
 * as the software chip select interrupt will be triggered almost at
 * the same time as an SPI overflow, and pin change interrupts have
 * higher priority.
 */
ISR(USI_STR_vect) {
    // Wait for overflow flag to become 1
    while (!(USISR & _BV(USIOIF)))
        ;
    // // Get data from USIDR
    uint8_t data = USIDR;
    // // Clear the overflow flag
    USISR |= _BV(USIOIF);  // setting the bit actually clears it
    // Clear the SPI start flag
    USISR |= _BV(USISIF);
    USIDR = last_voltage;
    if (state == ON) {
        uint8_t cmd = PARSE_CMD(data);
        switch (cmd) {
            case 0x2:  // chip
                state = ACTING;
                PORTA |= _BV(CHIP);
                break;
            case 0x3:  // kick
                state = ACTING;
                PORTA |= _BV(KICK);
                break;
            default:
                break;
        }
        millis_left = PARSE_TIME(data);
        TCCR0B |= _BV(CS01); // start timer /8 prescale
    }
}

/* Chip Select Interrupt */
ISR(PCINT0_vect) {
    int is_chip_selected = !(PINA & _BV(N_KICK_CS));  // check if selected
    if (is_chip_selected) {
        DDRA |= _BV(DO);
        state = ON;
    } else {
        DDRA &= ~_BV(DO);
        state = OFF;
    }
}

/* Timer interrupt for chipping / kicking */
/* Gets called once per millisecond */
ISR(TIM0_COMPA_vect) {
    int is_done = --millis_left <= 0;  // no more milliseconds left

    if (is_done) {
        // could be kicking or chipping, clear both
        PORTA &= ~_BV(KICK);
        PORTA &= ~_BV(CHIP);
        TCCR0B &= ~_BV(CS01); /// stop prescaled timer
        state = ON;
    }
}

/* Voltage Function */
uint8_t get_voltage() {
    // Hard-coded for PA1, check datasheet before changing
    // Set lower three bits to value of pin we read from
    ADMUX |= VOLTAGE;
    // Start conversation by writing to start bit
    ADCSRA |= _BV(ADSC);
    // Wait for ADSC bit to clear
    while (ADCSRA & _BV(ADSC))
        ;
    // ADHC will go from 0 to 255 corresponding to
    // 0 through VCC
    return ADCH;
}
