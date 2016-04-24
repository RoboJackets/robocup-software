#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* Bit manip. defines for clarity */
// _BV = bit value
#define SET_BIT(P, B) (P |= _BV(B))
#define CLEAR_BIT(P, B) (P &= ~_BV(B))
#define TOGGLE_BIT(P, B) (P ^= _BV(B))

#define PARSE_CMD(P) (P >> 6)
#define PARSE_TIME(P) (P & 0x3F)

/* Inputs */
#define VOLTAGE PA2
#define N_KICK_CS PA7

/* Outputs */
#define KICK PA0
#define CHIP PA1
#define DO PA5
#define LED PB0
#define CHARGE PB1

typedef enum {OFF, ON, ACTING} state_type;
volatile state_type state = OFF;
volatile millis_left = 0;

uint8_t get_voltage();

void main() {
    /* Port direction settings */
    DDRA |= _BV(KICK) | _BV(CHIP) | _BV(DO);
    DDRB |= _BV(LED) | _BV(CHARGE);

    DDRA &= ~_BV(N_KICK_CS); // ensure N_KICK_CS is input

    // CS Interrupt
    SET_BIT(GIMSK, PCIE0);
    SET_BIT(PCMSK0, PCINT7);

    /* SPI init - Pg. 120 */
    // 3 Wire Mode DO, DI, USCK - Pg. 124
    SET_BIT(USICR, USIWM0);
    // External, negative edge clock - Pg. 125
    SET_BIT(USICR, USICS1);
    // SET_BIT(USICR, USICS0);
    // Enable Global Interrupts - Required for below
    sei();
    // Enable SPI interrupt
    SET_BIT(USICR, USIOIE);
    // OCR0A is max val of timer
    // we need 1000 clocks at 1 Mhz to get 1 millisecond
    // if we prescale by 1024, then we need 125 on timer to get 1 ms exactly
    // direct access to timer (TCNT0)
    SET_BIT(TIMSK0, OCIE0A);
    SET_BIT(TCCR0A, WGM01);
    OCR0A = 125;

    /* ADC Initialization */
    CLEAR_BIT(PRR, PRADC);   // disable power reduction Pg. 133
    SET_BIT(ADCSRA, ADEN);   // enable the ADC - Pg. 133
    SET_BIT(ADCSRB, ADLAR);  // present left adjusted
    // because we left adjusted and only need
    // 8 bit precision, we can now read ADCH directly
    while (1) { }
}

/* SPI Interrupt */
ISR(USI_OVF_vect) {
    // Get data from USIDR
    uint8_t data = USIDR;
    // Clear the interrupt flag
    SET_BIT(USISR, USIOIF);

    USIDR = get_voltage();

    if (state == ON) {
        uint8_t cmd = PARSE_CMD(data);
        switch (cmd) {
            case 0x2: // chip
                state = ACTING;
                SET_BIT(PORTA, CHIP);
                break;
            case 0x3: // kick
                state = ACTING;
                SET_BIT(PORTA, KICK);
                break;
            default:
                break;
        }

        millis_left = PARSE_TIME(data);
        SET_BIT(TCCR0B, CS01); // start timer /8 prescale
    }
}

/* Chip Select Interrupt */
ISR(PCINT0_vect) {
    int is_chip_selected = !(PINA & _BV(N_KICK_CS)); // check if selected
    if (is_chip_selected) {
        SET_BIT(DDRA, DO);
        state = ON;
    } else {
        CLEAR_BIT(DDRA, DO);
        state = OFF;
    }
}

/* Timer interrupt for chipping / kicking */
/* Gets called once per millisecond */
ISR(TIM0_COMPA_vect) {
    int is_done = --millis_left <= 0; // no more milliseconds left

    if (is_done) {
        // could be kicking or chipping, clear both
        CLEAR_BIT(PORTA, KICK);
        CLEAR_BIT(PORTA, CHIP);
        CLEAR_BIT(TCCR0B, CS01); // stop /8 prescaled timer
        state = ON;
    }
}

/* Voltage Function */
uint8_t get_voltage() {
    // Hard-coded for PA1, check datasheet before changing
    // Set lower three bits to value of pin we read from
    ADMUX |= VOLTAGE;
    // Start conversation by writing to start bit
    SET_BIT(ADCSRA, ADSC);
    // Wait for ADSC bit to clear
    while (ADCSRA & _BV(ADSC));
    // ADHC will go from 0 to 255 corresponding to
    // 0 through VCC
    return ADCH;
}
