#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
// TODO: Make this path less explicit!
#include "../../common2015/drivers/kicker-board/kicker_commands.h"
#include "pins.h"

#define NO_COMMAND 0

// State of the ATTINY kicking/chipping
typedef enum { OFF, ON, ACTING } state_type;
volatile state_type state = OFF;

// Used to time kicks and chips
volatile unsigned millis_left = 0;

// Used to keep track of current button state
volatile int was_kick_debug_pressed = 0;
volatile int was_chip_debug_pressed = 0;
volatile int was_chg_debug_pressed = 0;

uint8_t cur_command = NO_COMMAND;

// always up-to-date voltage so we don't have to get_voltage() inside interupts
uint8_t last_voltage = 0;
// function that does a voltage reading
uint8_t get_voltage();
void execute_cmd(uint8_t, uint8_t);

void main() {
    /* Port direction - setting outputs */
    DDRA |= _BV(KICK) | _BV(CHIP) | _BV(MISO);
    DDRB |= _BV(CHARGE);

    // ensure N_KICK_CS is input
    DDRA &= ~_BV(N_KICK_CS);

    // ensure debug buttons are inputs
    DDRB &= ~_BV(DB_KICK)
             & _BV(DB_CHIP)
             & _BV(DB_CHG);

    // Chip Select Interrupts
    GIMSK |= _BV(PCIE0); // Enable interrupts for PCINT0-PCINT7
    GIMSK |= _BV(PCIE1); // Enable interrupts for PCINT8-PCINT11

    PCMSK0 |= _BV(INT_N_KICK_CS); // Enable on N_KICK_CS

    // Enable interrupts on debug buttons
    PCMSK1 |= _BV(INT_DB_KICK)
             | _BV(INT_DB_CHIP)
             | _BV(INT_DB_CHG);

    // SPI init - Pg. 120
    USICR |= _BV(USIWM0)     // 3 Wire Mode MISO, DI, USCK - Pg. 124
             | _BV(USICS1)   // External, negative edge clock - Pg. 125
             | _BV(USISIE);  // Enable SPI start interrupt

    // Interrupt on TIMER 0
    TIMSK0 |= _BV(OCIE0A);
    // CTC - Clear Timer on Compare Match
    TCCR0A |= _BV(WGM01);
    // OCR0A is max val of timer before reset
    // we need 1000 clocks at 1 Mhz to get 1 millisecond
    // if we prescale by 8, then we need 125 on timer to get 1 ms exactly
    OCR0A = 125; // reset every millisecond

    // ADC Initialization
    PRR &= ~_BV(PRADC); // disable power reduction Pg. 133
    ADCSRA |= _BV(ADEN);   // enable the ADC - Pg. 133
    ADCSRB |= _BV(ADLAR);  // present left adjusted
    // because we left adjusted and only need
    // 8 bit precision, we can now read ADCH directly
    DDRA |= _BV(MISO);
    state = ON;

    // Enable Global Interrupts
    sei();

    // We handle voltage readings here
    while (1) {
        last_voltage = get_voltage();
        _delay_ms(100);
    }
}

/*
 * SPI Starting Interrupt
 * NOTE: This is intentionally not done with an overflow interrupt,
 * as the software chip select interrupt will be triggered almost at
 * the same time as an SPI overflow, causing the SPI interrupt to not be
 * handled.
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
    if (state == ON) {
        USIDR = 0;
        if (cur_command == NO_COMMAND) { // we don't have a command already
            cur_command = data;
        } else { // now get argument and execute command
            execute_cmd(cur_command, data);
            cur_command = NO_COMMAND;
        }
    }
}

/* Chip Select Interrupt */
ISR(PCINT0_vect) {
    int is_chip_selected = !(PINA & _BV(N_KICK_CS));  // check if selected
    if (is_chip_selected) {
        DDRA |= _BV(MISO);
        state = ON;
    } else {
        DDRA &= ~_BV(MISO);
        state = OFF;
    }
}

/*
 * Interrupt if the state of any button has changed
 * Every time a button goes from LOW to HIGH, we will execute a command
 */
ISR(PCINT1_vect) {
    // First we get the current state of each button
    int new_is_kick_debug_pressed = PINB & _BV(DB_KICK);
    int new_is_chip_debug_pressed = PINB & _BV(DB_CHIP);
    int new_is_chg_debug_pressed = PINB & _BV(DB_CHG);

    // We only will execute commands when the user initially presses the button
    // So the old button state needs to be LOW and the new button state needs
    // to be HIGH
    if (!was_kick_debug_pressed && new_is_kick_debug_pressed)
        execute_cmd(KICK_CMD, DEBUG_KICK_TIME);
    if (!was_chip_debug_pressed && new_is_chip_debug_pressed)
        execute_cmd(CHIP_CMD, DEBUG_CHIP_TIME);
    if (!was_chg_debug_pressed && new_is_chg_debug_pressed) { // toggle charge
        if (PINB & _BV(CHARGE)) // check if charge is on
            execute_cmd(SET_CHARGE_CMD, OFF_ARG);
        else
            execute_cmd(SET_CHARGE_CMD, ON_ARG);
    }

    // Now our last state becomes the current state of the buttons
    was_kick_debug_pressed = new_is_kick_debug_pressed;
    was_chip_debug_pressed = new_is_chip_debug_pressed;
    was_chg_debug_pressed = new_is_chg_debug_pressed;
}

/*
 * Timer interrupt for chipping / kicking
 * Gets called once per millisecond
 */
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


/* Executes a command that can come from SPI or a debug button
 * WARNING: This will be called from a service routines, keep it short!
 */
void execute_cmd(uint8_t cmd, uint8_t arg) {
    switch (cur_command) {
    case KICK_CMD:
        state = ACTING;
        PORTA |= _BV(KICK); // set KICK pin
        millis_left = arg;
        TCCR0B |= _BV(CS01); // start timer /8 prescale
        break;
    case CHIP_CMD:
        state = ACTING;
        PORTA |= _BV(CHIP); // set CHIP pin
        millis_left = arg;
        TCCR0B |= _BV(CS01); // start timer /8 prescale
        break;
    case SET_CHARGE_CMD:
        if (arg == ON_ARG) PORTB |= _BV(CHARGE);
        else if (arg == OFF_ARG) PORTB &= ~_BV(CHARGE);
        break;
    case GET_VOLTAGE_CMD:
        USIDR = last_voltage;
        break;
    case PING_CMD:
        USIDR = PING_ACK;
    default:
        break;
    }
}

/* Voltage Function */
uint8_t get_voltage() {
    // Hard-coded for PA1, check datasheet before changing
    // Set lower three bits to value of pin we read from
    ADMUX |= V_MONITOR;
    // Start conversation by writing to start bit
    ADCSRA |= _BV(ADSC);
    // Wait for ADSC bit to clear
    while (ADCSRA & _BV(ADSC))
        ;
    // ADHC will go from 0 to 255 corresponding to
    // 0 through VCC
    return ADCH;
}
