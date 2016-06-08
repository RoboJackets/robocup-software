#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
// TODO: Make this path less explicit!
#include "kicker_commands.h"
#include "pins.h"

#define NO_COMMAND 0
#define IGNORE_CS 0

#define TIMING_CONSTANT 125
#define VOLTAGE_READ_DELAY_MS 100

// State of the ATTINY kicking/chipping
typedef enum { OFF, ON, ACTING } state_type;

volatile state_type state_ = ON;

// Used to time kicks and chips
volatile unsigned millis_left_ = 0;

// Used to keep track of current button state
volatile int kick_db_held_down_ = 0;
volatile int chip_db_down_ = 0;
volatile int charge_db_down_ = 0;

uint8_t cur_command_ = NO_COMMAND;

// always up-to-date voltage so we don't have to get_voltage() inside interupts
uint8_t last_voltage_ = 0;

// function that does a voltage reading
uint8_t get_voltage();

// executes a command coming from SPI
void execute_cmd(uint8_t, uint8_t);

void main() {
    /* Port direction - setting outputs */
    DDRA |= _BV(KICK_PIN) | _BV(CHIP_PIN) |
            _BV(CHARGE_PIN);  // MISO is handled by CS interrupt
                              //
    // ensure N_KICK_CS is input
    DDRA &= ~_BV(N_KICK_CS_PIN);

    // ensure debug buttons are inputs
    DDRB &= ~_BV(DB_KICK_PIN) & ~_BV(DB_CHIP_PIN) & ~_BV(DB_CHG_PIN);

    // Which DDRB = 0 and PORTB = 1, these are configured as pull-up inputs
    PORTB |= _BV(DB_KICK_PIN) | _BV(DB_CHIP_PIN) | _BV(DB_CHG_PIN);

    // Chip Select Interrupt
    if (IGNORE_CS)
        // Because we aren't using CS here, MISO must be manually set as an
        // output instead of being managed by the interrupt
        DDRA |= _BV(MISO_PIN);
    else
        GIMSK |= _BV(PCIE0);  // Enable interrupts for PCINT0-PCINT7

    // Debug Button Interrupts
    // Enable interrupts for PCINT8-PCINT11
    GIMSK |= _BV(PCIE1);

    // Enable on N_KICK_CS
    PCMSK0 |= _BV(INT_N_KICK_CS);

    // Enable interrupts on debug buttons
    PCMSK1 |= _BV(INT_DB_KICK) | _BV(INT_DB_CHIP) | _BV(INT_DB_CHG);

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
    OCR0A = TIMING_CONSTANT;  // reset every millisecond

    // ADC Initialization
    PRR &= ~_BV(PRADC);    // disable power reduction Pg. 133
    ADCSRA |= _BV(ADEN);   // enable the ADC - Pg. 133
    ADCSRB |= _BV(ADLAR);  // present left adjusted
    // because we left adjusted and only need
    // 8 bit precision, we can now read ADCH directly

    // Enable Global Interrupts
    sei();

    // We handle voltage readings here
    while (1) {
        last_voltage_ = get_voltage();

        if (last_voltage_ > 100) execute_cmd(SET_CHARGE_CMD, OFF_ARG);

        _delay_ms(VOLTAGE_READ_DELAY_MS);
    }
}

/*
 * SPI Starting Interrupt
 * NOTE: This is intentionally not done with an overflow interrupt,
 * as the software chip select interrupt will be triggered almost at
 * the same time as an SPI overflow, causing the SPI interrupt to not be
 * handled.
 */
ISR(USI_STR_vect) {  // interrupt service routine for the USI
    // Wait for overflow flag to become 1
    while (!(USISR & _BV(USIOIF)))
        ;
    // // Get data from USIDR
    uint8_t data = USIDR;

    // // Clear the overflow flag
    USISR |= _BV(USIOIF);  // setting the bit actually clears it
                           //
    // Clear the SPI start flag
    USISR |= _BV(USISIF);

    if (state_ == ON) {
        USIDR = 0;
        if (cur_command_ == NO_COMMAND) {  // we don't have a command already
            cur_command_ = data;
        } else {  // now get argument and execute command
            execute_cmd(cur_command_, data);
            cur_command_ = NO_COMMAND;
        }
    }
}

/* Chip Select Interrupt */
ISR(PCINT0_vect) {  // interrupt service routine for PCINT0-PCINT7
    // check if selected
    int is_chip_selected = !(PINA & _BV(N_KICK_CS_PIN));

    if (is_chip_selected) {
        DDRA |= _BV(MISO_PIN);
        state_ = ON;
    } else {
        DDRA &= ~_BV(MISO_PIN);
        state_ = OFF;
    }
}

/*
 * Interrupt if the state of any button has changed
 * Every time a button goes from LOW to HIGH, we will execute a command
 */
ISR(PCINT1_vect) {  // interrupt service routine for PCINT8-PCINT11
    // First we get the current state of each button
    int kick_db_pressed = PINB & _BV(DB_KICK_PIN);
    int chip_db_pressed = PINB & _BV(DB_CHIP_PIN);
    int charge_db_pressed = PINB & _BV(DB_CHG_PIN);

    // We only will execute commands when the user initially presses the button
    // So the old button state needs to be LOW and the new button state needs
    // to be HIGH
    if (!kick_db_held_down_ && kick_db_pressed)
        execute_cmd(KICK_CMD, DB_KICK_TIME);

    if (!chip_db_down_ && chip_db_pressed) execute_cmd(CHIP_CMD, DB_CHIP_TIME);

    if (!charge_db_down_ && charge_db_pressed) {  // toggle charge
        if (PINA & _BV(CHARGE_PIN))               // check if charge is on
            execute_cmd(SET_CHARGE_CMD, OFF_ARG);
        else
            execute_cmd(SET_CHARGE_CMD, ON_ARG);
    }

    // Now our last state becomes the current state of the buttons
    kick_db_held_down_ = kick_db_pressed;
    chip_db_down_ = chip_db_pressed;
    charge_db_down_ = charge_db_pressed;
}

/*
 * Timer interrupt for chipping / kicking
 * Gets called once per millisecond
 */
ISR(TIM0_COMPA_vect) {
    if (--millis_left_ <= 0) {
        // could be kicking or chipping, clear both
        PORTA &= ~_BV(KICK_PIN);
        PORTA &= ~_BV(CHIP_PIN);
        TCCR0B &= ~_BV(CS01);  /// stop prescaled timer
        state_ = ON;
    }
}

/*
 * Executes a command that can come from SPI or a debug button
 * WARNING: This will be called from a service routines, keep it short!
 */
void execute_cmd(uint8_t cmd, uint8_t arg) {
    switch (cur_command_) {
        case KICK_CMD:
            USIDR = KICK_ACK;
            state_ = ACTING;
            PORTA |= _BV(KICK_PIN);  // set KICK pin
            millis_left_ = arg;
            TCCR0B |= _BV(CS01);  // start timer /8 prescale
            break;

        case CHIP_CMD:
            USIDR = CHIP_ACK;
            state_ = ACTING;
            PORTA |= _BV(CHIP_PIN);  // set CHIP pin
            millis_left_ = arg;
            TCCR0B |= _BV(CS01);  // start timer /8 prescale
            break;

        case SET_CHARGE_CMD:
            USIDR = SET_CHARGE_ACK;
            if (arg == ON_ARG)
                PORTA |= _BV(CHARGE_PIN);
            else if (arg == OFF_ARG)
                PORTA &= ~_BV(CHARGE_PIN);
            break;

        case GET_VOLTAGE_CMD:
            USIDR = last_voltage_;
            break;

        case PING_CMD:
            USIDR = PING_ACK;
            break;

        case GET_BUTTON_STATE_CMD:
            switch (arg) {
                case DB_KICK_STATE:
                    USIDR = (PINB & _BV(DB_KICK_PIN)) != 0;
                    break;

                case DB_CHIP_STATE:
                    USIDR = (PINB & _BV(DB_CHIP_PIN)) != 0;
                    break;

                case DB_CHARGE_STATE:
                    USIDR = (PINB & _BV(DB_CHG_PIN)) != 0;
                    break;

                default:
                    USIDR = 0xF;  // return a weird value to show arg wasn't
                                  // recognized
                    break;
            }
            break;

        default:
            USIDR = 0xF;  // return a weird value to show arg wasn't recognized
            break;
    }
}

/* Voltage Function */
uint8_t get_voltage() {
    // Hard-coded for PA1, check datasheet before changing
    // Set lower three bits to value of pin we read from
    ADMUX |= V_MONITOR_PIN;

    // Start conversation by writing to start bit
    ADCSRA |= _BV(ADSC);

    // Wait for ADSC bit to clear
    while (ADCSRA & _BV(ADSC))
        ;

    // ADHC will go from 0 to 255 corresponding to 0 through VCC
    return ADCH;
}
