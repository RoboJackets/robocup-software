#include <stdbool.h>

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

// TODO: Make this path less explicit!
#include "kicker_commands.h"
#include "pins.h"

#define NO_COMMAND 0

#define TIMING_CONSTANT 125
#define VOLTAGE_READ_DELAY_MS 40

// Used to time kick and chip durations
volatile unsigned millis_left_ = 0;

// Used to keep track of current button state
volatile int kick_db_held_down_ = 0;
volatile int chip_db_down_ = 0;
volatile int charge_db_down_ = 0;

volatile uint8_t byte_cnt = 0;

volatile uint8_t cur_command_ = NO_COMMAND;

// always up-to-date voltage so we don't have to get_voltage() inside interupts
volatile uint8_t last_voltage_ = 0;

// executes a command coming from SPI
uint8_t execute_cmd(uint8_t, uint8_t);

/* Voltage Function */
uint8_t get_voltage() {
    // Start conversation by writing to start bit
    ADCSRA |= _BV(ADSC);

    // Wait for ADSC bit to clear
    while (ADCSRA & _BV(ADSC))
        ;

    // ADHC will go from 0 to 255 corresponding to 0 through VCC
    return ADCH;
}

/*
 * Returns true if charging is currently active
 */
bool is_charging() { return PORTA & _BV(CHARGE_PIN); }

/*
 * Returns true if the chip's SPI slave interface is currently
 * being selected by a master device.
 */
bool is_chip_selected() { return !(PINA & _BV(N_KICK_CS_PIN)); }

void main() {
    // disable global interrupts
    cli();

    // make sure we're not kicking/chipping right off the start
    PORTA &= ~(_BV(KICK_PIN) | _BV(CHIP_PIN));

    /* Port direction - setting outputs */
    DDRA |= _BV(KICK_PIN) | _BV(CHIP_PIN) |
            _BV(CHARGE_PIN);  // MISO is handled by CS interrupt

    // ensure N_KICK_CS & MISO are inputs
    DDRA &= ~(_BV(N_KICK_CS_PIN) | _BV(MISO_PIN));

    // nop to sync things up
    // _NOP();

    // Which DDRB = 0 and PORTB = 1, these are configured as pull-up inputs
    PORTB |= _BV(DB_KICK_PIN) | _BV(DB_CHIP_PIN) | _BV(DB_CHG_PIN);

    // ensure debug buttons are inputs
    DDRB &= ~_BV(DB_KICK_PIN) & ~_BV(DB_CHIP_PIN) & ~_BV(DB_CHG_PIN);

    // nop to sync things up
    // _NOP();

    // Enable interrupts for PCINT0-PCINT7
    GIMSK |= _BV(PCIE0);

    // == Debug Button Interrupts ==
    // Enable interrupts for PCINT8-PCINT11
    GIMSK |= _BV(PCIE1);

    // Only have the N_KICK_CS interrupt enabled
    PCMSK0 = _BV(INT_N_KICK_CS);

    // Enable interrupts on debug buttons
    PCMSK1 = _BV(INT_DB_KICK) | _BV(INT_DB_CHIP) | _BV(INT_DB_CHG);

    // Hard-coded for PA1, check datasheet before changing
    // Set lower three bits to value of pin we read from
    ADMUX |= V_MONITOR_PIN;

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
    PRR &= ~_BV(PRADC);    // disable power reduction - Pg. 133
    ADCSRA |= _BV(ADEN);   // enable the ADC - Pg. 133
    ADCSRB |= _BV(ADLAR);  // present left adjusted
                           // because we left adjusted and only need
                           // 8 bit precision, we can now read ADCH directly

    // Enable Global Interrupts
    sei();

    const uint8_t kalpha = 32;

    // We handle voltage readings here
    while (true) {
        // get a voltage reading by weighing in a new reading, same concept as
        // TCP RTT estimates
        int voltage_accum =
            (255 - kalpha) * last_voltage_ + kalpha * get_voltage();
        last_voltage_ = voltage_accum / 255;

        _delay_ms(VOLTAGE_READ_DELAY_MS);
    }
}

/*
 * SPI Starting Interrupt
 *
 * NOTE: This is intentionally not done with an overflow interrupt,
 * as the software chip select interrupt will be triggered almost at
 * the same time as an SPI overflow, causing the SPI interrupt to not be
 * handled.
 *
 * ISR for the USI
 */
ISR(USI_STR_vect) {
    // check if the start condition interrupt is set
    if ((USISR & _BV(USISIF))) {
        // clear the SPI start flag
        USISR |= _BV(USISIF);

        // only respond if we're being addressed
        if (!is_chip_selected()) return;
 
        // disable global interrupts
        cli();

        // // Wait for overflow flag to become 1
        while (!(USISR & _BV(USIOIF)))
            ;

        // Get data from USIDR
        uint8_t recv_data = USIDR;

        // Clear the overflow flagS
        USISR |= _BV(USIOIF);

        // increment our received byte count and take appropiate action
        byte_cnt++;
        if (byte_cnt == 1) {
            // we don't have a command already, set the response
            // buffer to the command we received to let the
            // master confirm the given command if desired, top
            // bit is set if currently charging
            cur_command_ = recv_data;
            USIDR = cur_command_;
        } else if (byte_cnt == 2) {
            // execute the currently set command with
            // the newly given argument, set the response
            // buffer to our return value
            USIDR = execute_cmd(cur_command_, recv_data);
        } else {
            USIDR = 0x11;
        }

        // enable global interrupts back
        sei();
    }
}

/*
 * Chip Select Interrupt
 *
 * ISR for PCINT0 - PCINT7
 */
ISR(PCINT0_vect) {
    // disable global interrupts
    cli();

    cur_command_ = NO_COMMAND;
    byte_cnt = 0;

    if (is_chip_selected()) {
        // set the slave data out pin as an output
        DDRA |= _BV(MISO_PIN);
    } else {
        // set the slave data out pin as an input
        DDRA &= ~_BV(MISO_PIN);
        USIDR = is_charging() << 7;
    }

    // enable global interrupts back
    sei();
}

/*
 * Interrupt if the state of any button has changed
 * Every time a button goes from LOW to HIGH, we will execute a command
 *
 * ISR for PCINT8 - PCINT11
 */
// ISR(PCINT1_vect) {
//     // First we get the current state of each button
//     int kick_db_pressed = PINB & _BV(DB_KICK_PIN);
//     int chip_db_pressed = PINB & _BV(DB_CHIP_PIN);
//     int charge_db_pressed = PINB & _BV(DB_CHG_PIN);

//     // We only will execute commands when the user initially presses the
//     button
//     // So the old button state needs to be LOW and the new button state needs
//     // to be HIGH
//     if (!kick_db_held_down_ && kick_db_pressed)
//         execute_cmd(KICK_CMD, DB_KICK_TIME);

//     if (!chip_db_down_ && chip_db_pressed) execute_cmd(CHIP_CMD,
//     DB_CHIP_TIME);

//     // toggle charge
//     if (!charge_db_down_ && charge_db_pressed) {
//         // check if charge is on
//         if (PINA & _BV(CHARGE_PIN)) {
//             execute_cmd(SET_CHARGE_CMD, OFF_ARG);
//         } else {
//             execute_cmd(SET_CHARGE_CMD, ON_ARG);
//         }
//     }

//     // Now our last state becomes the current state of the buttons
//     kick_db_held_down_ = kick_db_pressed;
//     chip_db_down_ = chip_db_pressed;
//     charge_db_down_ = charge_db_pressed;
// }

/*
 * Timer interrupt for chipping/kicking - called every millisecond by timer
 *
 * ISR for TIMER 0
 */
ISR(TIM0_COMPA_vect) {
    // decrement ms counter
    millis_left_--;

    // if the counter hits 0, clear the kick/chip pin state
    if (!millis_left_) {
        // could be kicking or chipping, clear both
        PORTA &= ~(_BV(KICK_PIN) | _BV(CHIP_PIN));
        // stop prescaled timer
        TCCR0B &= ~_BV(CS01);
    }
}

/*
 * Executes a command that can come from SPI or a debug button
 *
 * WARNING: This will be called from an interrupt service routines, keep it
 *short!
 */
uint8_t execute_cmd(uint8_t cmd, uint8_t arg) {
    uint8_t ret_val = 0;

    switch (cmd) {
        case KICK_CMD:
            millis_left_ = arg;
            PORTA |= _BV(KICK_PIN);  // set KICK pin
            TCCR0B |= _BV(CS01);     // start timer /8 prescale
            ret_val = KICK_ACK;
            break;

        case CHIP_CMD:
            millis_left_ = arg;
            PORTA |= _BV(CHIP_PIN);  // set CHIP pin
            TCCR0B |= _BV(CS01);     // start timer /8 prescale
            ret_val = CHIP_ACK;
            break;

        case SET_CHARGE_CMD:
            // set state based on argument
            if (arg == ON_ARG) {
                PORTA |= _BV(CHARGE_PIN);
            } else if (arg == OFF_ARG) {
                PORTA &= ~(_BV(CHARGE_PIN));
            }

            ret_val = SET_CHARGE_ACK;
            break;

        case GET_VOLTAGE_CMD:
            ret_val = last_voltage_;
            break;

        case PING_CMD:
            ret_val = PING_ACK;
            break;

        case GET_BUTTON_STATE_CMD:
            switch (arg) {
                case DB_KICK_STATE:
                    ret_val = (PINB & _BV(DB_KICK_PIN)) != 0;
                    break;

                case DB_CHIP_STATE:
                    ret_val = (PINB & _BV(DB_CHIP_PIN)) != 0;
                    break;

                case DB_CHARGE_STATE:
                    ret_val = (PINB & _BV(DB_CHG_PIN)) != 0;
                    break;

                default:
                    ret_val = 0xAA;  // return a weird value to show arg wasn't
                                     // recognized
                    break;
            }
            break;

        default:
            ret_val =
                0xCC;  // return a weird value to show arg wasn't recognized
            break;
    }

    return ret_val;
}
