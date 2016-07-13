#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

#include "main.h"
#include "LCD_driver.h"
#include "button.h"

// Distance between sensor centers in 1/10 mm
#define SENSOR_DISTANCE 265

// Minimum time between measurements, in timer2 overflows
#define HOLDOFF_COUNT 122

// Additional bits to augment the beam timer
volatile uint16_t beam_timer_high;

// The entire beam timer value
#define beam_timer_value() ((uint32_t)TCNT1 + ((uint32_t)beam_timer_high << 16))

volatile uint8_t beep_time;
volatile uint16_t power_off_time;
volatile uint8_t holdoff;

// Most recent measurement
uint32_t time_us = 0;
uint8_t forward = 0;

enum {
    SPEED_IDLE,  // Waiting for any beam to be broken

    // Object moving from first to second beam
    SPEED_FWD_FIRST,   // First beam is broken, forward direction
    SPEED_FWD_SECOND,  // Second beam is broken, forward direction
    SPEED_FWD_DONE,    // Cleared second beam

    // Object moving from second to first beam
    SPEED_REV_SECOND,  // Second beam broken, reverse direction
    SPEED_REV_FIRST,   // First beam broken, reverse direction
    SPEED_REV_DONE     // Cleared first beam
};

volatile uint8_t speed_state = SPEED_IDLE;

// Timer 1 is used for both beeping (with output compare) and beam timing.

void beep(uint16_t period) {
    cli();
    TCCR1A = 0x40;
    TCCR1B = 0x09;
    TIMSK1 = 0x00;
    OCR1A = period;
    beep_time = 15;
    sei();
}

void beep_off() {
    TCCR1A = 0;
    TCCR1B = 0;
}

void beam_timer_start() {
    beam_timer_high = 0;
    TCNT1 = 0;
    TIFR1 = 0x01;
    TIMSK1 = 0x01;
    TCCR1A = 0x00;
    TCCR1B = 0x01;
}

void beam_timer_stop() {
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TIMSK1 = 0x00;
}

ISR(TIMER1_OVF_vect) {
    beam_timer_high++;
    if (beam_timer_high >= 16) {
        // Measurement took too long
        speed_state = SPEED_IDLE;
        beam_timer_stop();
    }
}

void clear_display() { lcd_puts_r(PSTR("------")); }

void reset_auto_power_off() {
    // 5 minutes
    power_off_time = 36621;
}

void reset() {
    // Stop the beeper timer
    beep_off();

    // Stop the break-beam timer
    beam_timer_stop();

    // Reset the speed measurement state
    speed_state = SPEED_IDLE;
    holdoff = 0;
    time_us = 0;

    reset_auto_power_off();
}

void power_off() {
    // Turn off peripherals
    reset();

    // Turn off LEDs
    PORTE &= ~(1 << 6);

    // Disable LCD
    LCDCRA &= ~(1 << 7);

    // Wait for all buttons to be released
    while ((PINB & 0xd0) != 0xd0 || (PINE & 0x0c) != 0x0c) {
        Delay(100);
    }

    // Sleep until a button is pressed
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    while ((PINB & 0xd0) == 0xd0 && (PINE & 0x0c) == 0x0c) {
        sleep_mode();
    }

    // Clear the LCD screen before enabling
    uint8_t i;
    for (i = 0; i < 20; i++) {
        *(pLCDREG + i) = 0x00;
    }

    // Enable LCD
    LCDCRA |= 1 << 7;

    // Turn on LEDs
    PORTE |= 1 << 6;

    // Do this again because the LEDs just turned on
    reset();
    clear_display();
}

void speed_interrupt() {
    // Nonzero if first beam is broken
    uint8_t first = PINE & (1 << 4);

    // Nonzero if second beam is broken
    uint8_t second = PINE & (1 << 5);

    switch (speed_state) {
        case SPEED_IDLE:
            if (holdoff == 0) {
                if (second) {
                    beam_timer_start();
                    speed_state = SPEED_REV_SECOND;
                } else if (first) {
                    beam_timer_start();
                    speed_state = SPEED_FWD_FIRST;
                }
            }
            break;

        case SPEED_FWD_FIRST:
            if (second) {
                beam_timer_stop();
                speed_state = SPEED_FWD_SECOND;
            }
            break;

        case SPEED_FWD_SECOND:
            if (!second && !first) {
                speed_state = SPEED_FWD_DONE;
                holdoff = HOLDOFF_COUNT;
            }
            break;

        case SPEED_REV_SECOND:
            if (first) {
                beam_timer_stop();
                speed_state = SPEED_REV_FIRST;
            }
            break;

        case SPEED_REV_FIRST:
            if (!second && !first) {
                speed_state = SPEED_REV_DONE;
                holdoff = HOLDOFF_COUNT;
            }
            break;
    }
}

ISR(TIMER2_OVF_vect) {
    // This happens at about 122Hz.
    // Decrement some slow timers.

    if (holdoff) {
        --holdoff;
    }

    if (power_off_time) {
        --power_off_time;
    }

    if (beep_time) {
        --beep_time;
        if (!beep_time) {
            beep_off();
        }
    }
}

void display_number(uint32_t value, uint8_t digits) {
    lcd_clear();
    for (uint8_t i = 0; i < digits; i++) {
        uint8_t digit = value % 10;
        value /= 10;

        lcd_putc(5 - i, '0' + digit);
    }
    lcd_update();
}

void display_speed() {
    // Velocity in mm/s
    // v(mm/s) = d(mm) / (t(us) / 1e6)
    uint32_t v = SENSOR_DISTANCE * 100000 / time_us;
    if (v > 99999) {
        lcd_puts_r(PSTR("FAST"));
    } else {
        display_number(v, 5);

        if (forward) {
            lcd_putc(0, '>');
        } else {
            lcd_putc(0, '<');
        }
        lcd_update();
    }
}

int main() {
    uint8_t show_speed = 0;

    Initialization();

    // Beeper
    PORTB |= 0x20;
    DDRB |= 0x20;
    // LED enable
    DDRE |= 0x40;
    // Pullups on phototransistors
    PORTE |= 0x30;

    // Timer 2 is used for beep and power-off timing
    ASSR = 0;
    TCCR2A = 3;  // ~122Hz
    TIMSK2 = 1;

    power_off();

    while (1) {
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_mode();

        // Auto power off
        if (power_off_time == 0) {
            power_off();
        }

        switch (getkey()) {
            case KEY_DOWN:
                // Power off
                power_off();
                break;

            case KEY_UP:
                // Clear last measurement
                reset();
                clear_display();
                break;

            case KEY_RIGHT:
                if (time_us) {
                    if (show_speed) {
                        // Display raw time in microseconds
                        display_number(time_us, 6);
                        show_speed = 0;
                    } else {
                        display_speed();
                        show_speed = 1;
                    }
                }
                break;
        }

        if (speed_state == SPEED_FWD_DONE || speed_state == SPEED_REV_DONE) {
            // Display last speed
            // Time in microseconds
            time_us = beam_timer_value();

            if (speed_state == SPEED_FWD_DONE) {
                forward = 1;
                beep(1400);
            } else {
                forward = 0;
                beep(600);
            }
            speed_state = SPEED_IDLE;
            reset_auto_power_off();

            display_speed();
        }
    }
}

/*****************************************************************************
*
*   Function name : Initialization
*
*   Returns :       None
*
*   Parameters :    None
*
*   Purpose :       Initializate the different modules
*
*****************************************************************************/
void Initialization(void) {
    OSCCAL_calibration();  // calibrate the OSCCAL byte

    CLKPR = (1 << CLKPCE);  // set Clock Prescaler Change Enable

    // set prescaler = 8, Inter RC 8Mhz / 8 = 1Mhz
    CLKPR = (1 << CLKPS1) | (1 << CLKPS0);

    // Disable Analog Comparator (power save)
    ACSR = (1 << ACD);

    // Disable Digital input on PF0-2 (power save)
    DIDR0 = (7 << ADC0D);

    Button_Init();  // Initialize pin change interrupt on joystick
    lcd_init();     // initialize the LCD
}

/*****************************************************************************
*
*   Function name : Delay
*
*   Returns :       None
*
*   Parameters :    unsigned int millisec
*
*   Purpose :       Delay-loop
*
*****************************************************************************/
void Delay(unsigned int millisec) {
    // mt, int i did not work in the simulator:  int i;
    uint8_t i;

    while (millisec--) {
        for (i = 0; i < 125; i++) {
            asm volatile("nop" ::);
        }
    }
}

/*****************************************************************************
*
*   Function name : OSCCAL_calibration
*
*   Returns :       None
*
*   Parameters :    None
*
*   Purpose :       Calibrate the internal OSCCAL byte, using the external
*                   32,768 kHz crystal as reference
*
*****************************************************************************/
void OSCCAL_calibration(void) {
    unsigned char calibrate = FALSE;
    int temp;
    unsigned char tempL;

    CLKPR = (1 << CLKPCE);  // set Clock Prescaler Change Enable
    // set prescaler = 8, Inter RC 8Mhz / 8 = 1Mhz
    CLKPR = (1 << CLKPS1) | (1 << CLKPS0);

    TIMSK2 = 0;  // disable OCIE2A and TOIE2

    ASSR = (1 << AS2);  // select asynchronous operation of timer2 (32,768kHz)

    OCR2A = 200;  // set timer2 compare value

    TIMSK0 = 0;  // delete any interrupt sources

    TCCR1B = (1 << CS10);  // start timer1 with no prescaling
    TCCR2A = (1 << CS20);  // start timer2 with no prescaling

    while ((ASSR & 0x01) | (ASSR & 0x04))
        ;  // wait for TCN2UB and TCR2UB to be cleared

    Delay(100);  // wait for external crystal to stabilise

    while (!calibrate) {
        cli();  // disable global interrupt

        TIFR1 = 0xFF;  // clear TIFR1 flags
        TIFR2 = 0xFF;  // clear TIFR2 flags

        TCNT1H = 0;  // clear timer1 counter
        TCNT1L = 0;
        TCNT2 = 0;  // clear timer2 counter

        while (!(TIFR2 & (1 << OCF2A)))
            ;  // wait for timer2 compareflag

        TCCR1B = 0;  // stop timer1

        sei();  // enable global interrupt

        if ((TIFR1 & (1 << TOV1))) {
            temp = 0xFFFF;  // if timer1 overflows, set the temp to 0xFFFF
        } else {            // read out the timer1 counter value
            tempL = TCNT1L;
            temp = TCNT1H;
            temp = (temp << 8);
            temp += tempL;
        }

        if (temp > 6250) {
            OSCCAL--;  // the internRC oscillator runs to fast, decrease the
                       // OSCCAL
        } else if (temp < 6120) {
            OSCCAL++;  // the internRC oscillator runs to slow, increase the
                       // OSCCAL
        } else {
            calibrate = TRUE;  // the interRC is correct
        }

        TCCR1B = (1 << CS10);  // start timer1
    }
    TCCR1B = 0;
}
