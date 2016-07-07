//***************************************************************************
//
//  File........: button.c
//
//  Author(s)...: ATMEL Norway
//
//  Target(s)...: ATmega169
//
//  Compiler....: AVR-GCC 4.1.1; avr-libc 1.4.5
//
//  Description.: AVR Butterfly button handling routines
//
//  Revisions...: 1.0
//
//  YYYYMMDD - VER. - COMMENT                                       - SIGN.
//
//  20030116 - 1.0  - Created                                       - KS
//  20031009          port to avr-gcc/avr-libc                      - M.Thomas
//  20070129          SIGNAL->ISR                                   - mt
//
//***************************************************************************

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "button.h"
#include "main.h"

#define PINB_MASK ((1 << 4) | (1 << 6) | (1 << 7))
#define PINE_MASK ((1 << 2) | (1 << 3) | (1 << 4) | (1 << 5))

#define BUTTON_UP 6
#define BUTTON_DOWN 7
#define BUTTON_LEFT 2
#define BUTTON_RIGHT 3
#define BUTTON_CENTER 4

volatile uint8_t gButtonTimeout = 0;
volatile char last_key = 0;

void Button_Init(void) {
    // Init port pins
    DDRB &= ~(1 << 7) & ~(1 << 6) & ~(1 << 4);
    PORTB |= PINB_MASK;
    DDRE = 0x00;
    PORTE |= PINE_MASK;

    // Enable pin change interrupt on PORTB and PORTE
    PCMSK0 = PINE_MASK;
    PCMSK1 = PINB_MASK;
    EICRA = 2;
    EIFR = (1 << PCIF0) | (1 << PCIF1);
    EIMSK = (1 << PCIE0) | (1 << PCIE1);
}

void PinChangeInterrupt(void) {
    char buttons;
    char key;

    /*
        Read the buttons:

        Bit             7   6   5   4   3   2   1   0
        ---------------------------------------------
        PORTB           B   A       O
        PORTE                           D   C
        ---------------------------------------------
        PORTB | PORTE   B   A       O   D   C
        =============================================
    */

    buttons = (~PINB) & PINB_MASK;
    buttons |= (~PINE) & 0x0c;

    // Output virtual keys
    if (buttons & (1 << BUTTON_CENTER)) {
        key = KEY_CENTER;
    } else if (buttons & (1 << BUTTON_UP)) {
        key = KEY_UP;
    } else if (buttons & (1 << BUTTON_DOWN)) {
        key = KEY_DOWN;
    } else if (buttons & (1 << BUTTON_LEFT)) {
        key = KEY_LEFT;
    } else if (buttons & (1 << BUTTON_RIGHT)) {
        key = KEY_RIGHT;
    } else {
        key = KEY_NONE;
    }

    if (key != KEY_NONE) {
        if (gButtonTimeout)  // gButtonTimeout is set in the LCD_SOF_interrupt
                             // in LCD_driver.c
        {
            if (last_key == KEY_NONE) {
                last_key = key;
            }
            gButtonTimeout = 0;
        }
    }

    //    EIFR = (1<<PCIF1) | (1<<PCIF0);     // Clear pin change interrupt
    //    flags
}

ISR(PCINT0_vect) {
    speed_interrupt();
    PinChangeInterrupt();
}

ISR(PCINT1_vect) { PinChangeInterrupt(); }

char getkey() {
    char k;

    cli();
    k = last_key;
    last_key = KEY_NONE;
    sei();

    return k;
}

char getch() {
    char k;

    while (!(k = getkey())) {
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_mode();
    }

    return k;
}
