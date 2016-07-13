//*****************************************************************************
//
//  File........: LCD_driver.c
//
//  Author(s)...: ATMEL Norway
//
//  Target(s)...: ATmega169
//
//  Compiler....: AVR-GCC 4.1.1; avr-libc 1.4.5
//
//  Description.: Functions used to control the AVR Butterfly LCD
//
//  Revisions...: 1.0
//
//  YYYYMMDD - VER. - COMMENT                                       - SIGN.
//
//  20021015 - 1.0  - Written for STK502                            - JLL
//  20030116 - 2.0  - Code adapted to AVR Butterfly                 - KS
//  20031009          port to avr-gcc/avr-libc                      - M.Thomas
//  20070122          "updated 2006-10-10" included (from REV07)    - from Atmel
//  20070129          SIGNAL->ISR, gLCD_Start_Scroll_Timer volatile - mt
//
//*****************************************************************************

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdint.h>

#include "button.h"
#include "LCD_driver.h"

#define LCD_INITIAL_CONTRAST 0x0F
#define LCD_TIMER_SEED 3
#define LCD_FLASH_ON 3
#define LCD_FLASH_PERIOD 6

// Used to indicate when the LCD interrupt handler should update the LCD
volatile char gLCD_Update_Required = 0;

// LCD display buffer (for double buffering).
volatile char LCD_Data[LCD_REGISTER_COUNT];

// Buffer that contains the text to be displayed
// Note: Bit 7 indicates that this character is flashing
volatile char gTextBuffer[TEXTBUFFER_SIZE];

// Only six letters can be shown on the LCD.
// With the gScroll and gScrollMode variables,
// one can select which part of the buffer to show
volatile signed char gScroll;
volatile char gScrollMode;

////Start-up delay before scrolling a string over the LCD
volatile char gLCD_Start_Scroll_Timer = 0;

// The gFlashTimer is used to determine the on/off
// timing of flashing characters
volatile char flash_timer = 0;

// Look-up table used when converting ASCII to
// LCD display data (segment control)
unsigned int LCD_character_table[] PROGMEM = {
    0xeaa8,  // '*'
    0x2A80,  // '+'
    0x4000,  // ','
    0x0A00,  // '-'
    0x0A51,  // '.' Degree sign
    0x4008,  // '/'
    0x5559,  // '0'
    0x0118,  // '1'
    0x1e11,  // '2
    0x1b11,  // '3
    0x0b50,  // '4
    0x1b41,  // '5
    0x1f41,  // '6
    0x0111,  // '7
    0x1f51,  // '8
    0x1b51,  // '9'
    0x0000,  // ':' (Not defined)
    0x0000,  // ';' (Not defined)
    0x8208,  // '<'
    0x1a00,  // '='
    0x4820,  // '>'
    0x2851,  // '?'
    0x0000,  // '@' (Not defined)
    0x0f51,  // 'A' (+ 'a')
    0x3991,  // 'B' (+ 'b')
    0x1441,  // 'C' (+ 'c')
    0x3191,  // 'D' (+ 'd')
    0x1e41,  // 'E' (+ 'e')
    0x0e41,  // 'F' (+ 'f')
    0x1d41,  // 'G' (+ 'g')
    0x0f50,  // 'H' (+ 'h')
    0x2080,  // 'I' (+ 'i')
    0x1510,  // 'J' (+ 'j')
    0x8648,  // 'K' (+ 'k')
    0x1440,  // 'L' (+ 'l')
    0x0578,  // 'M' (+ 'm')
    0x8570,  // 'N' (+ 'n')
    0x1551,  // 'O' (+ 'o')
    0x0e51,  // 'P' (+ 'p')
    0x9551,  // 'Q' (+ 'q')
    0x8e51,  // 'R' (+ 'r')
    0x9021,  // 'S' (+ 's')
    0x2081,  // 'T' (+ 't')
    0x1550,  // 'U' (+ 'u')
    0x4448,  // 'V' (+ 'v')
    0xc550,  // 'W' (+ 'w')
    0xc028,  // 'X' (+ 'x')
    0x2028,  // 'Y' (+ 'y')
    0x5009,  // 'Z' (+ 'z')
    0x1441,  // '[' (Same as C)
    0x8020,  // '\'
    0x1111,  // ']' (Not defined)
    0xd000,  // '^' (delta)
    0x1000   // '_'
};

/*****************************************************************************
*
*   Function name : LCD_Init
*
*   Returns :       None
*
*   Parameters :    None
*
*   Purpose :       Initialize LCD_displayData buffer.
*                   Set up the LCD (timing, contrast, etc.)
*
*****************************************************************************/
void lcd_init() {
    LCD_CONTRAST_LEVEL(LCD_INITIAL_CONTRAST);  // Set the LCD contrast level

    // Select asynchronous clock source, enable all COM pins and enable all
    // segment pins.
    LCDCRB = (1 << LCDCS) | (3 << LCDMUX0) | (7 << LCDPM0);

    // Set LCD prescaler to give a framerate of 32,0 Hz
    LCDFRR = (0 << LCDPS0) | (7 << LCDCD0);

    LCDCRA =
        (1 << LCDEN) | (1 << LCDAB);  // Enable LCD and set low power waveform

    // Enable LCD start of frame interrupt
    LCDCRA |= (1 << LCDIE);

    // updated 2006-10-10, setting LCD drive time to 1150us in FW rev 07,
    // instead of previous 300us in FW rev 06. Due to some variations on the LCD
    // glass provided to the AVR Butterfly production.
    LCDCCR |= (1 << LCDDC2) | (1 << LCDDC1) | (1 << LCDDC0);

    gLCD_Update_Required = 0;
}

/*****************************************************************************
*
*   Function name : LCD_WriteDigit(char c, char digit)
*
*   Returns :       None
*
*   Parameters :    Inputs
*                   c: The symbol to be displayed in a LCD digit
*                   digit: In which digit (0-5) the symbol should be displayed
*                   Note: Digit 0 is the first used digit on the LCD,
*                   i.e LCD digit 2
*
*   Purpose :       Stores LCD control data in the LCD_displayData buffer.
*                   (The LCD_displayData is latched in the LCD_SOF interrupt.)
*
*****************************************************************************/
void lcd_write_digit(char c, char digit) {
    unsigned int seg = 0x0000;  // Holds the segment pattern
    char mask, nibble;
    volatile char* ptr;
    char i;

    if (digit > 5)  // Skip if digit is illegal
    {
        return;
    }

    // Lookup character table for segmet data
    if ((c >= '*') && (c <= 'z')) {
        // c is a letter
        if (c >= 'a')  // Convert to upper case
        {
            c &= ~0x20;  // if necessarry
        }

        c -= '*';

        // mt seg = LCD_character_table[c];
        seg = (unsigned int)pgm_read_word(&LCD_character_table[(uint8_t)c]);
    }

    // Adjust mask according to LCD segment mapping
    if (digit & 0x01) {
        mask = 0x0F;  // Digit 1, 3, 5
    } else {
        mask = 0xF0;  // Digit 0, 2, 4
    }

    ptr = LCD_Data + (digit >> 1);  // digit = {0,0,1,1,2,2}

    for (i = 0; i < 4; i++) {
        nibble = seg & 0x000F;
        seg >>= 4;
        if (digit & 0x01) {
            nibble <<= 4;
        }
        *ptr = (*ptr & mask) | nibble;
        ptr += 5;
    }
}

/*****************************************************************************
*
*   LCD Interrupt Routine
*
*   Returns :       None
*
*   Parameters :    None
*
*   Purpose: Latch the LCD_displayData and Set LCD_status.updateComplete
*
*****************************************************************************/

ISR(LCD_vect) {
    static char LCD_timer = LCD_TIMER_SEED;
    static char timeout_count;

    char eol;
    unsigned char i;
    char update;

    /**************** Button timeout for the button.c, START ****************/
    if (!gButtonTimeout) {
        timeout_count++;

        if (timeout_count > 3) {
            gButtonTimeout = 1;
            timeout_count = 0;
        }
    }

    /**************** Button timeout for the button.c, END ******************/

    LCD_timer--;  // Decreased every LCD frame

    update = gLCD_Update_Required;

    if (gScrollMode) {
        // If we are in scroll mode, and the timer has expired,
        // we will update the LCD
        if (LCD_timer == 0) {
            if (gLCD_Start_Scroll_Timer == 0) {
                update = 1;
            } else {
                gLCD_Start_Scroll_Timer--;
            }
        }
    } else {
        // disble LCD start of frame interrupt
        gScroll = 0;
    }

    flash_timer++;
    if (flash_timer == LCD_FLASH_ON) {
        update = 1;
    } else if (flash_timer == LCD_FLASH_PERIOD) {
        flash_timer = 0;
        update = 1;
    }

    eol = 0;
    if (update) {
        // Repeat for the six LCD characters
        for (i = 0; i < 6; i++) {
            char ch, flash;
            if ((gScroll + i) >= 0 && (!eol)) {
                // We have some visible characters
                ch = gTextBuffer[i + gScroll];
                flash = ch & 0x80;
                ch &= 0x7F;

                if (ch == '\0') {
                    eol = i + 1;  // End of character data
                }
            } else {
                ch = ' ';
                flash = 0;
            }

            // Check if this character is flashing
            if (!flash || flash_timer < LCD_FLASH_ON) {
                lcd_write_digit(ch, i);
            } else {
                lcd_write_digit(' ', i);
            }
        }

        // Copy the segment buffer to the real segments
        for (i = 0; i < LCD_REGISTER_COUNT; i++) {
            *(pLCDREG + i) = *(LCD_Data + i);
        }

        // If the text scrolled off the display,
        // we have to start over again.
        if (eol == 1) {
            gScroll = -6;
        } else {
            gScroll++;
        }

        // No need to update anymore
        gLCD_Update_Required = 0;
    }

    // LCD_timer is used when scrolling text
    if (LCD_timer == 0) {
        LCD_timer = LCD_TIMER_SEED;
    }
}

void lcd_update() {
    gLCD_Update_Required = 1;
    flash_timer = 0;
}

void lcd_flash_off() { flash_timer = LCD_FLASH_ON - 1; }

void lcd_clear() {
    uint8_t i;  // char i;

    for (i = 0; i < TEXTBUFFER_SIZE; i++) {
        gTextBuffer[i] = ' ';
    }

    gTextBuffer[0] = '\0';  // mt 5/2007
}

void lcd_putc(uint8_t digit, char character) {
    if (digit < TEXTBUFFER_SIZE) {
        gTextBuffer[digit] = character;
    }
}

void lcd_puts(char* pStr) {
    uint8_t i;

    while (gLCD_Update_Required)
        ;  // Wait for access to buffer

    for (i = 0; pStr[i] && i < TEXTBUFFER_SIZE; i++) {
        gTextBuffer[i] = pStr[i];
    }

    gTextBuffer[i] = '\0';

    if (i > 6) {
        gScrollMode = 1;  // Scroll if text is longer than display size
        gScroll = 0;
        gLCD_Start_Scroll_Timer = 3;  // Start-up delay before scrolling the
                                      // text
    } else {
        gScrollMode = 0;
        gScroll = 0;
    }

    gLCD_Update_Required = 1;
}

void lcd_puts_r(const char* pFlashStr) {
    // char i;
    uint8_t i;

    while (gLCD_Update_Required)
        ;  // Wait for access to buffer

    for (i = 0;
         (const char)(pgm_read_byte(&pFlashStr[i])) && i < TEXTBUFFER_SIZE;
         i++) {
        gTextBuffer[i] = pgm_read_byte(&pFlashStr[i]);
    }

    gTextBuffer[i] = '\0';

    if (i > 6) {
        gScrollMode = 1;  // Scroll if text is longer than display size
        gScroll = 0;
        gLCD_Start_Scroll_Timer = 3;  // Start-up delay before scrolling the
                                      // text
    } else {
        gScrollMode = 0;
        gScroll = 0;
    }

    gLCD_Update_Required = 1;
}
