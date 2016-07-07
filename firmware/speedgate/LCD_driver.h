//*****************************************************************************
//
//  File........: LCD_Driver.h
//
//  Author(s)...: ATMEL Norway
//
//  Target(s)...: ATmega169
//
//  Description.: Defines and prototypes for LCD_Driver.c
//
//  Revisions...: 1.0
//
//  YYYYMMDD - VER. - COMMENT                                       - SIGN.
//
//  20020606 - 0.10 - File created                                  - RM
//  20021010 - 1.0  - Clean up                                      - JLL
//  20031009          port to avr-gcc/avr-libc                      - M.Thomas
//  20070129          LCD_CONTRAST_LEVEL from Atmel's REV07-code    - mt
//
//*****************************************************************************

#define FALSE 0
#define TRUE (!FALSE)

#define LCD_REGISTER_COUNT 20
#define TEXTBUFFER_SIZE 25

/************************************************************************/
// MACROS
/************************************************************************/
// active = [TRUE;FALSE]
#define LCD_SET_COLON(active) LCD_Data[8] = active

// DEVICE SPECIFIC!!! (ATmega169)
#define pLCDREG ((unsigned char*)(0xEC))

// DEVICE SPECIFIC!!! (ATmega169) First LCD segment register
#define LCD_CONTRAST_LEVEL(level) LCDCCR = ((LCDCCR & 0xF0) | (0x0F & level))

/************************************************************************/
// Global variables
/************************************************************************/
extern volatile char gLCD_Update_Required;
extern volatile char gTextBuffer[TEXTBUFFER_SIZE];
extern volatile char gScrollMode;
extern volatile signed char gScroll;

/************************************************************************/
// Global functions
/************************************************************************/
void lcd_init(void);
void lcd_write_digit(char input, char digit);

void lcd_update(void);
void lcd_flash_off(void);
void lcd_clear(void);
void lcd_putc(uint8_t digit, char character);
void lcd_puts(char* pStr);
void lcd_puts_r(const char* pFlashStr);
