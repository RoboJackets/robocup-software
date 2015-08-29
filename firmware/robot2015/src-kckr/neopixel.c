#include <stdlib.h>

#include "kicker.h"
#include "neopixel.h"

/*
 * buffer that holds RGB values for an LED string
 */
uint8_t* rgbBuffer;

/**
 * initializes the neopixel buffer based on LED_COUNT
 */
void initNeopixelBuffer(void)
{
	rgbBuffer = (uint8_t*) malloc(LED_COUNT * 3);
}

/**
 * frees the neopixel buffer
 */
void freeNeopixelBuffer(void)
{
	free(rgbBuffer);
}

/**
 * directly sets the neopixel byte buffer
 *
 * @param new byte buffer
 */
void setBytes(uint8_t* newBytes)
{
	rgbBuffer = newBytes;
}

/**
 * gets the neopixel byte buffer
 * @return current byte buffer
 */
uint8_t* getBytes(void)
{
	return rgbBuffer;
}

/**
 * sets an led's color in the buffer
 * @param red
 * @param green
 * @param blue
 * @param position in the chain (starting with 1 not 0)
 */
void setLed(uint8_t red, uint8_t green, uint8_t blue, uint8_t pos)
{
	rgbBuffer[(pos - 1) * 3]     = green;
	rgbBuffer[(pos - 1) * 3 + 1] = red;
	rgbBuffer[(pos - 1) * 3 + 2] = blue;
}

/**
 * sets all of the leds to a color in the buffer
 * @param red
 * @param green
 * @param blue
 */
void setLeds(uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t index;

	for (index = 1; index <= LED_COUNT; index++) {
		setLed(red, green, blue, index);
	}
}

/**
 * writes the Neopixel buffer
 */
void writeNeopixels(void)
{
	asm volatile("CLI"			"\n\t" //disable interrupts
	             "EOR r16, r16"		"\n\t" //clear counter

	             ////////////////////////////////////////////////////////
	             //
	             //  WRITE SUBROUTINE/OPERATIONS
	             //
	             //  Interrupts need to remain disabled thoughout this
	             //  routine to gaurentee timing.
	             //
	             //  The entire loop subroutine needs to be less than 64
	             //  words due to limitations of the attiny.
	             //  RJMP .+0 takes 2 cycles saving 1 word over using 2
	             //  NOPs.
	             //  This brings the length to a total of exactly 64
	             //  words.
	             //
	             //  Each write bit operation needs to take 12 cycles
	             //  @9.6MHz
	             //  When SBRS/SBRC fails, it takes 2 cycles.
	             //  When SBRS/SBRC succeeds, it takes 1 cycle.
	             //  This brings the total to exactly 12 cycles.
	             //
	             ////////////////////////////////////////////////////////
	             "LOOP:"			"\n\t" //loop to write a byte

	             //bit 7
	             "SBI  %[PORT], %[PIN]"	"\n\t" //2   cycles
	             "NOP"			"\n\t" //1   cycle
	             "SBRS %[BYTE], 7"	"\n\t" //1/2 cycle(s)
	             "CBI  %[PORT], %[PIN]"	"\n\t" //2   cycles
	             "RJMP .+0"		"\n\t" //2   cycles
	             "SBRC %[BYTE], 7"	"\n\t" //1/2 cycle(s)
	             "CBI  %[PORT], %[PIN]"	"\n\t" //2   cycles
	             "RJMP .+0"		"\n\t" //2   cycles

	             //bit 6
	             "SBI  %[PORT], %[PIN]"	"\n\t"
	             "NOP"			"\n\t"
	             "SBRS %[BYTE], 6"	"\n\t"
	             "CBI  %[PORT], %[PIN]"	"\n\t"
	             "RJMP .+0"		"\n\t"
	             "SBRC %[BYTE], 6"	"\n\t"
	             "CBI  %[PORT], %[PIN]"	"\n\t"
	             "RJMP .+0"		"\n\t"

	             //bit 5
	             "SBI  %[PORT], %[PIN]"	"\n\t"
	             "NOP"			"\n\t"
	             "SBRS %[BYTE], 5"	"\n\t"
	             "CBI  %[PORT], %[PIN]"	"\n\t"
	             "RJMP .+0"		"\n\t"
	             "SBRC %[BYTE], 5"	"\n\t"
	             "CBI  %[PORT], %[PIN]"	"\n\t"
	             "RJMP .+0"		"\n\t"

	             //bit 4
	             "SBI  %[PORT], %[PIN]"	"\n\t"
	             "NOP"			"\n\t"
	             "SBRS %[BYTE], 4"	"\n\t"
	             "CBI  %[PORT], %[PIN]"	"\n\t"
	             "RJMP .+0"		"\n\t"
	             "SBRC %[BYTE], 4"	"\n\t"
	             "CBI  %[PORT], %[PIN]"	"\n\t"
	             "RJMP .+0"		"\n\t"

	             //bit 3
	             "SBI  %[PORT], %[PIN]"	"\n\t"
	             "NOP"			"\n\t"
	             "SBRS %[BYTE], 3"	"\n\t"
	             "CBI  %[PORT], %[PIN]"	"\n\t"
	             "RJMP .+0"		"\n\t"
	             "SBRC %[BYTE], 3"	"\n\t"
	             "CBI  %[PORT], %[PIN]"	"\n\t"
	             "RJMP .+0"		"\n\t"

	             //bit 2
	             "SBI  %[PORT], %[PIN]"	"\n\t"
	             "NOP"			"\n\t"
	             "SBRS %[BYTE], 2"	"\n\t"
	             "CBI  %[PORT], %[PIN]"	"\n\t"
	             "RJMP .+0"		"\n\t"
	             "SBRC %[BYTE], 2"	"\n\t"
	             "CBI  %[PORT], %[PIN]"	"\n\t"
	             "RJMP .+0"		"\n\t"

	             //bit 1 (+ counter increment and active byte update)
	             "SBI  %[PORT], %[PIN]"	"\n\t"
	             "INC  r16"		"\n\t" //increment subroutine counter
	             "SBRS %[BYTE], 1"	"\n\t"
	             "CBI  %[PORT], %[PIN]"	"\n\t"
	             "RJMP .+0"		"\n\t"
	             "SBRC %[BYTE], 1"	"\n\t"
	             "CBI  %[PORT], %[PIN]"	"\n\t"
	             "LD   %[BYTE], %a[PTR]+""\n\t" //load byte from bytes pointer

	             //bit 0 (+ counter compare to end point and branch)
	             "SBI  %[PORT], %[PIN]"	"\n\t"
	             "CPI  r16,     %[COUNT]""\n\t" //compare counter to end
	             "SBRS %[BYTE], 0"	"\n\t"
	             "CBI  %[PORT], %[PIN]"	"\n\t"
	             "RJMP .+0"		"\n\t"
	             "SBRC %[BYTE], 0"	"\n\t"
	             "CBI  %[PORT], %[PIN]"	"\n\t"
	             "BRNE LOOP"		"\n\t" //branch if bytes remain

	             "SEI"			"\n\t" //restore interrupts
	             //no outputs
	             :
	             //inputs
	             : [PORT] "I" (NEOPIXEL_PORT),
	             [PIN]   "I" (NEOPIXEL_PIN),
	             [COUNT] "I" (LED_COUNT * 3),
	             [BYTE]  "r" (rgbBuffer[0]),
	             [PTR]	"e" (rgbBuffer + 1)
	             //clobbered registers
	             : "r16"
	            );

	//delay to ensure write operation succeeds (it should anyway because of
	//function return times on the ATtiny @9.6MHz)
	_delay_us(50);
}
