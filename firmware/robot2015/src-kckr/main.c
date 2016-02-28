/*
 * Pin - Action - SPI_Pin
 * 10 - SS - p8
 * 11 - MOSI - p5
 * 12 - MISO - p6
 * 13 - SCK - p7
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* Bit manip. defines for clarity */
// BV = bit value
#define BV(x)           (1 << x)
#define setBit(P,B)     (P |= BV(B))
#define clearBit(P,B)   (P &= ~BV(B))
#define toggleBit(P,B)  (P ^= BV(B))

#define DO  PA5 
#define LED PA0

volatile char hadInterrupt = 0;
volatile char data = 0;

void main()
{
    /* Port direction settings */
    setBit(DDRA, DO); // Set DO/MISO OUT
    setBit(DDRA, LED); // Set LED OUT

    /* SPI init - Pg. 120 */
    // 3 Wire Mode DO, DI, USCK - Pg. 124
    setBit(USICR, USIWM0);
    // External, positive edge clock - Pg. 125 
    setBit(USICR, USICS1);
    // Enable Global Interrupts - Required for below
    sei();
    // Enable interrupt
    setBit(USICR, USIOIE);
    
    while (1) {
        if (hadInterrupt) {
            // Reset interrupt flag
            hadInterrupt = 0;
            
            // Simulate kick by toggling LED
            if (data == 'k') {
                toggleBit(PORTA, LED);
            }
        }
    }
}

ISR(USI_OVF_vect)
{
    data = USIBR;
    hadInterrupt = 1;
    // Set this to voltage reading later
    USIDR = 'z';
} 
