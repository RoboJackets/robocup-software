#include <avr/io.h>
#include <util/delay.h>


//  simple test program to blink an led on pin2 on the ATtiny13
int main(void)
{
    DDRB = 1<<3; // port B3, ATtiny13a pin 2
    PORTB = 0x0;

    while (1)
    {
        PORTB = 1<<3; // port B3, ATtiny13a pin 2
        _delay_ms(100);
        PORTB = 0X0;
        _delay_ms(100);
    }
}
