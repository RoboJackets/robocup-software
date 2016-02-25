#include <avr/io.h>
#include <util/delay.h>

int main(void) {
    DDRA = 0xFF;
    
    while(1) {
        PORTA = 0xFF;
        _delay_ms(50);
        PORTA = 0x00;
        _delay_ms(50);
    }
}
