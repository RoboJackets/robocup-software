#include <avr/io.h>
/*
 * main
 */
int main(void) {
    DDRA = 0xFF;
    
    while(1) {
        PORTA = 0xFF;
    }
}
