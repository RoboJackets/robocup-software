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
#define BV(x)            (1 << x)
#define SET_BIT(P,B)     (P |= BV(B))
#define CLEAR_BIT(P,B)   (P &= ~BV(B))
#define TOGGLE_BIT(P,B)  (P ^= BV(B))

/* Inputs */
#define VOLTAGE     PA2
#define N_KICK_CS   PA7

/* Outputs */
#define KICK        PA0
#define CHIP        PA1
#define DO          PA5
#define LED         PB0
#define CHARGE      PB1

volatile char had_interrupt_ = 0;
volatile uint8_t data_ = 0;
uint8_t spi_enabled = 0;

uint8_t get_voltage();
void init();
void trigger(uint8_t time, uint8_t useKicker);
void delay_us(uint16_t time);

uint8_t time;

void main()
{
    init();

    while (1) {
        if (!(PINA & BV(N_KICK_CS))) { // Check if N_KICK_CS == 0
            if (!spi_enabled) {
                SET_BIT(USICR, USIOIE); // Enable ISR
                SET_BIT(DDRA, DO); // Drive DO
                spi_enabled = 1;
            }

            if (had_interrupt_) {
                char cmd = data_>>6;
                switch(cmd) {
                    case 0x1: // read voltage
                        break;
                    case 0x2: // chip
                        time  = data_ & 0x3F;
                        trigger(time, 0);
                        break;
                    case 0x3: // kick
                        time  = data_ & 0x3F;
                        trigger(time, 1);
                        break;
                }
                // // Simulate kick by toggling LED
                // if (data_ == (uint8_t) 255) {
                //     TOGGLE_BIT(PORTB, LED);
                // }
                // Reset interrupt flag
                had_interrupt_ = 0;

                //USIDR = get_voltage();
                USIDR = (char) 50;
                //USIDR = data_;
            }

        } else if (spi_enabled) {
            CLEAR_BIT(USICR, USIOIE); // Disable ISR
            CLEAR_BIT(DDRA, DO); // DO to Z
            spi_enabled = 0;
        }

    }
}

/* SPI Interrupt */
ISR(USI_OVF_vect)
{
    // Get data from USIDR
    // Using USIBR caused a bit to be shifted
    data_ = USIDR;
    // Clear the interrupt flag (would have been done by USIBR)
    SET_BIT(USISR, USIOIF);
    had_interrupt_ = 1;
}

void init()
{
    /* Port direction settings */
    SET_BIT(DDRA, KICK);
    SET_BIT(DDRA, CHIP);
    SET_BIT(DDRA, DO);
    SET_BIT(DDRB, LED);
    SET_BIT(DDRB, CHARGE);

    SET_BIT(PORTA, N_KICK_CS);

    /* SPI init - Pg. 120 */
    // 3 Wire Mode DO, DI, USCK - Pg. 124
    SET_BIT(USICR, USIWM0);
    // External, negative edge clock - Pg. 125
    SET_BIT(USICR, USICS1);
    // SET_BIT(USICR, USICS0);
    // Enable Global Interrupts - Required for below
    sei();
    // Enable SPI interrupt
    SET_BIT(USICR, USIOIE);

    /* ADC Initialization */
    CLEAR_BIT(PRR, PRADC); // disable power reduction Pg. 133
    SET_BIT(ADCSRA, ADEN); // enable the ADC - Pg. 133
    SET_BIT(ADCSRB, ADLAR); // present left adjusted

    // because we left adjusted and only need
    // 8 bit precision, we can now read ADCH directly
}

void trigger(uint8_t timeKick, uint8_t useKicker)
{
    uint8_t action = useKicker ? KICK : CHIP;
    TOGGLE_BIT(PORTA, action);
    delay_us(timeKick*125);
    TOGGLE_BIT(PORTA, action);
}

/* Voltage Function */
uint8_t get_voltage()
{
    // Hard-coded for PA1
    //SET_BIT(ADMUX, MUX0);
    //Set lower three bits to value of pin we read from
    ADMUX |= VOLTAGE;
    // Start conversation by writing to start bit
    SET_BIT(ADCSRA, ADSC);
    // Wait for ADSC bit to clear
    while (ADCSRA & BV(ADSC));
    // ADHC will go from 0 to 255 corresponding to
    // 0 through VCC
    return ADCH;
}

void delay_us(uint16_t count) {
    while(count--) {
        _delay_us(1);
    }
}
