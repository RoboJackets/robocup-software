#include <stdio.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "bits.h"
#include "serial.h"

#define SER_TX_SIZE 256

int uart1_putchar(char ch, FILE* fp);

FILE uart1_stream = FDEV_SETUP_STREAM(uart1_putchar, NULL, _FDEV_SETUP_WRITE);

char ser_tx_buf[SER_TX_SIZE];
uint8_t ser_tx_write_pos, ser_tx_read_pos;

char ser_tx_empty() {
    clear_bit(UCSR1B, TXCIE1);
    char ret = (ser_tx_write_pos == ser_tx_read_pos);
    set_bit(UCSR1B, TXCIE1);

    return ret;
}

int uart1_putchar(char ch, FILE* fp) {
    // End-of-line expansion
    if (ch == '\n') {
        uart1_putchar('\r', fp);
    }

    // This must be done atomically with respect to the transmit interrupt.
    clear_bit(UCSR1B, TXCIE1);

    if (ser_tx_write_pos == ser_tx_read_pos && bit_is_set(UCSR1A, UDRE1)) {
        // Send immediately.
        UDR1 = ch;
    } else {
        // Something is being sent now, so add this character to the ring buffer

        uint8_t next_write_pos;
        while (1) {
            next_write_pos = ser_tx_write_pos + 1;
            if (next_write_pos == SER_TX_SIZE) {
                next_write_pos = 0;
            }

            if (next_write_pos == ser_tx_read_pos) {
                // The ring buffer is full.
                // Sleep until something happens and check again.
                set_bit(UCSR1B, TXCIE1);
                sleep_cpu();
                clear_bit(UCSR1B, TXCIE1);
            } else {
                break;
            }
        }

        ser_tx_buf[ser_tx_write_pos++] = ch;
        if (ser_tx_write_pos == SER_TX_SIZE) {
            ser_tx_write_pos = 0;
        }
    }

    set_bit(UCSR1B, TXCIE1);

    return 0;
}

ISR(USART1_RX_vect) { uart1_putchar(UDR1, &uart1_stream); }

ISR(USART1_TX_vect) {
    if (ser_tx_read_pos != ser_tx_write_pos) {
        UDR1 = ser_tx_buf[ser_tx_read_pos++];
        if (ser_tx_read_pos == SER_TX_SIZE) {
            ser_tx_read_pos = 0;
        }
    }
}

void ser_init() {
    UBRR1L = 12;  // 38400 baud with 8MHz xtal
    UCSR1B = (1 << RXCIE1) | (1 << TXCIE1) | (1 << RXEN1) | (1 << TXEN1);
    UCSR1C = 0x06;  // 8-bit characters

    stdout = &uart1_stream;
}
