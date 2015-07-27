#include "DS2411.hpp"


static const int tREC = 5;
static const int tSLOT = 65;
static const int tRSTL = 540;
static const int tPDHmax = 60;
static const int tPDLmax = 240;
static const int tMSP = 68;
static const int tRSTH = tPDHmax + tPDLmax + tREC;
static const int tW0L = 90;
static const int tW1L = 10;
static const int tRL = 6;
static const int tMSR = 15;


/**
 * [writeOne description]
 * @param pin [description]
 */
void writeOne(DigitalInOut *pin)
{
    *pin = 0;
    wait_us(tW1L);
    *pin = 1;
    wait_us(tSLOT - tW1L);
}


/**
 * [writeZero description]
 * @param pin [description]
 */
void writeZero(DigitalInOut *pin)
{
    *pin = 0;
    wait_us(tW0L);
    *pin = 1;
    wait_us(tREC);
}


/**
 * Writes one bit at a time, LSB first
 * @param pin [description]
 * @param b   [description]
 */
void writeByte(DigitalInOut *pin, char b)
{
    pin->output();

    for (int i = 1; i < 256; i <<= 1) {
        if ((b & i) == i) {
            writeOne(pin);
        } else {
            writeZero(pin);
        }
    }
}


/**
 * Reads one bit at a time, LSB first
 * @param  pin [description]
 * @return     [description]
 */
char readByte(DigitalInOut *pin)
{
    char value = 0;

    for (int i = 0; i < 8; i++) {
        // Signal ready to read by setting line low
        pin->output();
        *pin = 0;
        wait_us(tRL);
        *pin = 1;

        wait_us(tMSR - tRL);
        pin->input();

        int bit = *pin;
        value |= bit << i;

        wait_us(tSLOT - tMSR); // wait for rest of time slot
    }

    return value;
}


/**
 *  Calculate CRC using polynomial function x^8 + x^5 + x^4 + 1
 *  Source: https://github.com/contiki-os/contiki/blob/master/dev/ds2411/ds2411.c#L175
 */
unsigned int crc8_add(unsigned int acc, char byte)
{
    acc ^= byte;

    for (int i = 0; i < 8; i++) {
        if (acc & 1) {
            acc = (acc >> 1) ^ 0x8c;
        } else {
            acc >>= 1;
        }
    }

    return acc;
}


/**
 * [ds2411_read_id description]
 * @param  pin   [description]
 * @param  id    [description]
 * @param  debug [description]
 * @return       [description]
 */
DS2411_Result ds2411_read_id(PinName pin, DS2411_ID *id, bool debug)
{
    if (debug)
        printf("Communicating with ID Chip...\r\n");

    DigitalInOut idPin(pin, PIN_OUTPUT, PullUp, 1);

    // Reset signal, low for 480us
    idPin = 0;
    wait_us(tRSTL);
    idPin = 1;

    // Wait for presence signal
    wait_us(tMSP);
    idPin.input();

    if (idPin == 1) {
        if (debug)
            printf("Handshake failure!\r\n");

        return HANDSHAKE_FAIL;
    }

    wait_us(tRSTH - tMSP); // wait for rest of tRSTH

    writeByte(&idPin, 0x33); // write Read ROM command 0x33

    char family = readByte(&idPin);

    id->family = family;
    unsigned int calcCRC = crc8_add(0x0, family);

    for (int i = 0; i < 6; i++) {
        char v = readByte(&idPin);

        id->serial[i] = v;
        calcCRC = crc8_add(calcCRC, v);
    }

    char crc = readByte(&idPin);
    id->crc = crc;

    if (debug) {
        printf("Family byte: 0x%02X\r\n", id->family);

        printf("Serial byte: 0x");

        for (int i = 5; i >= 0; i--)
            printf("%02X", id->serial[i]);

        printf("\r\nCRC        : 0x%02X \r\n", id->crc);

        printf("CRCs match : %s\r\n", calcCRC == crc ? "true" : "false");
    }

    return calcCRC == crc ? CRC_MATCH : CRC_FAIL;
}
