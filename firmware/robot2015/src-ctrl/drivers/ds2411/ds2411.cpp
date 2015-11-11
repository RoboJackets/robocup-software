#include "ds2411.hpp"

#include "logger.hpp"


namespace
{
const unsigned int ID_tREC = 5;
const unsigned int ID_tPDHmax = 60;
const unsigned int ID_tPDLmax = 240;
const unsigned int ID_tRSTH = ID_tPDHmax + ID_tPDLmax + ID_tREC;
const unsigned int ID_tSLOT = 65;
const unsigned int ID_tRSTL = 540;
const unsigned int ID_tMSP = 68;
const unsigned int ID_tW0L = 90;
const unsigned int ID_tW1L = 10;
const unsigned int ID_tRL = 6;
const unsigned int ID_tMSR = 15;



/**
 * [writeOne description]
 * @param pin [description]
 */
void writeOne(DigitalInOut* pin)
{
    *pin = 0;
    wait_us(ID_tW1L);
    *pin = !(*pin);
    wait_us(ID_tSLOT - ID_tW1L);
}


/**
 * [writeZero description]
 * @param pin [description]
 */
void writeZero(DigitalInOut* pin)
{
    *pin = 0;
    wait_us(ID_tW0L);
    *pin = !(*pin);
    wait_us(ID_tREC);
}


/**
 * Writes one bit at a time, LSB first
 * @param pin [description]
 * @param b   [description]
 */
void writeByte(DigitalInOut* pin, char b)
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
char readByte(DigitalInOut* pin)
{
    char value = 0;

    for (int i = 0; i < 8; i++) {
        // Signal ready to read by setting line low
        pin->output();
        *pin = 0;
        wait_us(ID_tRL);
        *pin = !(*pin);

        wait_us(ID_tMSR - ID_tRL);
        pin->input();

        int bit = *pin;
        value |= bit << i;

        wait_us(ID_tSLOT - ID_tMSR); // wait for rest of time slot
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

}   // end of anonymous namespace

/**
 * [ds2411_read_id description]
 * @param  pin   [description]
 * @param  id    [description]
 * @param  debug [description]
 * @return       [description]
 */
DS2411Result_t ds2411_read_id(PinName pin, DS2411_t* id, bool debug)
{
    LOG(INF3, "Communicating with ID Chip...");

    DigitalInOut idPin(pin, PIN_OUTPUT, PullUp, 1);

    // Reset signal, low for 480us
    idPin = 0;
    wait_us(ID_tRSTL);
    idPin = !idPin;

    // Wait for presence signal
    wait_us(ID_tMSP);
    idPin.input();

    if (idPin == 1) {
        // LOG(SEVERE, "Handshake failure!");

        return ID_HANDSHAKE_FAIL;
    }

    wait_us(ID_tRSTH - ID_tMSP); // wait for rest of ID_tRSTH

    writeByte(&idPin, 0x33); // write Read ROM command 0x33

    id->family = readByte(&idPin);
    unsigned int calcCRC = crc8_add(0x00, id->family);

    for (int i = 0; i < 6; i++) {
        char v = readByte(&idPin);
        id->serial[i] = v;
        calcCRC = crc8_add(calcCRC, v);
    }

    char crc = readByte(&idPin);
    id->crc = crc;

    /*
        if (debug) {
            printf("Family byte: 0x%02X\r\n", id->family);

            printf("Serial byte: 0x");

            for (int i = 5; i >= 0; i--)
                printf("%02X", id->serial[i]);

            printf("\r\nCRC        : 0x%02X \r\n", id->crc);

            printf("CRCs match : %s\r\n", (calcCRC == crc ? "true" : "false"));
        }
        */

    return (calcCRC == crc ? ID_CRC_MATCH : ID_CRC_FAIL);
}
