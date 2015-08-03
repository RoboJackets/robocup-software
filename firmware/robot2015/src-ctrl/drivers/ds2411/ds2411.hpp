#pragma once

/**
 *  The DS2411 Maxim Integrated Serial Number SOT23 chip is a 3-pin chip that stores
 *  a unique 48-bit serial number, an 8-bit family code (0x01), and an 8-bit CRC value
 *  computed from both. More information can be found here:
 *  http://www.digikey.com/product-detail/en/DS2411R%2BT%26R/DS2411R%2BCT-ND/2045729
 */

#include "robot.hpp"

#define ID_DS2411_PIN RJ_BASE_ID

struct DS2411_t {
    char family;
    char serial[6]; // 0 = LSB, 5 = MSB
    char crc;
};

typedef enum {
    ID_CRC_MATCH,
    ID_CRC_FAIL,
    ID_HANDSHAKE_FAIL
} DS2411Result_t;

/**
 *  This function connects to the specified PinName and populates the provided DS2411_ID block.
 *  If debug is set to true, the received data is formatted and printed to stdout.
 *  If the computed CRC using the family and serial bytes does not match the received crc, CRC_FAIL is returned.
 *  If the initial handshake fails, HANDSHAKE_FAIL is returned.
 *
 * [ds2411_read_id description]
 * @param  pin   [description]
 * @param  id    [description]
 * @param  debug [description]
 * @return       [description]
 */
DS2411Result_t ds2411_read_id(PinName, DS2411_t *, bool = false);
