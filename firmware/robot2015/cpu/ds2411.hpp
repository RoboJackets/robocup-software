/**
 *	The DS2411 Maxim Integrated Serial Number SOT23 chip is a 3-pin chip that stores
 *  a unique 48-bit serial number, an 8-bit family code (0x01), and an 8-bit CRC value
 *  computed from both. More information can be found here:
 *  http://www.digikey.com/product-detail/en/DS2411R%2BT%26R/DS2411R%2BCT-ND/2045729
 */

#pragma once

#include "mbed.h"

typedef struct {
	char family;
	char serial[6]; // 0 = LSB, 5 = MSB
	char crc;
} DS2411_ID;

typedef enum {
	CRC_MATCH, CRC_FAIL, HANDSHAKE_FAIL
} DS2411_Result;

/**
 *  This function connects to the specified PinName and populates the provided DS2411_ID block.
 *  If debug is set to true, the received data is formatted and printed to stdout.
 *  If the computed CRC using the family and serial bytes does not match the received crc, CRC_FAIL is returned.
 *  If the initial handshake fails, HANDSHAKE_FAIL is returned.
 */
DS2411_Result ds2411_read_id(PinName pin, DS2411_ID* id, bool debug = false);
