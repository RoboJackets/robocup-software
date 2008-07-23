#include "crc.hpp"

#include <stdio.h>

#define CRC_START   0x0000
#define CRC_POLY    0x8408

uint16_t crc_calc(const uint8_t *packet, unsigned int length)
{
	unsigned int byte, bit;
	uint16_t crc = CRC_START;

	for (byte = 0; byte < length; byte++)
	{
		uint8_t value = packet[byte];

		crc ^= value;
		for (bit = 0; bit < 8; bit++)
		{
			if (crc & 1)
			{
				crc = (crc >> 1) ^ CRC_POLY;
			} else {
				crc >>= 1;
			}
		}
	}

	return crc;
}

void crc_set(uint8_t *packet, unsigned int length)
{
    uint16_t crc = crc_calc(packet, length - 2);
    packet[length - 2] = crc & 0xff;
    packet[length - 1] = crc >> 8;
}

bool crc_check(const uint8_t *packet, unsigned int length)
{
    uint16_t crc = crc_calc(packet, length - 2);
    uint8_t low = crc & 0xff;
    uint8_t high = crc >> 8;
//    printf("%02x check %04x -> %02x %02x\n", packet[0], crc, packet[length - 1], packet[length - 2]);
    return packet[length - 2] == low && packet[length - 1] == high;
}
