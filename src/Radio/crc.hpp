#ifndef _CRC_HPP_
#define _CRC_HPP_

#include <stdint.h>

uint16_t crc_calc(const uint8_t *packet, unsigned int length);
void crc_set(uint8_t *packet, unsigned int length);
bool crc_check(const uint8_t *packet, unsigned int length);

#endif // _CRC_HPP_
