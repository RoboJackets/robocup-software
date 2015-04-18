#pragma once

#include "robot.hpp"

typedef struct {
	char family;
	char serial[6]; // 0 = LSB, 5 = MSB
	char crc;
} DS2411_ID;

typedef enum {
	CRC_MATCH, CRC_FAIL, HANDSHAKE_FAIL
} DS2411_Result;

DS2411_Result ds2411_read_id(DigitalInOut*, DS2411_ID*);
