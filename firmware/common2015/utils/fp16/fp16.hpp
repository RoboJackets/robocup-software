/**
 * RoboJackets: RoboCup SSL Firmware
 *
 * Copyright (C) 2015 RoboJackets JJ
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * fp16.cpp - 16bit floating point handling functions
 */

/* To not use the GCC implementation, uint16_t is used to carry fp16 values
 *
 * FP16 or Half precision floating points is specified by IEEE 754 as binary 16.
 * (float is specified as binary 32). This implementation is NOT GUARANTEED to
 * be conform to the ieee 754 specification, it is 'just' good enough for the
 * Crazyflie usage. For more info about fp16 see
 * http://en.wikipedia.org/wiki/Half-precision_floating-point_format
 *
 * The current implementation has the following limitation:
 *  * No subnormalized number generation
 *  * Rounding seems to give at least 11 bits precision
 *  * Faster and smaller than the GCC implementation
 */

#pragma once

#include <stdint.h>

uint16_t single2half(float number);
float half2single(uint16_t number);
