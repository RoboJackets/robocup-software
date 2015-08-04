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
 * param.hpp - Provides dynamic system-wide variable access for those defined through macros.
 */

#pragma once


// Includes
#include <cstdint>


// Defines
#define PARAM_1BYTE     (0x00)
#define PARAM_2BYTES    (0x01)
#define PARAM_4BYTES    (0x02)
#define PARAM_8BYTES    (0x03)
#define PARAM_BYTES_MASK  (0x03)

#define PARAM_TYPE_INT    (0 << 2)
#define PARAM_TYPE_FLOAT  (1 << 2)

#define PARAM_SIGNED    (0 << 3)
#define PARAM_UNSIGNED    (1 << 3)

#define PARAM_VARIABLE    (0 << 7)
#define PARAM_GROUP       (1 << 7)

#define PARAM_RONLY     (1 << 6)

#define PARAM_START     (0x01)
#define PARAM_STOP      (0x00)
#define PARAM_SYNC      (0x02)

#define TOC_CH 0
#define READ_CH 1
#define WRITE_CH 2
#define MISC_CH 3

#define CMD_RESET 0
#define CMD_GET_NEXT 1
#define CMD_GET_CRC 2

#define CMD_GET_ITEM 0
#define CMD_GET_INFO 1

#define MISC_SETBYNAME 0


// User-friendly macros
#define PARAM_UINT8     (PARAM_1BYTE  | PARAM_TYPE_INT    | PARAM_UNSIGNED)
#define PARAM_INT8      (PARAM_1BYTE  | PARAM_TYPE_INT    | PARAM_SIGNED)
#define PARAM_UINT16    (PARAM_2BYTES | PARAM_TYPE_INT    | PARAM_UNSIGNED)
#define PARAM_INT16     (PARAM_2BYTES | PARAM_TYPE_INT    | PARAM_SIGNED)
#define PARAM_UINT32    (PARAM_4BYTES | PARAM_TYPE_INT    | PARAM_UNSIGNED)
#define PARAM_INT32     (PARAM_4BYTES | PARAM_TYPE_INT    | PARAM_SIGNED)
#define PARAM_FLOAT     (PARAM_4BYTES | PARAM_TYPE_FLOAT  | PARAM_SIGNED)


// Macros
#ifdef LINK_TOC_PARAMS

#define PARAM_ADD(TYPE, NAME, ADDRESS) \
  { .type = TYPE, .name = #NAME, .address = (void*)(ADDRESS), },

#define PARAM_ADD_GROUP(TYPE, NAME, ADDRESS) \
  { \
    .type = TYPE, .name = #NAME, .address = (void*)(ADDRESS), },

#define PARAM_GROUP_START(NAME)  \
  static const struct param_s __params_##NAME[] __attribute__((section(".param." #NAME), used)) = \
      { \
        PARAM_ADD_GROUP(PARAM_GROUP | PARAM_START, NAME, 0x00)

#define PARAM_GROUP_STOP(NAME) \
  PARAM_ADD_GROUP(PARAM_GROUP | PARAM_STOP, stop_##NAME, 0x00) \
  };

#else

// Null definitions
#define PARAM_ADD(TYPE, NAME, ADDRESS)
#define PARAM_ADD_GROUP(TYPE, NAME, ADDRESS)
#define PARAM_GROUP_START(NAME)
#define PARAM_GROUP_STOP(NAME)

#endif


// Task declaration
void Task_Param(void const* arg);


// Function declarations
void paramInit(void);
bool paramTest(void);


// Data structures
struct param_s {  // Basic parameter structure
  uint8_t type;
  char* name;
  void* address;
};
