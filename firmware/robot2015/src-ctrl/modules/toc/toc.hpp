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
 * toc.hpp - Dynamic logging system for variables defined through macros.
 */

#pragma once


// Includes
#include <cstdint>

#include "logger.hpp"


// Defines
#define LOG_GROUP 0x80


// Macros
#ifdef LINK_TOC_PARAMS

#define LOG_ADD(TYPE, NAME, ADDRESS) \
  { .type = TYPE, .name = #NAME, .address = (void*)(ADDRESS), },

#define LOG_ADD_GROUP(TYPE, NAME, ADDRESS) \
  { \
    .type = TYPE, .name = #NAME, .address = (void*)(ADDRESS), },

#define LOG_GROUP_START(NAME)  \
  static const struct log_s __logs_##NAME[] __attribute__((section(".logsect." #NAME), used)) = \
      { \
        LOG_ADD_GROUP(LOG_GROUP | LOG_START, NAME, 0x00)

#define LOG_GROUP_STOP(NAME) \
  LOG_ADD_GROUP(LOG_GROUP | LOG_STOP, stop_##NAME, 0x00) \
  };

#else

// Null definitions
#define LOG_ADD(TYPE, NAME, ADDRESS)
#define LOG_ADD_GROUP(TYPE, NAME, ADDRESS)
#define LOG_GROUP_START(NAME)
#define LOG_GROUP_STOP(NAME)

#endif


// Task declaration
void Task_TOC(void const* arg);


// Function declarations
void TOCInit(void);
bool TOCTest(void);
// Internal access of log variables
int logGetVarId(char* group, char* name);
float logGetFloat(int varid);
int logGetInt(int varid);
unsigned int logGetUint(int varid);


// Data structures
struct log_s {  // Basic log structure
  uint8_t type;
  char* name;
  void* address;
};

enum {  // Possible variable types
  LOG_UINT8   = 1,
  LOG_UINT16  = 2,
  LOG_UINT32  = 3,
  LOG_INT8    = 4,
  LOG_INT16   = 5,
  LOG_INT32   = 6,
  LOG_FLOAT   = 7,
  LOG_FP16    = 8
};

enum {  // Internal enums
  LOG_STOP  = 0,
  LOG_START = 1
};
