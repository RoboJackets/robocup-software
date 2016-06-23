/*! ----------------------------------------------------------------------------
 *  @file    deca_range_tables.c
 *  @brief   DW1000 range correction tables
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
// #include <cstdio>
// #include <cstdlib>

#include "deca_device_api.h"
#include "deca_param_types.h"

#include "mbed.h"

#define NUM_16M_OFFSET (37)
#define NUM_16M_OFFSETWB (68)
#define NUM_64M_OFFSET (26)
#define NUM_64M_OFFSETWB (59)

const uint8 chan_idxnb[NUM_CH_SUPPORTED] = {
    0, 0, 1, 2,
    0, 3, 0, 0}; // Only channels 1,2,3 and 5 are in the narrow band tables
const uint8 chan_idxwb[NUM_CH_SUPPORTED] = {
    0, 0, 0, 0,
    0, 0, 0, 1}; // Only channels 4 and 7 are in in the wide band tables

//---------------------------------------------------------------------------------------------------------------------------
// Range Bias Correction TABLES of range values in integer units of 25 CM, for
// 8-bit unsigned storage, MUST END IN 255 !!!!!!
//---------------------------------------------------------------------------------------------------------------------------

// offsets to nearest centimetre for index 0, all rest are +1 cm per value

#define CM_OFFSET_16M_NB (-23) // For normal band channels at 16 MHz PRF
#define CM_OFFSET_16M_WB (-28) // For wider  band channels at 16 MHz PRF
#define CM_OFFSET_64M_NB (-17) // For normal band channels at 64 MHz PRF
#define CM_OFFSET_64M_WB (-30) // For wider  band channels at 64 MHz PRF

//---------------------------------------------------------------------------------------------------------------------------
// range25cm16PRFnb: Range Bias Correction table for narrow band channels at 16
// MHz PRF, NB: !!!! each MUST END IN 255 !!!!
//---------------------------------------------------------------------------------------------------------------------------

const uint8 range25cm16PRFnb[4][NUM_16M_OFFSET] = {
    // Ch 1 - range25cm16PRFnb
    {1, 3, 4, 5, 7, 9, 11, 12, 13, 15, 18, 20, 23, 25, 28, 30, 33, 36, 40, 43,
     47, 50, 54, 58, 63, 66, 71, 76, 82, 89, 98, 109, 127, 155, 222, 255, 255},

    // Ch 2 - range25cm16PRFnb
    {1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 15, 18, 20, 22, 24, 27, 29, 32, 35, 38,
     41, 44, 47, 51, 55, 58, 62, 66, 71, 78, 85, 96, 111, 135, 194, 240, 255},

    // Ch 3 - range25cm16PRFnb
    {1, 2, 3, 4, 5, 7, 8, 9, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 31, 33, 36,
     39, 42, 45, 49, 52, 55, 59, 63, 69, 76, 85, 98, 120, 173, 213, 255},

    // Ch 5 - range25cm16PRFnb
    {1, 1, 2, 3, 4, 5, 6, 6, 7, 8, 9, 11, 12, 14, 15, 16, 18, 20, 21, 23, 25,
     27, 29, 31, 34, 36, 38, 41, 44, 48, 53, 59, 68, 83, 120, 148,
     255}}; // end range25cm16PRFnb

//---------------------------------------------------------------------------------------------------------------------------
// range25cm16PRFwb: Range Bias Correction table for wide band channels at 16
// MHz PRF, NB: !!!! each MUST END IN 255 !!!!
//---------------------------------------------------------------------------------------------------------------------------

const uint8 range25cm16PRFwb[2][NUM_16M_OFFSETWB] = {
    // Ch 4 - range25cm16PRFwb
    {7, 7, 8, 9, 9, 10, 11, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
     24, 26, 27, 28, 30, 31, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 55, 57,
     59, 61, 63, 66, 68, 71, 74, 78, 81, 85, 89, 94, 99, 104, 110, 116, 123,
     130, 139, 150, 164, 182, 207, 238, 255, 255, 255, 255, 255},

    // Ch 7 - range25cm16PRFwb
    {4, 5, 5, 5, 6, 6, 7, 7, 7, 8, 9, 9, 10, 10, 11, 11, 12, 13, 13, 14, 15, 16,
     17, 17, 18, 19, 20, 21, 22, 23, 25, 26, 27, 29, 30, 31, 32, 34, 35, 36, 38,
     39, 40, 42, 44, 46, 48, 50, 52, 55, 58, 61, 64, 68, 72, 75, 80, 85, 92,
     101, 112, 127, 147, 168, 182, 194, 205, 255}}; // end range25cm16PRFwb

//---------------------------------------------------------------------------------------------------------------------------
// range25cm64PRFnb: Range Bias Correction table for narrow band channels at 64
// MHz PRF, NB: !!!! each MUST END IN 255 !!!!
//---------------------------------------------------------------------------------------------------------------------------

const uint8 range25cm64PRFnb[4][NUM_64M_OFFSET] = {
    // Ch 1 - range25cm64PRFnb
    {1, 2, 2, 3, 4, 5, 7, 10, 13, 16, 19, 22, 24, 27, 30, 32, 35, 38, 43, 48,
     56, 78, 101, 120, 157, 255},

    // Ch 2 - range25cm64PRFnb
    {1, 2, 2, 3, 4, 4, 6, 9, 12, 14, 17, 19, 21, 24, 26, 28, 31, 33, 37, 42, 49,
     68, 89, 105, 138, 255},

    // Ch 3 - range25cm64PRFnb
    {1, 1, 2, 3, 3, 4, 5, 8, 10, 13, 15, 17, 19, 21, 23, 25, 27, 30, 33, 37, 44,
     60, 79, 93, 122, 255},

    // Ch 5 - range25cm64PRFnb
    {1, 1, 1, 2, 2, 3, 4, 6, 7, 9, 10, 12, 13, 15, 16, 17, 19, 21, 23, 26, 30,
     42, 55, 65, 85, 255}}; // end range25cm64PRFnb

//---------------------------------------------------------------------------------------------------------------------------
// range25cm64PRFwb: Range Bias Correction table for wide band channels at 64
// MHz PRF, NB: !!!! each MUST END IN 255 !!!!
//---------------------------------------------------------------------------------------------------------------------------

const uint8 range25cm64PRFwb[2][NUM_64M_OFFSETWB] = {
    // Ch 4 - range25cm64PRFwb
    {7, 8, 8, 9, 9, 10, 11, 12, 13, 13, 14, 15, 16, 16, 17, 18, 19, 19, 20, 21,
     22, 24, 25, 27, 28, 29, 30, 32, 33, 34, 35, 37, 39, 41, 43, 45, 48, 50, 53,
     56, 60, 64, 68, 74, 81, 89, 98, 109, 122, 136, 146, 154, 162, 178, 220,
     249, 255, 255, 255},

    // Ch 7 - range25cm64PRFwb
    {4, 5, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 12, 13, 13, 14, 15,
     16, 16, 17, 18, 19, 19, 20, 21, 22, 23, 24, 25, 26, 28, 29, 31, 33, 35, 37,
     39, 42, 46, 50, 54, 60, 67, 75, 83, 90, 95, 100, 110, 135, 153, 172, 192,
     255}}; // end range25cm64PRFwb

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_getrangebias()
 *
 * @brief This function is used to return the range bias correction need for TWR
 *with DW1000 units.
 *
 * input parameters:
 * @param chan  - specifies the operating channel (e.g. 1, 2, 3, 4, 5, 6 or 7)
 * @param range - the calculated distance before correction
 * @param prf	- this is the PRF e.g. DWT_PRF_16M or DWT_PRF_64M
 *
 * output parameters
 *
 * returns correction needed in meters
 */
double dwt_getrangebias(uint8 chan, float range, uint8 prf) {
  // First get the lookup index that corresponds to given range for a particular
  // channel at 16M PRF
  int i = 0;
  int chanIdx;
  int cmoffseti; // Integer number of CM offset

  double mOffset; // Final offset result in metres

  // NB: note we may get some small negitive values e.g. up to -50 cm.

  int rangeint25cm =
      (int)(range * 4.00); // Convert range to integer number of 25cm values.

  if (rangeint25cm > 255)
    rangeint25cm = 255; // Make sure it matches largest value in table (all
                        // tables end in 255 !!!!)

  if (prf == DWT_PRF_16M) {
    switch (chan) {
    case 4:
    case 7: {
      chanIdx = chan_idxwb[chan];
      while (rangeint25cm > range25cm16PRFwb[chanIdx][i])
        i++; // Find index in table corresponding to range
      cmoffseti = i + CM_OFFSET_16M_WB; // Nearest centimetre correction
    } break;
    default: {
      chanIdx = chan_idxnb[chan];
      while (rangeint25cm > range25cm16PRFnb[chanIdx][i])
        i++; // Find index in table corresponding to range
      cmoffseti = i + CM_OFFSET_16M_NB; // Nearest centimetre correction
    }
    } // end of switch
  } else // 64M PRF
  {
    switch (chan) {
    case 4:
    case 7: {
      chanIdx = chan_idxwb[chan];
      while (rangeint25cm > range25cm64PRFwb[chanIdx][i])
        i++; // Find index in table corresponding to range
      cmoffseti = i + CM_OFFSET_64M_WB; // Nearest centimetre correction
    } break;
    default: {
      chanIdx = chan_idxnb[chan];
      while (rangeint25cm > range25cm64PRFnb[chanIdx][i])
        i++; // Find index in table corresponding to range
      cmoffseti = i + CM_OFFSET_64M_NB; // Nearest centimetre correction
    }
    } // end of switch
  } // end else

  mOffset = (float)cmoffseti; // Offset result in centimetres

  mOffset *= 0.01; // Convert to metres

  return (mOffset);
}
