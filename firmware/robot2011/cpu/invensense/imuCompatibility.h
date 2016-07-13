/*******************************************************************************
 * Copyright (c) 2010 InvenSense Corporation, All Rights Reserved.
 * See license.txt for license.
 ******************************************************************************/
#ifndef INVENSENSE_IMU_COMPATABILITY_H__
#define INVENSENSE_IMU_COMPATABILITY_H__

#include "mltypes.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_C_OPEN 1
#define IMU_C_START 2

#define IMU_C_OPEN_CALLED IMU_C_OPEN
#define IMU_C_START_CALLED IMU_C_START

    tMLError isCompatible(uint_fast8_t mask, uint_fast8_t want);
    void setCompatible( uint_fast8_t bit );
    void clearCompatible();

#ifdef __cplusplus
}
#endif



#endif // INVENSENSE_IMU_COMPATABILITY_H__
