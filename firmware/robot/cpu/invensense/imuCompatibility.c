/*******************************************************************************
 * Copyright (c) 2010 InvenSense Corporation, All Rights Reserved.
 * See license.txt for license.
 ******************************************************************************/
#include "imuCompatibility.h"

uint_fast8_t sStatus = 0;

tMLError isCompatible(uint_fast8_t mask, uint_fast8_t want) {
    if ((sStatus & mask) == want)
        return ML_SUCCESS;
    else
        return ML_ERROR;
}

void clearCompatible() { sStatus = 0; }

void setCompatible(uint_fast8_t bit) { sStatus |= bit; }
