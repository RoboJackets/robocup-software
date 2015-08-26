/*******************************************************************************
 * Copyright (c) 2010 InvenSense Corporation, All Rights Reserved.
 * See license.txt for license.
 ******************************************************************************/
#ifndef INVENSENSE_IMU_MLSL_H__
#define INVENSENSE_IMU_MLSL_H__

#include "mltypes.h"

/*---- Possible sources of interrupt into driver ----*/
typedef enum {
    INTSRC_IMU = 0,
    INTSRC_AUX1,
    INTSRC_AUX2,
    INTSRC_TIMER,
    INTSRC_UNKNOWN,
    INTSRC_MPU_FIFO,
    INTSRC_MPU_MOTION,
    NUM_OF_INTSOURCES,
} INT_SOURCE;

#ifdef __cplusplus
extern "C" {
#endif

typedef tMLError (*tWriteBurst)(unsigned char slaveAddr,
                                unsigned char registerAddr,
                                unsigned short length,
                                const unsigned char* data);
typedef tMLError (*tReadBurst)(unsigned char slaveAddr,
                               unsigned char registerAddr,
                               unsigned short length, unsigned char* data);

tMLError IMUserialOpen();
tMLError IMUserialClose(void);
tMLError MLSLSerialReset(void);
tMLError MLSLSerialWriteSingle(unsigned char slaveAddr,
                               unsigned char registerAddr, unsigned char data);
tMLError MLSLSerialWriteBurst(unsigned char slaveAddr,
                              unsigned char registerAddr, unsigned short length,
                              const unsigned char* data);
tMLError MLSLSerialReadSingle(unsigned char slaveAddr,
                              unsigned char registerAddr, unsigned char* data);
tMLError MLSLSerialReadBurst(unsigned char slaveAddr,
                             unsigned char registerAddr, unsigned short length,
                             unsigned char* data);
tMLError MLSLIntHandler(unsigned char intSource);

#ifdef __cplusplus
}
#endif

#endif  // INVENSENSE_IMU_MLSL_H__
