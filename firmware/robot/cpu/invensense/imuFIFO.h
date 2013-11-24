/*******************************************************************************
 * Copyright (c) 2010 InvenSense Corporation, All Rights Reserved.
 * See license.txt for license.
 ******************************************************************************/
#ifndef INVENSENSE_IMU_FIFO_H__
#define INVENSENSE_IMU_FIFO_H__

#include "mltypes.h"
#include "mlinclude.h"

#ifdef __cplusplus
extern "C" {
#endif

    /*******************************************************************************/
    /*  Elements                                                                   */
    /*******************************************************************************/

#define ML_ELEMENT_1                    0x0001
#define ML_ELEMENT_2                    0x0002
#define ML_ELEMENT_3                    0x0004
#define ML_ELEMENT_4                    0x0008
#define ML_ELEMENT_5                    0x0010
#define ML_ELEMENT_6                    0x0020
#define ML_ELEMENT_7                    0x0040

#define ML_ALL               (0xFF)

    /*******************************************************************************/
    /*  Accuracy                                                                   */
    /*******************************************************************************/

#define ML_16_BIT                       0x40
#define ML_32_BIT                       0x80

    
    tMLError IMUsetFIFORate(unsigned short fifoRate);

    // Setup FIFO for various output
    tMLError IMUsendQuaternionToFIFO( uint_fast8_t accuracy );
    tMLError IMUsendGyroToFIFO(uint_fast8_t elements, uint_fast8_t accuracy);
    tMLError IMUsendAccelToFIFO(uint_fast8_t elements, uint_fast8_t accuracy);
    tMLError IMUsendLinearAccelToFIFO(uint_fast8_t elements, uint_fast8_t accuracy);
    tMLError IMUsendLinearAccelWorldToFIFO(uint_fast8_t elements, uint_fast8_t accuracy);
    tMLError IMUsendControlDataToFIFO(uint_fast8_t elements, uint_fast8_t accuracy);
    tMLError IMUsendAngularVelocityWorldToFIFO(uint_fast8_t accuracy);
    tMLError IMUsendRawToFIFO(uint_fast8_t elements);

    // Get Fixed Point data from FIFO
    tMLError IMUgetAccel(long *data);
    tMLError IMUgetQuaternion(long *data);
    tMLError IMUgetGyro(long *data);
    tMLError IMUgetLinearAccel(long *data);
    tMLError IMUgetLinearAccelWorld(long *data);
    tMLError IMUgetControlData(long *data);
    tMLError IMUgetAngularVelocityWorld(long *data);
    tMLError IMUgetSensorData(long *data);
    tMLError IMUgetTemperature(long *data);

    // Get Floating Point data from FIFO
    tMLError IMUgetAccelFloat(float *data);
    tMLError IMUgetQuaternionFloat(float *data);

    // Helper function to convert FIFO data
    void quaternionToRotationMatrix( const long *quat, long *rot );

    tMLError MLProcessFIFOData(const unsigned char* dmpData);
    int_fast8_t readAndProcessFIFO( int_fast8_t numPackets );

    tMLError MLSetProcessedFIFOCallback(void (*func)(void) );

    void FIFOParamInit();

#ifdef __cplusplus
}
#endif

#endif // INVENSENSE_IMU_FIFO_H__
