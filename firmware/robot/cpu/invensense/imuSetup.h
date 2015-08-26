/*******************************************************************************
 * Copyright (c) 2010 InvenSense Corporation, All Rights Reserved.
 * See license.txt for license.
 ******************************************************************************/
#ifndef INVENSENSE_IMU_SETUP_H__
#define INVENSENSE_IMU_SETUP_H__

#include <stdint.h>
#include "mltypes.h"
#include "mlinclude.h"

/*******************************************************************************/
/*  Bias update functions */
/*******************************************************************************/

#define ML_BIAS_FROM_NO_MOTION 0x0001
#define ML_BIAS_FROM_GRAVITY 0x0002
#define ML_BIAS_FROM_TEMPERATURE 0x0004
#define ML_BIAS_FROM_LPF 0x0008
#define ML_MAG_BIAS_FROM_MOTION 0x0010

#define ML_BIAS_UPDATE_FUNC_DEFAULT \
    ML_BIAS_FROM_NO_MOTION | ML_BIAS_FROM_GRAVITY
#define ML_RAW_DATA_CALLBACK_DEFAULT 0
#define ML_PROCESSED_DATA_CALLBACK_DEFAULT 0
#define ML_ORIENTATION_CALLBACK_DEFAULT 0
#define ML_MOTION_CALLBACK_DEFAULT 0

/*******************************************************************************/
/*  Motion processing engines */
/*******************************************************************************/

#define ML_MOTION_DETECT (0x0004)
#define ML_BIAS_UPDATE (0x0008)
#define ML_SENSOR_FUSION (0x0010)
#define ML_CONTROL (0x0040)
#define ML_BASIC (ML_MOTION_DETECT | ML_BIAS_UPDATE | ML_SENSOR_FUSION)

/*******************************************************************************/
/*  Motion states */
/*******************************************************************************/

#define ML_MOTION 0x0001
#define ML_NO_MOTION 0x0002

/*******************************************************************************/
/*  Sensor types */
/*******************************************************************************/

#define ML_GYROS 0x0001
#define ML_ACCELS 0x0002

/*******************************************************************************/
/*  Motion arrays */
/*******************************************************************************/

#define ML_QUATERNION 0x0003
#define ML_LINEAR_ACCELERATION 0x0004
#define ML_LINEAR_ACCELERATION_WORLD 0x0005
#define ML_CONTROL_DATA 0x0006
#define ML_FOOTER 0x0007
#define ML_RAW_DATA 0x0008
#define ML_TEMPERATURE 0x0009
#define ML_ANGULAR_VELOCITY 0x000a
#define ML_ANGULAR_VELOCITY_WORLD 0x000b
#define ML_EULER_ANGLES 0x000c
#define ML_GRAVITY 0x000d

/*******************************************************************************/
/*  Data Source */
/*******************************************************************************/

#define ML_DATA_FIFO (0x1)
#define ML_DATA_POLL (0x2)

// ((2^30)*((3.14159265358/180)/200Hz)/2)
#define ML_CALIBRATED_GYRO_SCALE (46861LL)

/*******************************************************************************/
/*  Euler angles */
/*******************************************************************************/

#define ML_PITCH 0x0001
#define ML_ROLL 0x0002
#define ML_YAW 0x0004

/*******************************************************************************/
/*  Interrupt Source */
/*******************************************************************************/
#define ML_INT_MOTION (0x01)
#define ML_INT_FIFO (0x02)

#define ML_MOTION_STATE_CHANGE 0x0006

typedef struct {
    unsigned short biasUpdateFunc;  // A function or bitwise OR of functions
                                    // that determine how the gyroscope bias
                                    // will be automatically updated.
    // Functions include ML_BIAS_FROM_NO_MOTION, ML_BIAS_FROM_GRAVITY, and
    // ML_BIAS_FROM_TEMPERATURE.
    // The engine ML_BIAS_UPDATE must be enabled for these algorithms to run.

    unsigned short orientationMask;  // Allows a user to register which
                                     // orientations will trigger the user
                                     // defined callback function.
    // The orientations are ML_X_UP, ML_X_DOWN, ML_Y_UP, ML_Y_DOWN, ML_Z_UP, and
    // ML_Z_DOWN.
    // ML_ORIENTATION_ALL is equivalent to ML_X_UP | ML_X_DOWN | ML_Y_UP |
    // ML_Y_DOWN | ML_Z_UP | ML_Z_DOWN.

    void (*rawDataCallback)(void);  // Callback function that triggers when raw
                                    // data is acquired by the sensor driver.

    void (*processedDataCallback)(void);  // Callback function that triggers
                                          // when all the processing has been
                                          // finished by the motion processing
                                          // engines.

    // The new orientation. May be one of ML_X_UP, ML_X_DOWN, ML_Y_UP,
    // ML_Y_DOWN, ML_Z_UP, or ML_Z_DOWN.

    void (*motionCallback)(unsigned short motionState);  // Callback function
                                                         // that will run when a
                                                         // change of motion
                                                         // state is detected.
    // The new motion state. May be one of ML_MOTION, or ML_NO_MOTION.

} tMLParams,      // new type
    ML_Params_t;  // backward-compatibily type

/* ----------------------- */
/* - MLX Data Structure. - */
/* ----------------------- */
typedef struct {
    // Sensor data variables
    unsigned short mlEngineMask;

    // Calibration parameters
    long mlBias[6];
    long mlAccelCal[9];
    long mlGyroCal[9];  // Deprecated, used mlGyroOrient
    long mlGyroOrient[9];
    long mlAccelSens;
    long mlGyroSens;
    long mlTempSlope[3];
    long mlTempOffset[3];

    // Sensor data
    long mlCalibratedData[6];
    long mlTemperature[1];
    long mlBiasUncertainty[1];
    unsigned short mlFlags[7];
    unsigned short suspend;

    unsigned short mlMotionState;

    unsigned short mlDataMode;
    unsigned short mlInterruptSources;

    unsigned char internalMotionState;
    //      unsigned char saveData[17];

    unsigned char newData;

} tMLXData;

#ifdef __cplusplus
extern "C" {
#endif

// Initialization
tMLError IMUopen(void);
tMLError IMUstart(void);

tMLError IMUsetMotionCallback(void (*func)(unsigned short motionState));

tMLError IMUclose(void);

tMLError IMUsetBiasUpdateFunc(unsigned short function);

tMLError IMUupdateData(void);

// Non User
tMLError IMUsetTemperatureSlope(const long* data);

void MLXInit(void);
tMLError MLControlUpdate(void);
tMLError MLProcessInts(void);
tMLError MLGetDMPData(void);
tMLError MLSetAuxSlaveAddr(unsigned char auxSlaveAddr);
tMLError MLSetAccelCalibration(float range, float* orientation);
tMLError MLSetGyroCalibration(float range, float* orientation);
void MLXAccelInit(unsigned char auxSlaveAddr);
tMLError MLPollMotionStatus(void);
tMLError MLSetDataMode(unsigned short dataMode);
unsigned char MLUpdateBias(void);
tMLError MLEnableControl(void);
tMLError MLDisableControl(void);
tMLError MLSetInterrupts(uint_fast8_t interrupts);

#ifdef __cplusplus
}
#endif

#endif  // INVENSENSE_IMU_SETUP_H__
