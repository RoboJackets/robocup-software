/*******************************************************************************
 * Copyright (c) 2010 InvenSense Corporation, All Rights Reserved.
 * See license.txt for license.
 ******************************************************************************/
#include "imuSetup.h"
#include "imuMldl.h"
#include "dmpDefault.h"
#include "dmpKey.h"
#include "mlcontrol.h"
#include "imuMlos.h"
#include "mpuregs.h"
#include "imuFIFO.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "imuCompatibility.h"

/**
 *   @defgroup MLSETUP
 *   @brief Open, Init, Setup & Close Interface.
 *
 *   @{
 *       @file imuSetup.c
 *       @brief Open, Init, Setup & Close Interface.
**/


#define ML_MOT_STATE_MOVING 0

extern tMLParams mlParams;

tMLParams mlParams = {  
                          ML_BIAS_UPDATE_FUNC_DEFAULT,             // biasUpdateFunc
                          ML_RAW_DATA_CALLBACK_DEFAULT,            // rawDataCallback
                          ML_PROCESSED_DATA_CALLBACK_DEFAULT,      // processedDataCallback
                          ML_ORIENTATION_CALLBACK_DEFAULT,         // orientationCallback
                          ML_MOTION_CALLBACK_DEFAULT               // motionCallback
                     };          

tMLXData mlxData;
extern tMLCTRLXData mlCtrlxData;
extern tMLCTRLParams mlCtrlParams;

/**
 * @internal
 * @brief   Initialize MLX data.  This should be called to setup the mlx
 *          output buffers before any motion processing is done.
**/
void MLXInit(void)
{
    INVENSENSE_FUNC_START
    unsigned char  auxSlaveAddr;
    float identity[9];

    // Set all values to zero by default
    memset( &mlxData, 0, sizeof(tMLXData) );
    memset( identity, 0, 9*sizeof(float) );

    // Now set all the non-zero values
    mlxData.mlEngineMask = ML_BASIC;            // mlEngineMask

    mlxData.mlMotionState = ML_MOTION;          //Motion state

    mlxData.internalMotionState = ML_MOT_STATE_MOVING;

    identity[0] = 1.f;
    identity[4] = 1.f;
    identity[8] = 1.f;
    MLSetGyroCalibration( 2000., identity );

    auxSlaveAddr = MLDLGetDefaultAuxSlaveAddr();
    MLXAccelInit(auxSlaveAddr);
}


/**
 *  @brief  Open the motion sensor engine. This should be called before
 *          all other setup routines, except opening up the serial port
 *          to talk to the hardware (IMUserialOpen).
 *  @return ML_SUCCESS on success; ML_ERROR on any failure.
 *
 */
tMLError IMUopen(void)
{
    tMLError result;
    unsigned char regs;

    INVENSENSE_FUNC_START
    void *dmpInfo;

    clearCompatible();
    setCompatible( IMU_C_OPEN_CALLED );

    // Initialize the driver layer
    MLDLSetDefaultParams();

    if (MLDLInit() != ML_SUCCESS)
	{
		printf("fail 1\n");
		return ML_ERROR;
	}

    FIFOParamInit();

    // Init vars.
    MLXInit();

    dmpInfo = dmpDefault();

    // DMP Init.
    result = MLDLDMPInit();
	if (result != ML_SUCCESS)
	{
		printf("fail 2\n");
	}

    // Send data out of the FIFO
    mlxData.mlDataMode = ML_DATA_FIFO;
    regs = DINADD;
    if ( MLDLSetMemoryMPU(KEY_CFG_17, 1, &regs) != ML_SUCCESS )
	{
		printf("fail 3\n");
        return ML_ERROR;
	}

    return result;
}

/**
 *  @brief  IMUsetBiasUpdateFunc is used to register which algorithms will be
 *          used to automatically reset the gyroscope bias.
 *          The engine ML_BIAS_UPDATE must be enabled for these algorithms to
 *          run.
 *
 *  @pre    IMUopen() must have been called. imuStart() 
 *          must <b>NOT</b> have been called.
 *
 *  @param  function    A function or bitwise OR of functions that determine
 *                      how the gyroscope bias will be automatically updated.
 *                      Functions include:
 *                      - ML_NONE or 0,
 *                      - ML_BIAS_FROM_NO_MOTION,
 *                      - ML_BIAS_FROM_GRAVITY,
 *                      - ML_BIAS_FROM_TEMPERATURE, and
 *                      - ML_ALL.
 *
 *  @return ML_SUCCESS if successful or Non-zero error code otherwise.
 */
tMLError IMUsetBiasUpdateFunc(unsigned short function)
{
    INVENSENSE_FUNC_START
    unsigned char regs[5] = {0};
    long tmp[3] = {0, 0, 0};
    tMLError retval = ML_SUCCESS;
    extern tMLCTRLParams mlCtrlParams;
    
    if ( isCompatible( IMU_C_OPEN | IMU_C_START, IMU_C_OPEN_CALLED ) != ML_SUCCESS)
        return ML_ERROR;    

    mlParams.biasUpdateFunc = function;

    if (mlParams.biasUpdateFunc & ML_BIAS_FROM_LPF) {
        regs[0] = DINA80 + 2;
        regs[1] = DINA2D;
        regs[2] = DINA55;
        regs[3] = DINA7D;
        regs[4] = 0x02;
    } else {
        regs[0] = DINA80 + 7;
        regs[1] = DINA2D;
        regs[2] = DINA35;
        regs[3] = DINA3D;
        regs[4] = 0x00;
        if (mlCtrlParams.functions & ML_DEAD_ZONE) {
            regs[4] = 0x08;
        }
    }
    retval += MLDLSetMemoryMPU(KEY_FCFG_5, 4, regs);
    retval += MLDLSetMemoryMPU(KEY_D_0_163, 1, &regs[4]);

    if (mlParams.biasUpdateFunc & ML_BIAS_FROM_GRAVITY) {
        regs[0] = DINAA0+5;
        regs[1] = DINA90+7;
        regs[2] = DINA80+10;
    } else {
        regs[0] = DINA28;
        regs[1] = DINA4C;
        regs[2] = DINA6C;
    }
    retval += MLDLSetMemoryMPU(KEY_CFG_2, 3, regs);

    if (mlParams.biasUpdateFunc & ML_BIAS_FROM_TEMPERATURE) {
        retval += IMUsetTemperatureSlope(mlxData.mlTempSlope);
    } else {
        retval += IMUsetTemperatureSlope(tmp);
    }

    return retval;
}

/**
 * Set the temperature slope
 * @param[in] data Length 3.
 */
tMLError IMUsetTemperatureSlope( const long *data )
{
    long sf;
    unsigned char reg;

    mlxData.mlTempSlope[0] = data[0];            
    mlxData.mlTempSlope[1] = data[1];
    mlxData.mlTempSlope[2] = data[2];
    sf = -mlxData.mlTempSlope[0]/1278;
    if (sf>127) {
        sf-=256;
    }
    reg = (unsigned char)sf; 
    MLSLSerialWriteSingle(MLDLGetMPUSlaveAddr(), MPUREG_05_RSVD, reg);
    sf = -mlxData.mlTempSlope[1]/1278;
    if (sf>127) {
        sf-=256;
    }
    reg = (unsigned char)sf;
    MLSLSerialWriteSingle(MLDLGetMPUSlaveAddr(), MPUREG_08_RSVD, reg);
    sf = -mlxData.mlTempSlope[2]/1278;
    if (sf>127) {
        sf-=256;
    }
    reg = (unsigned char)sf;
    MLSLSerialWriteSingle(MLDLGetMPUSlaveAddr(), MPUREG_0B_RSVD, reg);
    return ML_SUCCESS;
}

/**
 *  @brief  Used to register a callback function that
 *          will trigger when a change of motion state is detected.
 *
 *  @pre    IMUopen() must have been called. imuStart() 
 *          must <b>NOT</b> have been called.
 *
 *  @param  func    A user defined callback function accepting a
 *                  motionState parameter, the new motion state.
 *                  May be one of ML_MOTION or ML_NO_MOTION.
 *  @return ML_SUCCESS if successful or Non-zero error code otherwise.
 */
tMLError IMUsetMotionCallback(void (*func)(unsigned short motionState) )
{
    INVENSENSE_FUNC_START

    if ( isCompatible( IMU_C_OPEN | IMU_C_START, IMU_C_OPEN_CALLED ) != ML_SUCCESS)
        return ML_ERROR;    

    mlParams.motionCallback = func;

    return ML_SUCCESS;
}

/**
 *  @brief Start the DMP
 *
 *  @pre IMUstart() must have been called.
 *
 *  @return ML_SUCCESS if successful, or Non-zero error code otherwise.
 */
tMLError IMUstart(void)
{
    INVENSENSE_FUNC_START
    tMLError result;

    if ( isCompatible( IMU_C_OPEN | IMU_C_START, IMU_C_OPEN_CALLED ) != ML_SUCCESS)
        return ML_ERROR;    
    setCompatible( IMU_C_START_CALLED );

    result = MLDLDmpStart();

    return result;
}
/**
 *  @brief  Closes the motion sensor engine.
 *
 *  @pre IMUopen() Must First have been called.
 * 
 *  @code
 *     result = MLDmpClose();
 *     if (ML_SUCCESS != result) {
 *         // Handle the error case
 *     }
 *  @endcode
 *
 *  @return ML_SUCCESS, Non-zero error code otherwise.
 */
tMLError IMUclose(void)
{
    INVENSENSE_FUNC_START

    if ( isCompatible( IMU_C_OPEN, IMU_C_OPEN_CALLED ) != ML_SUCCESS)
        return ML_ERROR;    

    //put MPU back into passthrough mode
    MLDLSetI2CPassThrough(1);

    // Close Serial Interface
    IMUserialClose();

    clearCompatible();

    return ML_SUCCESS;
}

/**
 * @brief Enables the ML_CONTROL engine.
 *
 * @return ML_SUCCESS or non-zero error code
 */
tMLError MLEnableControl(void)
{
    INVENSENSE_FUNC_START
    
    if ( isCompatible( IMU_C_OPEN | IMU_C_START, IMU_C_OPEN_CALLED ) != ML_SUCCESS)
        return ML_ERROR;

    mlxData.mlEngineMask |= ML_CONTROL;
    return ML_SUCCESS;
}

/**
 * @brief Disables the ML_CONTROL engine.
 *
 * @return ML_SUCCESS or non-zero error code
 */
tMLError MLDisableControl(void)
{
    INVENSENSE_FUNC_START
    
    if ( isCompatible( IMU_C_OPEN | IMU_C_START, IMU_C_OPEN_CALLED | IMU_C_START_CALLED ) != ML_SUCCESS)
        return ML_ERROR;    

    mlxData.mlEngineMask &= (~ML_CONTROL);
    return ML_SUCCESS;
}

/** 
 * @internal
 * @brief   Update the ML Control engine.  This function should be called 
 *          every time new data from the MPU becomes available.
 *          Control engine outputs are written to the mlCtrlxData data 
 *          structure.
 * @return  error code.
**/
tMLError MLControlUpdate(void)
{
    INVENSENSE_FUNC_START
    unsigned char i;
    long gridTmp;
    long tmp;
    if ((mlxData.mlEngineMask & ML_CONTROL) && (mlxData.newData)) {
        for (i=0; i<4; i++) {
            if (mlCtrlParams.functions & ML_GRID) {
                if (mlCtrlParams.functions & ML_HYSTERESIS) {
                    mlCtrlxData.mlGridNumDMP[i] += mlCtrlxData.gridNumOffset[i];
                }
                mlCtrlxData.mlGridNumDMP[i] = mlCtrlxData.mlGridNumDMP[i]/2 + 1073741824;
                mlCtrlxData.controlInt[i] = (mlCtrlxData.mlGridNumDMP[i]%(128*mlCtrlParams.gridThreshold[i]))/128;
                gridTmp = mlCtrlxData.mlGridNumDMP[i]/(128*mlCtrlParams.gridThreshold[i]);
                tmp = 1+16777216L/mlCtrlParams.gridThreshold[i];
                mlCtrlxData.gridChange[i] = gridTmp-mlCtrlxData.lastGridNum[i];
                if (mlCtrlxData.gridChange[i]>tmp/2) {
                    mlCtrlxData.gridChange[i] = gridTmp-tmp-mlCtrlxData.lastGridNum[i];
                } else if (mlCtrlxData.gridChange[i]<-tmp/2) {
                    mlCtrlxData.gridChange[i] = gridTmp+tmp-mlCtrlxData.lastGridNum[i];
                }
                if ((mlCtrlParams.functions & ML_HYSTERESIS) && (mlCtrlxData.gridChange[i]!=0)) {
                    if (mlCtrlxData.gridChange[i]>0) {
                        mlCtrlxData.gridNumOffset[i] += 128*mlCtrlParams.gridThreshold[i];
                        mlCtrlxData.controlInt[i] = mlCtrlParams.gridThreshold[i]/2;
                    }
                    if (mlCtrlxData.gridChange[i]<0) {
                        mlCtrlxData.gridNumOffset[i] -= 128*mlCtrlParams.gridThreshold[i];
                        mlCtrlxData.controlInt[i] = mlCtrlParams.gridThreshold[i]/2;
                    }
                }
                mlCtrlxData.gridNum[i]+=mlCtrlxData.gridChange[i];
                if (mlCtrlxData.gridNum[i]>=mlCtrlParams.gridMaximum[i]) {
                    mlCtrlxData.gridNum[i] = mlCtrlParams.gridMaximum[i];
                    if (mlCtrlxData.controlInt[i]>=mlCtrlParams.gridThreshold[i]/2) {
                        mlCtrlxData.controlInt[i]=mlCtrlParams.gridThreshold[i]/2;
                    }
                } else if (mlCtrlxData.gridNum[i]<=0) {
                    mlCtrlxData.gridNum[i] = 0;
                    if (mlCtrlxData.controlInt[i]<mlCtrlParams.gridThreshold[i]/2) {
                        mlCtrlxData.controlInt[i]=mlCtrlParams.gridThreshold[i]/2;
                    }
                }
                mlCtrlxData.lastGridNum[i] = gridTmp;
                if ((mlCtrlParams.gridCallback) && (mlCtrlxData.gridChange[i]!=0)) {
                    mlCtrlParams.gridCallback((ML_CONTROL_1<<i), mlCtrlxData.gridNum, mlCtrlxData.gridChange);
                }

            } else {
                mlCtrlxData.controlInt[i] = mlCtrlxData.mlGridNumDMP[i];
            }

        }
    }

    return ML_SUCCESS;
}

/**
 * @internal
 * @brief   MLProcessInts contains default interrupt handling logic.  
 *          In the default case, signaled interrupts are recorded by the 
 *          MLDL and MLUpdateData periodically calls this function to check 
 *          which interrupts are signaled and process them appropriately.
 *          Setting interrupt callbacks via MLDLSetIntCallback should disable 
 *          the default processing.
**/
tMLError MLProcessInts(void)
{
    INVENSENSE_FUNC_START

    //Check if interrupt was from MPU
    if(MLDLGetIntTrigger(INTSRC_IMU)) {
        MLDLClearIntTrigger(INTSRC_IMU);

        //Check if interrupt was from FIFO
        if (mlxData.mlInterruptSources & ML_INT_FIFO) {
            int_fast8_t got,ftry=0;
                
            if (mlxData.mlDataMode & ML_DATA_FIFO) {
                ftry = 100;
            } else if (mlxData.mlDataMode & ML_DATA_POLL) {
                ftry = 1;
            }
            got = readAndProcessFIFO( ftry );
            if ( got > 0 )
                mlxData.newData = 1;
        }

        //Check if interrupt was from motion/no motion
        if (mlxData.mlInterruptSources & ML_INT_MOTION) {
            MLPollMotionStatus();
        }
        
    }
    
    return ML_SUCCESS;
}
/**
 * @internal
 * @brief   Retrieve and process DMP data.  
 *          mlxData.mlDataMode specifies the data delivery method. 
 *          This will be one of ML_DATA_FIFO or ML_DATA_POLL.
**/
tMLError MLGetDMPData(void)
{
    INVENSENSE_FUNC_START
    int_fast8_t got,ftry=0;

    if (mlxData.mlDataMode & ML_DATA_FIFO) {
        ftry = 100;
    } else if (mlxData.mlDataMode & ML_DATA_POLL) {
        ftry = 1;
    }
    got = readAndProcessFIFO( ftry );
    if ( got > 0 )
        mlxData.newData = 1;

    if(MLDLGetFifoStatus() != 0)
        return ML_ERROR;

    return ML_SUCCESS;
}

/**
 *  @brief  Updates all the realtime data from the motion algorithms.
 *
 *  @pre    IMUopen() must have been called. imuStart() 
 *          must <b>NOT</b> have been called.
 *
 *  @note   Functions like MLGetArray will return the same data if
 *          MLUpdateData is not called.
 *
 * @return
 * - ML_SUCCESS
 * - Non-zero error code
 */
tMLError IMUupdateData(void)
{
    INVENSENSE_FUNC_START
    static unsigned long polltimeNoMotion = 0;
    tMLError result=ML_SUCCESS;

    if ( isCompatible( IMU_C_OPEN | IMU_C_START, IMU_C_OPEN_CALLED | IMU_C_START_CALLED ) != ML_SUCCESS)
        return ML_ERROR;    

    //Process all interrupts
    MLProcessInts();

    //"No motion" and bias tracker manager
    if ( !(mlxData.mlInterruptSources & ML_INT_MOTION)) {
        // at the beginning and every second
        if(( polltimeNoMotion == 0) ||  
           (((unsigned int)MLOSGetTickCount() -  polltimeNoMotion) > 1000) ) { 
             polltimeNoMotion = MLOSGetTickCount();
            MLPollMotionStatus();
        }
    }

    //If FIFO interrupt is not set, poll DMP data
    if ((mlxData.mlInterruptSources & ML_INT_FIFO)==0) {
        result = MLGetDMPData();
    }

    //Update control data
    MLControlUpdate();

    mlxData.newData = 0;

    return result;
}



/**
 *  @brief  change default AUX slave address.
 *          configuration function to change default AUX slave address.
 *          Call MLSetGyroCalibration after changing the reference 
 *          accelerometer, because calibration and scale values will be reset 
 *          to the default for the chosen accelerometer.
 *  @pre    IMUopen() must have been called. imuStart() 
 *          must <b>NOT</b> have been called.
 *          See MLDLSetDefaultAUXSlaveAddr() for the legacy method for setting
 *          the accelerometer type at runtime (prior to MLDmpOpen() call).
 *  @param  auxSlaveAddr AUX slave address
 *  @return ML_SUCCESS if successful or Non-zero error code otherwise.
 */
tMLError MLSetAuxSlaveAddr(unsigned char auxSlaveAddr) 
{
    INVENSENSE_FUNC_START
    tMLError result = ML_SUCCESS;

    if ( isCompatible( IMU_C_OPEN | IMU_C_START, IMU_C_OPEN_CALLED ) != ML_SUCCESS)
        return ML_ERROR;    

    result += MLDLSetAuxParams(auxSlaveAddr);
    MLXAccelInit(auxSlaveAddr);
    result += MLDLDmpAccelInit();

    return result;
}

/** 
 *  @brief     Sets up the Accelerometer calibration.
 *
 *
 *  @pre    IMUopen() must have been called. imuStart() 
 *          must <b>NOT</b> have been called.
 *
 *  @param[in] range The range of th accelerometers in g's. An accelerometer
 *             that has a range of +2g's to -2g's should pass in 2.
 *  @param[in] orientation A 9 element matrix that represents how the accelerometers
 *             are oriented with respect to the device they are mounted in.
 *             A typical set of values are {1, 0, 0, 0, 1, 0, 0, 0, 1}. This
 *             example corresponds to a 3 by 3 identity matrix.
 *
 *  @return    ML_SUCCESS if successful or Non-zero error code otherwise.
 */
tMLError MLSetAccelCalibration( float range, float *orientation )
{
    INVENSENSE_FUNC_START
    int kk;
    float scale = range / 32768.f;
    unsigned char maxVal = 0;
    unsigned char tmpPtr = 0;
    unsigned char tmpSign = 0;
    int_fast8_t i = 0;
    long sf = 0;
    unsigned char regs[12] = {0}; // fixme

    if ( isCompatible( IMU_C_OPEN | IMU_C_START, IMU_C_OPEN_CALLED ) != ML_SUCCESS)
        return ML_ERROR;    

    if (scale==0) {
        mlxData.mlAccelSens = 0;
    }

    for ( kk=0; kk<9; ++kk )
        mlxData.mlAccelCal[kk] = (long)(scale * orientation[kk] * 1073741824L);
    
    {
        unsigned char tmpA = DINA4C;
        unsigned char tmpB = DINACD;
        unsigned char tmpC = DINA6C;
        regs[3] = DINA26;
        regs[4] = DINA46;
        regs[5] = DINA66;
        for (i=0; i<3; i++) {
            maxVal = 0;
            tmpSign = 0;
            if (mlxData.mlAccelCal[0+3*i]<0)
                tmpSign = 1;
            if (abs(mlxData.mlAccelCal[1+3*i])>abs(mlxData.mlAccelCal[0+3*i])) {
                maxVal = 1;
                if (mlxData.mlAccelCal[1+3*i]<0)
                    tmpSign = 1;
            }
            if (abs(mlxData.mlAccelCal[2+3*i])>abs(mlxData.mlAccelCal[1+3*i])) {
                tmpSign = 0;
                maxVal = 2;
                if (mlxData.mlAccelCal[2+3*i]<0)
                    tmpSign = 1;
            }
            if (maxVal==0) {
                regs[tmpPtr++] = tmpA;
                if (tmpSign)
                    regs[tmpPtr+2] |= 0x01;
            } else if (maxVal==1) {
                regs[tmpPtr++] = tmpB;
                if (tmpSign)
                    regs[tmpPtr+2] |= 0x01;
            } else {
                regs[tmpPtr++] = tmpC;
                if (tmpSign)
                    regs[tmpPtr+2] |= 0x01;
            }
        }
        if ( MLDLSetMemoryMPU(KEY_FCFG_2, 3, regs ) != ML_SUCCESS )
            return ML_ERROR;
        if ( MLDLSetMemoryMPU(KEY_FCFG_7, 3, &regs[3] ) != ML_SUCCESS )
            return ML_ERROR;
    }
    if (mlxData.mlAccelSens != 0) {
        sf = 1073741824/mlxData.mlAccelSens;
    } else {
        sf = 0;
    }
    regs[0] = (unsigned char)(sf/256);
    regs[1] = (unsigned char)(sf%256);
    if ( MLDLSetMemoryMPU(KEY_D_0_108, 2, regs) != ML_SUCCESS )
        return ML_ERROR;
    
    sf = mlxData.mlAccelSens/1024;
    regs[0] = (unsigned char)(sf/256);
    regs[1] = (unsigned char)(sf%256);
    if ( MLDLSetMemoryMPU(KEY_D_0_96, 2, regs) != ML_SUCCESS )
        return ML_ERROR;
    
    return ML_SUCCESS;
}

/**
 *  @brief     Sets up the Gyro calibration.
 *
 *
 *  @pre    IMUopen() must have been called. imuStart() 
 *          must <b>NOT</b> have been called.
 *
 *  @param[in] range The range of the gyros in degrees per second. A gyro
 *             that has a range of +2000 dps to -2000 dps should pass in 2000.
 *  @param[in] orientation A 9 element matrix that represents how the gyro
 *             are oriented with respect to the device they are mounted in.
 *             A typical set of values are {1, 0, 0, 0, 1, 0, 0, 0, 1}. This
 *             example corresponds to a 3 by 3 identity matrix.
 *
 *  @return    ML_SUCCESS if successful or Non-zero error code otherwise.
 */
tMLError MLSetGyroCalibration( float range, float *orientation )
{
    INVENSENSE_FUNC_START
    int kk;
    float scale = range / 32768.f;

    unsigned char regs[12] = {0};
    unsigned char maxVal = 0;
    unsigned char tmpPtr = 0;
    unsigned char tmpSign = 0;
    unsigned char i = 0;
    long sf = 0;

    if ( isCompatible( IMU_C_OPEN | IMU_C_START, IMU_C_OPEN_CALLED ) != ML_SUCCESS)
        return ML_ERROR;    

    mlxData.mlGyroSens = (long)(range * 32768);

    for( kk=0; kk<9; ++kk ) {
        mlxData.mlGyroCal[kk] = (long)(scale * orientation[kk]); // Deprecated
        mlxData.mlGyroOrient[kk] = (long)(orientation[kk] * (1L<<30));
    }


    {
        unsigned char tmpD = DINAC9;
        unsigned char tmpE = DINA2C;
        unsigned char tmpF = DINACB;
        regs[3] = DINA36;
        regs[4] = DINA56;
        regs[5] = DINA76;
        
        for (i=0; i<3; i++) {
            maxVal = 0;
            tmpSign = 0;
            if (mlxData.mlGyroOrient[0+3*i]<0)
                tmpSign = 1;
            if (abs(mlxData.mlGyroOrient[1+3*i])>abs(mlxData.mlGyroOrient[0+3*i])) {
                maxVal = 1;
                if (mlxData.mlGyroOrient[1+3*i]<0)
                    tmpSign = 1;
            }
            if (abs(mlxData.mlGyroOrient[2+3*i])>abs(mlxData.mlGyroOrient[1+3*i])) {
                tmpSign = 0;
                maxVal = 2;
                if (mlxData.mlGyroOrient[2+3*i]<0)
                    tmpSign = 1;
            }
            if (maxVal==0) {
                regs[tmpPtr++] = tmpD;
                if (tmpSign)
                    regs[tmpPtr+2] |= 0x01;
            } else if (maxVal==1) {
                regs[tmpPtr++] = tmpE;
                if (tmpSign)
                    regs[tmpPtr+2] |= 0x01;
            } else {
                regs[tmpPtr++] = tmpF;
                if (tmpSign)
                    regs[tmpPtr+2] |= 0x01;
            }
        }
        if ( MLDLSetMemoryMPU(KEY_FCFG_1, 3, regs ) != ML_SUCCESS )
            return ML_ERROR;
        if ( MLDLSetMemoryMPU(KEY_FCFG_3, 3, &regs[3] ) != ML_SUCCESS )
            return ML_ERROR;
        
        //sf = (gyroSens) * (0.5 * (pi/180) / 200.0) * 16384
        sf = (long)(((long long)mlxData.mlGyroSens*767603923LL)/1073741824LL);
        regs[0] = (unsigned char)(sf/16777216L);
        regs[1] = (unsigned char)((sf/65536L)%256);
        regs[2] = (unsigned char)((sf/256)%256);
        regs[3] = (unsigned char)(sf%256);
        if ( MLDLSetMemoryMPU(KEY_D_0_104, 4, regs) != ML_SUCCESS )
            return ML_ERROR;
        
        if (mlxData.mlGyroSens!=0) {
            sf = (long)((long long)23832619764371LL/mlxData.mlGyroSens);
        } else {
            sf = 0;
        }
        regs[0] = (unsigned char)(sf/16777216L);
        regs[1] = (unsigned char)((sf/65536L)%256);
        regs[2] = (unsigned char)((sf/256)%256);
        regs[3] = (unsigned char)(sf%256);
        if ( MLDLSetMemoryMPU(KEY_D_0_24, 4, regs) != ML_SUCCESS )
            return ML_ERROR;
    }
    return ML_SUCCESS;
}


/**
 * @brief   Set the interrupt sources that the ML should respond to.
 *          The available sources are:
 *          - ML_INT_MOTION
 *          - ML_INT_FIFO
 *
 * @param   interrupts  A bit field specifying the interrupt sources.
 *
 * @return  ML_SUCCESS on success, ML error code on any failure.
 */
tMLError MLSetInterrupts(uint_fast8_t interrupts)
{
    INVENSENSE_FUNC_START
    unsigned char  regs[2] = {0};
    unsigned short intMask = (ML_INT_MOTION|
                              ML_INT_FIFO);

    if ( isCompatible( IMU_C_OPEN | IMU_C_START, IMU_C_OPEN_CALLED ) != ML_SUCCESS)
        return ML_ERROR;    

    if ((interrupts & (~intMask)) != 0)
        return ML_ERROR_INVALID_PARAMETER;

    if (interrupts & ML_INT_FIFO) {
        regs[0] = DINAFE;
    } else {
        regs[0] = DINAD8;
    }
    if ( MLDLSetMemoryMPU(KEY_CFG_6, 1, regs) != ML_SUCCESS )
        return ML_ERROR;

    mlxData.mlInterruptSources = interrupts;

    return ML_SUCCESS;
}
/**
 * @internal
 * @brief   Initialize MLX data with values specific to the hardware in use.
 *          This should be called every time a new default accelerometer 
 *          type is specified in MLSetDefaultAuxSlaveAddr to reset the 
 *          internal variables involved.
 * @param   auxSlaveAddr    AUX slave address
**/
void MLXAccelInit(unsigned char auxSlaveAddr)
{
    INVENSENSE_FUNC_START
//     if (auxSlaveAddr == KIONIX_AUX_SLAVEADDR) {
        mlxData.mlAccelSens = (int)65536;
//     }
    
    // Default assumes orientation the same as the gyros
    mlxData.mlAccelCal[0] = mlxData.mlAccelSens;   // mlRotMatrix[0]
    mlxData.mlAccelCal[4] = mlxData.mlAccelSens;   // mlRotMatrix[4]
    mlxData.mlAccelCal[8] = mlxData.mlAccelSens;   // mlRotMatrix[8]
}
/**
 * @internal
 * @brief   Manually update the motion/no motion status.  This is a 
 *          convienence function for implementations that do not wish to use 
 *          MLUpdateData.  
 *          This function can be called periodically to check for the 
 *          'no motion' state and update the internal motion status and bias 
 *          calculations.
**/
tMLError MLPollMotionStatus(void)
{
    INVENSENSE_FUNC_START
    unsigned char regs[3] = {0};
    unsigned short motionFlag = 0;

    if (mlxData.mlEngineMask & ML_MOTION_DETECT) {
        if ( MLDLGetMemoryMPU(KEY_D_1_98, 2, regs) != ML_SUCCESS )
            return ML_ERROR;
        motionFlag = (unsigned short)regs[0]*256 + (unsigned short)regs[1];

        if (motionFlag == 1536) {
            if (mlxData.mlMotionState==ML_MOTION) {
                MLUpdateBias();

                regs[0] = DINAD8 + 1;
                regs[1] = DINA0C;
                regs[2] = DINAD8 + 2;
                if ( MLDLSetMemoryMPU( KEY_CFG_18, 3, regs) != ML_SUCCESS )
                    return ML_ERROR;

                regs[0] = 0;
                regs[1] = 5;
                if ( MLDLSetMemoryMPU( KEY_D_1_106, 2, regs) != ML_SUCCESS )
                    return ML_ERROR;

                //Trigger no motion callback
                mlxData.mlMotionState = ML_NO_MOTION;
                mlxData.mlFlags[ML_MOTION_STATE_CHANGE] = ML_NO_MOTION;
                if (mlParams.motionCallback) {
                    mlParams.motionCallback(ML_NO_MOTION);
                }
            }
        }
        if (motionFlag == 5) {
            if (mlxData.mlMotionState==ML_NO_MOTION) {
                regs[0] = DINAD8 + 2;
                regs[1] = DINA0C;
                regs[2] = DINAD8 + 1;
                if ( MLDLSetMemoryMPU(KEY_CFG_18, 3, regs) != ML_SUCCESS )
                    return ML_ERROR;

                regs[0] = 6;
                regs[1] = 0;
                if ( MLDLSetMemoryMPU(KEY_D_1_106, 2, regs) != ML_SUCCESS )
                    return ML_ERROR;

                //Trigger no motion callback
                mlxData.mlMotionState = ML_MOTION;
                mlxData.mlFlags[ML_MOTION_STATE_CHANGE] = ML_MOTION;
                if (mlParams.motionCallback) {
                    mlParams.motionCallback(ML_MOTION);
                }
            }
        }
    }

    return ML_SUCCESS;
}
/**
 * @brief   Set the data source for the ML.
 *          Used to switch between FIFO data sent at a constant
 *          rate (ML_DATA_FIFO) and data sent through the FIFO
 *          at a variable rate with polling.
 *
 *  @pre    IMUopen() must have been called. imuStart() 
 *          must <b>NOT</b> have been called.
 *
 * @param   dataMode The data mode can be one of
 *                   - ML_DATA_FIFO
 *                   - ML_DATA_POLL
 *                   - 0 for none.
 *
 * @return  ML_SUCCESS if successful, error code on any error.
 */
tMLError MLSetDataMode(unsigned short dataMode)
{
    INVENSENSE_FUNC_START
    unsigned char regs[1];

    if ( isCompatible( IMU_C_OPEN | IMU_C_START, IMU_C_OPEN_CALLED ) != ML_SUCCESS)
        return ML_ERROR;    

    /* check mutual exclusivity of data modes */
    if ( (dataMode & ML_DATA_FIFO) && (dataMode & ML_DATA_POLL) ) {
        return ML_ERROR_INVALID_PARAMETER;
    }
    /* check if dataMode contains other spurious bits other than 
     * ML_DATA_FIFO and ML_DATA_POLL */
    if ( (dataMode & (~(ML_DATA_FIFO|
                        ML_DATA_POLL))) != 0 ) {
        return ML_ERROR_INVALID_PARAMETER;
    }

    mlxData.mlDataMode = dataMode;

    if (mlxData.mlDataMode & ML_DATA_FIFO) {
        regs[0] = DINA18;
    } else {
        regs[0] = DINA08;
    }
    if ( MLDLSetMemoryMPU(KEY_CFG_19, 1, regs) != ML_SUCCESS )
        return ML_ERROR;

    if (mlxData.mlDataMode & ML_DATA_POLL) {
        regs[0] = 1;
        if ( MLDLSetMemoryMPU(KEY_D_1_179, 1, regs) != ML_SUCCESS )
            return ML_ERROR;
    }

    if ((mlxData.mlDataMode & ML_DATA_FIFO) || 
        (mlxData.mlDataMode & ML_DATA_POLL)) {
        regs[0] = DINADD;
        if ( MLDLSetMemoryMPU(KEY_CFG_17, 1, regs) != ML_SUCCESS )
            return ML_ERROR;
    } else {
        regs[0] = DINAA0+3;
        if ( MLDLSetMemoryMPU(KEY_CFG_17, 1, regs) != ML_SUCCESS )
            return ML_ERROR;
    }


    return ML_SUCCESS;
}
/**
 * @internal
 * @brief   Main entry point for the ML Bias engine.
 *          MLUpdateBias contains the bias calculation state machine.
 *          The global array saveData is used for the calculations if
 *          ML_BIAS_FROM_NO_MOTION is set.
 *          This array is only populated in MLGetDMPData.
 *          Thus, ML_BIAS_FROM_NO_MOTION will not work if FIFO data is used.
 *
 * @return  Zero for success; ML error code on any failure.
**/
unsigned char MLUpdateBias(void)
{
    INVENSENSE_FUNC_START
    unsigned char i;
    unsigned char regs[12] = {0};
    long biasTmp[3],biasTmp2[3];
    long biasPrev[3] = {0};
    extern tWriteBurst WriteBurst;
    extern tReadBurst  ReadBurst;

    if (mlParams.biasUpdateFunc & ML_BIAS_FROM_NO_MOTION) {

        //Reset bias
        regs[0] = DINAA0 + 3;
        if ( MLDLSetMemoryMPU( KEY_FCFG_6, 1, regs) != ML_SUCCESS )
            return ML_ERROR;

        if ( MLDLGetMemoryMPU(KEY_D_1_244, 12, regs) != ML_SUCCESS )
            return ML_ERROR;

        for ( i=0; i<3; i++ ) {
            biasTmp2[i] = (((long)regs[i*4]<<24)+((long)regs[i*4+1]<<16) + ((long)regs[i*4+2]<<8)+ ((long)regs[i*4+3]));
        }
        // Rotate bias vector by the transpose of the orientation matrix
	// m0 m3 m6     in0   out0
	// m1 m4 m7  *  in1 = out1
	// m2 m5 m8     in2   ou2
	// 
	// out0 = in0 * m0 + in1 * m3 + in2 * m6
	// out1 = in0 * m1 + in1 * m4 + in2 * m7
	// out2 = in0 * m2 + in1 * m5 + in2 * m8
        for ( i=0; i<3; ++i ) {
            biasTmp[i] = (long)(biasTmp2[0] * (float)mlxData.mlGyroOrient[i] / (1L<<30) +
                biasTmp2[1] * (float)mlxData.mlGyroOrient[i+3] / (1L<<30) +
                biasTmp2[2] * (float)mlxData.mlGyroOrient[i+6] / (1L<<30));
        }
        regs[0] = DINAA0 + 15;
        if ( MLDLSetMemoryMPU( KEY_FCFG_6, 1, regs) != ML_SUCCESS )
            return ML_ERROR;

        ReadBurst(MLDLGetMPUSlaveAddr(), MPUREG_X_OFFS_USRH, 6, regs);

        for( i=0 ; i<3 ; i++ ) {
            biasTmp[i]/=1430;
            biasPrev[i] = (long)regs[2*i]*256+(long)regs[2*i+1];
            if (biasPrev[i]>32767)
                biasPrev[i]-=65536L;
        }
        for (i=0; i<3; i++) {
            biasTmp[i]=biasPrev[i]-biasTmp[i];
            mlxData.mlBias[i] = -biasTmp[i]*2000;
            if (biasTmp[i]<0) 
                biasTmp[i]+=65536;
            regs[2*i] = (unsigned char)(biasTmp[i]/256);
            regs[2*i+1] = (unsigned char)(biasTmp[i]%256);
        }
        biasTmp[0] = 0;
        biasTmp[1] = 0;
        biasTmp[2] = 0;
        if ( MLDLSetMemoryMPU( KEY_D_0_36, 12, (unsigned char*)biasTmp ) != ML_SUCCESS )
            return ML_ERROR;
        if ( MLDLSetMemoryMPU( KEY_D_0_52, 12, (unsigned char*)biasTmp ) != ML_SUCCESS )
            return ML_ERROR;
        WriteBurst(MLDLGetMPUSlaveAddr(), MPUREG_X_OFFS_USRH, 6, regs);
        ReadBurst(MLDLGetMPUSlaveAddr(), MPUREG_TEMP_OUT_H, 2, regs);
        if ( MLDLSetMemoryMPU(KEY_DMP_PREVPTAT, 2, regs) != ML_SUCCESS )
            return ML_ERROR;
    }
    return ML_SUCCESS;
}




  /*********************/
 /** \}*/ /* defgroup */
/*********************/
