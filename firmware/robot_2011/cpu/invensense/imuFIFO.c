/*******************************************************************************
 * Copyright (c) 2010 InvenSense Corporation, All Rights Reserved.
 * See license.txt for license.
 ******************************************************************************/
#include "imuFIFO.h"
#include "imuSetup.h"
#include "mlcontrol.h"
#include "dmpKey.h"
#include "imuMldl.h"
#include <string.h>
#include "imuCompatibility.h"

/**
 *   @defgroup MLFIFO
 *   @brief FIFO Interface.
 *
 *   @{
 *       @file imuFIFO.c
 *       @brief FIFO Interface.
**/


static tMLError IMUsetFIFOFooter();

extern tMLCTRLXData mlCtrlxData;
extern tMLXData mlxData;

typedef struct _tFIFOData {
    unsigned short mlFIFORate;
    unsigned short mlFIFOPacketSize;
    long mlQuat[4];
    long mlLinearAccBody[3];
    long mlLinearAccWorld[3];
    long mlCalibratedData[6];
    long mlTemperature;
    long mlSensorData[6];
#define NUMFIFOELEMENTS 10
    uint_fast8_t mlFIFODataConfig[NUMFIFOELEMENTS];
    void (*FIFOProcessCB)(void);
} tFIFOData;

tFIFOData FIFOData;

/**
 * @internal
 * Initializes all the internal static variables for the FIFO module.
 * Should be called by the initialization routine such as IMUopen()
 */
void FIFOParamInit()
{
    memset( &FIFOData, 0, sizeof( tFIFOData ) );
    FIFOData.mlQuat[0] = 1073741824L;             // mlQuat[0]
    FIFOData.mlFIFORate = 20;
}

/** @internal
 * Reads and processes FIFO data.
 * @param[in] numPackets Number of FIFO packets to try to read. You should
 *        use a large number here, such as 100, if you want to read all
 *        the full packets in the FIFO, which is typical operation.
 * @return Returns number of packets actually read.
 */
int_fast8_t readAndProcessFIFO( int_fast8_t numPackets )
{
    unsigned char buf[MAX_FIFO_LENGTH];
    int_fast8_t processed = 0;

    for (processed=0; processed<numPackets; ++processed) {
        if ( MLDLGetDMPData(DATASRC_FIFO, buf, (unsigned char)FIFOData.mlFIFOPacketSize) 
             != FIFOData.mlFIFOPacketSize-2 )
            return processed;
        MLProcessFIFOData(buf);
    }
    return processed;
}

/**
 *  @brief  MLSetProcessedFIFOCallback is used to set a callback when a FIFO data has
 *          been processed. This is useful because system slow down may cause more than
 *          one FIFO buffer to be processed for a single interrupt. This would keep things
 *          from dropping data.
 *
 *  @param[in]  func    A user defined callback function.
 *
 *  @return ML_SUCCESS if successful, or non-zero error code otherwise.
 */
tMLError MLSetProcessedFIFOCallback(void (*func)(void) )
{
    INVENSENSE_FUNC_START

    if ( isCompatible( IMU_C_OPEN | IMU_C_START, IMU_C_OPEN_CALLED ) != ML_SUCCESS)
        return ML_ERROR;    

    FIFOData.FIFOProcessCB = func;

    return ML_SUCCESS;
}

/**
 * @internal
 * @brief   Process data from the dmp read via the fifo.  Takes a buffer 
 *          filled with bytes read from the DMP FIFO. 
 *          Calculates the motion parameters from that data and stores the 
 *          results in an internal data structure.
 * @param[in]   dmpData Pointer to the buffer containing the fifo data.
 * @return  error code.
**/
tMLError MLProcessFIFOData(const unsigned char* dmpData)
{           
    INVENSENSE_FUNC_START
    int_fast8_t FIFOPtr = 0; 
    int_fast8_t i;
    
    if (FIFOData.mlFIFODataConfig[ML_QUATERNION] & ML_32_BIT) {
        for (i=0; i<4; i++) {
            if ((FIFOData.mlFIFODataConfig[ML_QUATERNION]>>i) & 0x0001) {
                FIFOData.mlQuat[i] = (((long)dmpData[FIFOPtr]<<24) +
                                      ((long)dmpData[FIFOPtr+1]<<16) +
                                      ((long)dmpData[FIFOPtr+2]<<8)+
                                      ((int)dmpData[FIFOPtr+3]));
                FIFOPtr += 4;
            }
        }
    } else if (FIFOData.mlFIFODataConfig[ML_QUATERNION] & ML_16_BIT) {
        for (i=0; i<4; i++) {
            if ((FIFOData.mlFIFODataConfig[ML_QUATERNION]>>i) & 0x0001) {
                FIFOData.mlQuat[i] = (((long)dmpData[FIFOPtr]<<24) +
                                      ((long)dmpData[FIFOPtr+1]<<16));
                FIFOPtr += 2;
            }
        }
    }
    if (FIFOData.mlFIFODataConfig[ML_GYROS] & ML_32_BIT) {
        for (i=0; i<3; i++) {
            if ((FIFOData.mlFIFODataConfig[ML_GYROS]>>i) & 0x0001) {
                FIFOData.mlCalibratedData[i] = (((long)dmpData[FIFOPtr]<<24) +
                                                ((long)dmpData[FIFOPtr+1]<<16) +
                                                ((long)dmpData[FIFOPtr+2]<<8) +
                                                ((int)dmpData[FIFOPtr+3]));                                
                FIFOData.mlCalibratedData[i] =  (long)(((long long)FIFOData.mlCalibratedData[i]*65536L) /
                                                       ML_CALIBRATED_GYRO_SCALE);
                
                FIFOPtr += 4;
            }
        }
    } else if (FIFOData.mlFIFODataConfig[ML_GYROS] & ML_16_BIT) {
        for (i=0; i<3; i++) {
            if ((FIFOData.mlFIFODataConfig[ML_GYROS]>>i) & 0x0001) {
                FIFOData.mlCalibratedData[i] = (((long)dmpData[FIFOPtr]<<24) +
                                                ((long)dmpData[FIFOPtr+1]<<16));                                
                FIFOData.mlCalibratedData[i] =  (long)(((long long)FIFOData.mlCalibratedData[i]*65536L) /
                                                       ML_CALIBRATED_GYRO_SCALE);
                
                FIFOPtr += 2;
            }
        }
    }
    if (FIFOData.mlFIFODataConfig[ML_ACCELS] & ML_32_BIT) {
        for (i=0; i<3; i++) {
            if ((FIFOData.mlFIFODataConfig[ML_ACCELS]>>i) & 0x0001) {
                FIFOData.mlCalibratedData[i+3] = (((long)dmpData[FIFOPtr]<<24) +
                                                  ((long)dmpData[FIFOPtr+1]<<16) +
                                                  ((long)dmpData[FIFOPtr+2]<<8)+ 
                                                  ((int)dmpData[FIFOPtr+3]));
                if (mlxData.mlAccelSens != 0) {
                    FIFOData.mlCalibratedData[i+3]/=(0x20000000/mlxData.mlAccelSens);
                }
                FIFOPtr += 4;
            }
        }
    } else if (FIFOData.mlFIFODataConfig[ML_ACCELS] & ML_16_BIT) {
        for (i=0; i<3; i++) {
            if ((FIFOData.mlFIFODataConfig[ML_ACCELS]>>i) & 0x0001) {
                FIFOData.mlCalibratedData[i+3] = (((long)dmpData[FIFOPtr]<<24) +
                                                  ((long)dmpData[FIFOPtr+1]<<16));
                if (mlxData.mlAccelSens != 0) {
                    FIFOData.mlCalibratedData[i+3]/=(0x20000000UL/mlxData.mlAccelSens);
                }
                FIFOPtr += 2;
            }
        }
    }

    if (FIFOData.mlFIFODataConfig[ML_LINEAR_ACCELERATION] & ML_32_BIT) {
        for (i=0; i<3; i++) {
            if ((FIFOData.mlFIFODataConfig[ML_LINEAR_ACCELERATION]>>i) & 0x0001) {               
                FIFOData.mlLinearAccBody[i] = (((long)dmpData[FIFOPtr]<<24) +
                                               ((long)dmpData[FIFOPtr+1]<<16) +
                                               ((long)dmpData[FIFOPtr+2]<<8) +
                                               ((int)dmpData[FIFOPtr+3]));
                // 2^29 = 536870912
                if (mlxData.mlAccelSens != 0) {
                    FIFOData.mlLinearAccBody[i]/=(536870912L/mlxData.mlAccelSens);
                }
                FIFOPtr += 4;
            }
        }
    } else if (FIFOData.mlFIFODataConfig[ML_LINEAR_ACCELERATION] & ML_16_BIT) {
        for (i=0; i<3; i++) {
            if ((FIFOData.mlFIFODataConfig[ML_LINEAR_ACCELERATION]>>i) & 0x0001) {               
                FIFOData.mlLinearAccBody[i] = (((long)dmpData[FIFOPtr]<<24) +
                                               ((long)dmpData[FIFOPtr+1]<<16));
                // 2^29 = 536870912
                if (mlxData.mlAccelSens != 0) {
                    FIFOData.mlLinearAccBody[i]/=(536870912L/mlxData.mlAccelSens);
                }
                FIFOPtr += 2;
            }
        }
    }
    if (FIFOData.mlFIFODataConfig[ML_LINEAR_ACCELERATION_WORLD] & ML_32_BIT) {
        for (i=0; i<3; i++) {
            if ((FIFOData.mlFIFODataConfig[ML_LINEAR_ACCELERATION_WORLD]>>i) & 0x0001) {
                FIFOData.mlLinearAccWorld[i] = (((long)dmpData[FIFOPtr]<<24) +
                                                ((long)dmpData[FIFOPtr+1]<<16) +
                                                ((long)dmpData[FIFOPtr+2]<<8) +
                                                ((int)dmpData[FIFOPtr+3]));
                if (mlxData.mlAccelSens != 0) {
                    FIFOData.mlLinearAccWorld[i]/=(536870912L/mlxData.mlAccelSens);
                }
                if (i==2)
                    FIFOData.mlLinearAccWorld[2]-=65536L;
                FIFOPtr += 4;
            }
        }
    } else if (FIFOData.mlFIFODataConfig[ML_LINEAR_ACCELERATION_WORLD] & ML_16_BIT) {
        for (i=0; i<3; i++) {
            if ((FIFOData.mlFIFODataConfig[ML_LINEAR_ACCELERATION_WORLD]>>i) & 0x0001) {
                FIFOData.mlLinearAccWorld[i] = (((long)dmpData[FIFOPtr]<<24) +
                                                ((long)dmpData[FIFOPtr+1]<<16));
                if (mlxData.mlAccelSens != 0) {
                    FIFOData.mlLinearAccWorld[i]/=(536870912L/mlxData.mlAccelSens);
                }
                if (i==2)
                    FIFOData.mlLinearAccWorld[2]-=65536L;
                FIFOPtr += 2;
            }
        }
    }
    
    if (FIFOData.mlFIFODataConfig[ML_CONTROL_DATA] & ML_32_BIT) {
        for (i=0; i<4; i++) {
            if ((FIFOData.mlFIFODataConfig[ML_CONTROL_DATA]>>i) & 0x0001) {
                mlCtrlxData.mlGridNumDMP[i] = (((long)dmpData[FIFOPtr]<<24) +
                                               ((long)dmpData[FIFOPtr+1]<<16) +
                                               ((long)dmpData[FIFOPtr+2]<<8)+
                                               ((int)dmpData[FIFOPtr+3]));
                FIFOPtr += 4;
            }
        }
    } else if (FIFOData.mlFIFODataConfig[ML_CONTROL_DATA] & ML_16_BIT) {
        for (i=0; i<4; i++) {
            if ((FIFOData.mlFIFODataConfig[ML_CONTROL_DATA]>>i) & 0x0001) {
                mlCtrlxData.mlGridNumDMP[i] = (((long)dmpData[FIFOPtr]<<24) +
                                               ((long)dmpData[FIFOPtr+1]<<16));
                FIFOPtr += 2;
            }
        }
    }

    if ((FIFOData.mlFIFODataConfig[ML_TEMPERATURE]) & 0x0001) {
        FIFOData.mlTemperature = (((int)dmpData[FIFOPtr]<<8)+((int)dmpData[FIFOPtr+1]));
        if (FIFOData.mlTemperature > 32768L)
            FIFOData.mlTemperature -= 65536L;
        FIFOData.mlTemperature = (FIFOData.mlTemperature+8000)*204;
        FIFOPtr += 2;
    }
    for (i=0; i<6; i++) {
        if ((FIFOData.mlFIFODataConfig[ML_RAW_DATA]>>i) & 0x0001) {
            // Kionics Accel Data is byte swapped
            FIFOData.mlSensorData[i] = (((int)dmpData[FIFOPtr+1]<<8)+((int)dmpData[FIFOPtr]));
            if (FIFOData.mlSensorData[i]>32768L)
                FIFOData.mlSensorData[i]-=65536L;
            FIFOPtr += 2;
        }
    }
    
    if (FIFOData.FIFOProcessCB)
        FIFOData.FIFOProcessCB();

    return ML_SUCCESS;
}

/** Sends quaternion data to the FIFO. Quaternion data is a 4 length vector
 *   of 32-bits. Should be called once after IMUopen() and before IMUstart().
 * @param[in] accuracy Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *            bit data.
 */
tMLError IMUsendQuaternionToFIFO(uint_fast8_t accuracy)
{
    INVENSENSE_FUNC_START
    unsigned char regs[5] = { DINAF8 + 1, DINA20, DINA28,
                              DINA30, DINA38};

    if ( isCompatible( IMU_C_OPEN | IMU_C_START, IMU_C_OPEN_CALLED ) != ML_SUCCESS)
        return ML_ERROR;    

    if ( accuracy == ML_16_BIT )
        regs[0] = DINAF8 + 2;

    if ( MLDLSetMemoryMPU(KEY_CFG_8, 5, regs) != ML_SUCCESS )
        return ML_ERROR;
    FIFOData.mlFIFODataConfig[ML_QUATERNION] = 0x000f | accuracy;
    return IMUsetFIFOFooter();
}

/** Sends raw data to the FIFO. 
 *  Should be called once after IMUopen() and before IMUstart().
 *  @param[in] elements Which of the 3 elements to send. Use ML_ALL for all of them
 *            or ML_ELEMENT_1, ML_ELEMENT_2, ML_ELEMENT_3 ... ML_ELEMENT_7 or'd together
 *            for a subset. The first element is temperature, the next 3 are gyro data,
 *            and the last 3 accel data.
 */
tMLError IMUsendRawToFIFO(uint_fast8_t elements)
{
    INVENSENSE_FUNC_START
    unsigned char regs[13] = {DINAF8+3, DINAF8+3, DINAA0+3,
                              DINAF8+2, DINAA0+3, DINAF8+3,
                              DINAA0+3, DINAF8+2, DINAA0+3,
                              DINAF8+3, DINAA0+3, DINAF8+2,
                              DINAA0+3};

    if ( isCompatible( IMU_C_OPEN | IMU_C_START, IMU_C_OPEN_CALLED ) != ML_SUCCESS)
        return ML_ERROR;    

    if (elements & ML_ELEMENT_1) {
        regs[0] = DINA20;
    }
    if (elements & ML_ELEMENT_2) {
        regs[2] = DINA20;
    }
    if (elements & ML_ELEMENT_3) {
        regs[4] = DINA28;
    }
    if (elements & ML_ELEMENT_4) {
        regs[6] = DINA28;
    }
    if (elements & ML_ELEMENT_5) {
        regs[8] = DINA30;
    }
    if (elements & ML_ELEMENT_6) {
        regs[10] = DINA30;
    }
    if (elements & ML_ELEMENT_7) {
        regs[12] = DINA38;
    }
    if ( MLDLSetMemoryMPU(KEY_CFG_15, 13, regs) != ML_SUCCESS )
        return ML_ERROR;
    if ( elements & 0x7e )
        FIFOData.mlFIFODataConfig[ML_RAW_DATA] = (0x3f & (elements>>1)) | ML_16_BIT;
    if ( elements & 1 )
        FIFOData.mlFIFODataConfig[ML_TEMPERATURE] = 1 | ML_16_BIT;

    return IMUsetFIFOFooter();
}
/** 
 *  @brief      Returns 1-element vector of temperature
 *  @param[out] data    1-element vector of temperature
 *  @return     0 on success or an error code.
 */
tMLError IMUgetTemperature(long *data)
{
    if ( data == NULL )
        return ML_ERROR;
    data[0] = FIFOData.mlTemperature;
    return ML_SUCCESS;
}
/** 
 *  @brief      Returns 6-element vector of gyro and accel data
 *  @param[out] data    6-element vector of gyro and accel data
 *  @return     0 on success or an error code.
 */
tMLError IMUgetSensorData(long *data)
{
    if ( data == NULL )
        return ML_ERROR;
    data[0] = FIFOData.mlSensorData[0];
    data[1] = FIFOData.mlSensorData[1];
    data[2] = FIFOData.mlSensorData[2];
    data[3] = FIFOData.mlSensorData[3];
    data[4] = FIFOData.mlSensorData[4];
    data[5] = FIFOData.mlSensorData[5];
    return ML_SUCCESS;
}
 
/** Sends data needed to compute the angular velocity in the world
* coordinate system to the FIFO.
* @param[in] accuracy 16-bit ML_16_BIT or 32-bit ML_32_BIT
* @return  error code.
*/
tMLError IMUsendAngularVelocityWorldToFIFO(uint_fast8_t accuracy)
{
    tMLError status;

    status = IMUsendQuaternionToFIFO(accuracy);
    if ( status )
        return status;
    status = IMUsendGyroToFIFO(ML_ALL, accuracy);
    return status;
}

tMLError IMUgetAngularVelocityWorld(long *data)
{
    long quat[4],gyro[3],rot[9];
    tMLError status;
    
    status = IMUgetQuaternion( quat );
    if ( status )
        return status;
    status = IMUgetGyro( gyro );
    if ( status )
        return status;
    quaternionToRotationMatrix( quat, rot );
    data[0] = (long)(((long long)rot[0]*gyro[0]+
                      (long long)rot[1]*gyro[1]+
                      (long long)rot[2]*gyro[2])>>30);
    data[1] = (long)(((long long)rot[3]*gyro[0]+
                      (long long)rot[4]*gyro[1]+
                      (long long)rot[5]*gyro[2])>>30);
    data[2] = (long)(((long long)rot[6]*gyro[0]+
                      (long long)rot[7]*gyro[1]+
                      (long long)rot[8]*gyro[2])>>30);
    return ML_SUCCESS;
}

/** Sends gyro data to the FIFO. Gyro data is a 3 length vector
 *  of 32-bits. Should be called once after IMUopen() and before IMUstart().
 *  @param[in] elements Which of the 3 elements to send. Use ML_ALL for all of them
 *            or ML_ELEMENT_1, ML_ELEMENT_2, ML_ELEMENT_3 or'd together
 *            for a subset.
 *  @param[in] accuracy Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *             bit data.
 */
tMLError IMUsendGyroToFIFO(uint_fast8_t elements, uint_fast8_t accuracy)
{
    INVENSENSE_FUNC_START
    unsigned char regs[4] = { DINAF8 + 1, DINA20, DINA28, DINA30};

    if ( isCompatible( IMU_C_OPEN | IMU_C_START, IMU_C_OPEN_CALLED ) != ML_SUCCESS)
        return ML_ERROR;    
    if ( accuracy == ML_16_BIT )
        regs[0] = DINAF8 + 2;
    if ( (elements & ML_ELEMENT_1) == 0 )
        regs[1] = DINAA0+3;
    if ( (elements & ML_ELEMENT_2) == 0 )
        regs[2] = DINAA0+3;
    if ( (elements & ML_ELEMENT_3) == 0 )
        regs[3] = DINAA0+3;
    
    if ( MLDLSetMemoryMPU(KEY_CFG_9, 4, regs) != ML_SUCCESS )
        return ML_ERROR;
    FIFOData.mlFIFODataConfig[ML_GYROS] = (0x0007 & elements) | accuracy;
    return IMUsetFIFOFooter();
}

/** Sends accelerometer data to the FIFO. Accelerometer data is a 3 length vector
 *  of 32-bits. Should be called once after IMUopen() and before IMUstart().
 *  @param[in] elements Which of the 3 elements to send. Use ML_ALL for all of them
 *            or ML_ELEMENT_1, ML_ELEMENT_2, ML_ELEMENT_3 or'd together
 *            for a subset.
 * @param[in] accuracy Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *            bit data.
 */
tMLError IMUsendAccelToFIFO(uint_fast8_t elements, uint_fast8_t accuracy)
{
    INVENSENSE_FUNC_START
    unsigned char regs[4] = { DINAF8 + 1, DINA28, DINA50,
                              DINA78};

    if ( isCompatible( IMU_C_OPEN | IMU_C_START, IMU_C_OPEN_CALLED ) != ML_SUCCESS)
        return ML_ERROR;    
    if ( accuracy == ML_16_BIT )
        regs[0] = DINAF8 + 2;
    if ( (elements & ML_ELEMENT_1) == 0 )
        regs[1] = DINAA0+3;
    if ( (elements & ML_ELEMENT_2) == 0 )
        regs[2] = DINAA0+3;
    if ( (elements & ML_ELEMENT_3) == 0 )
        regs[3] = DINAA0+3;
    
    if ( MLDLSetMemoryMPU(KEY_CFG_10, 4, regs) != ML_SUCCESS )
        return ML_ERROR;
    FIFOData.mlFIFODataConfig[ML_ACCELS] = (0x0007 & elements) | accuracy;
    return IMUsetFIFOFooter();
}


/** Sends linear accelerometer data to the FIFO. Linear accelerometer data is a
 *  3 length vector of 32-bits. It is the acceleration in the body frame
 *  with gravity removed.  Should be called once after IMUopen() and before IMUstart().
 *  @param[in] elements Which of the 3 elements to send. Use ML_ALL for all of them
 *            or ML_ELEMENT_1, ML_ELEMENT_2, ML_ELEMENT_3 or'd together
 *            for a subset.
 *  @param[in] accuracy Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *             bit data.
 */
tMLError IMUsendLinearAccelToFIFO(uint_fast8_t elements, uint_fast8_t accuracy)
{
    INVENSENSE_FUNC_START
    unsigned char regs[4] = { DINAF8 + 1, DINA28, DINA30, DINA38};

    if ( isCompatible( IMU_C_OPEN | IMU_C_START, IMU_C_OPEN_CALLED ) != ML_SUCCESS)
        return ML_ERROR;    
    if ( accuracy == ML_16_BIT )
        regs[0] = DINAF8 + 2;
    if ( (elements & ML_ELEMENT_1) == 0 )
        regs[1] = DINAA0+3;
    if ( (elements & ML_ELEMENT_2) == 0 )
        regs[2] = DINAA0+3;
    if ( (elements & ML_ELEMENT_3) == 0 )
        regs[3] = DINAA0+3;
    
    if ( MLDLSetMemoryMPU(KEY_CFG_12, 4, regs) != ML_SUCCESS )
        return ML_ERROR;
    FIFOData.mlFIFODataConfig[ML_LINEAR_ACCELERATION] = (0x0007 & elements) | accuracy;
    return IMUsetFIFOFooter();
}

/** Sends linear world accelerometer data to the FIFO. Linear world accelerometer 
 *  data is a 3 length vector of 32-bits. It is the acceleration in the world frame
 *  with gravity removed. Should be called once after IMUopen() and before IMUstart().
 *  @param[in] elements Which of the 3 elements to send. Use ML_ALL for all of them
 *            or ML_ELEMENT_1, ML_ELEMENT_2, ML_ELEMENT_3 or'd together
 *            for a subset.
 *  @param[in] accuracy Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *             bit data.
 */
tMLError IMUsendLinearAccelWorldToFIFO(uint_fast8_t elements, uint_fast8_t accuracy)
{
    INVENSENSE_FUNC_START
    unsigned char regs[6] = { DINAF8 + 1, DINA20, DINA90+11,
                              DINA20, DINA90+8,
                              DINA20};

    if ( isCompatible( IMU_C_OPEN | IMU_C_START, IMU_C_OPEN_CALLED ) != ML_SUCCESS)
        return ML_ERROR;    
    if ( accuracy == ML_16_BIT )
        regs[0] = DINAF8 + 2;
    if ( (elements & ML_ELEMENT_1) == 0 )
        regs[1] = DINAA0+3;
    if ( (elements & ML_ELEMENT_2) == 0 )
        regs[3] = DINAA0+3;
    if ( (elements & ML_ELEMENT_3) == 0 )
        regs[3] = DINAA0+3;
    
    if ( MLDLSetMemoryMPU(KEY_CFG_13, 6, regs) != ML_SUCCESS )
        return ML_ERROR;
    FIFOData.mlFIFODataConfig[ML_LINEAR_ACCELERATION_WORLD] = (0x0007 & elements) | accuracy;
    return IMUsetFIFOFooter();
}

/** Sends control data to the FIFO. Control data is a 4 length vector of 32-bits. 
 *  Should be called once after IMUopen() and before IMUstart().
 *  @param[in] elements Which of the 4 elements to send. Use ML_ALL for all of them
 *            or ML_ELEMENT_1, ML_ELEMENT_2, ML_ELEMENT_3, ML_ELEMENT_4 or'd together
 *            for a subset.
 *  @param[in] accuracy Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *             bit data.
 */
tMLError IMUsendControlDataToFIFO(uint_fast8_t elements, uint_fast8_t accuracy)
{
    INVENSENSE_FUNC_START
    unsigned char regs[5] = { DINAF8 + 1, DINA20, DINA28,
                              DINA30, DINA38};

    if ( isCompatible( IMU_C_OPEN | IMU_C_START, IMU_C_OPEN_CALLED ) != ML_SUCCESS)
        return ML_ERROR;    
    if ( accuracy == ML_16_BIT )
        regs[0] = DINAF8 + 2;
    if ( (elements & ML_ELEMENT_1) == 0 )
        regs[1] = DINAA0+3;
    if ( (elements & ML_ELEMENT_2) == 0 )
        regs[2] = DINAA0+3;
    if ( (elements & ML_ELEMENT_3) == 0 )
        regs[3] = DINAA0+3;
    if ( (elements & ML_ELEMENT_4) == 0 )
        regs[3] = DINAA0+3;
    
    if ( MLDLSetMemoryMPU(KEY_CFG_1, 5, regs) != ML_SUCCESS )
        return ML_ERROR;
    FIFOData.mlFIFODataConfig[ML_CONTROL_DATA] = (0x000f & elements) | accuracy;
    return IMUsetFIFOFooter();
}

/** 
 * @internal
 * Puts footer on FIFO data.
 */
static tMLError IMUsetFIFOFooter()
{
    unsigned char regs = DINA30;
    uint_fast8_t tmpCount;
    int_fast8_t i,j;

    if ( MLDLSetMemoryMPU(KEY_CFG_16, 1, &regs) != ML_SUCCESS )
        return ML_ERROR;
    FIFOData.mlFIFODataConfig[ML_FOOTER] = 0x0001 | ML_16_BIT;

    FIFOData.mlFIFOPacketSize = 0;
    for (i=0; i<NUMFIFOELEMENTS; i++) {
        tmpCount = 0;
        for (j=0; j<6; j++) {
            if ((FIFOData.mlFIFODataConfig[i]>>j) & 0x0001) {
                tmpCount += 2;
            }
        }
        if (FIFOData.mlFIFODataConfig[i] & ML_32_BIT) {
            tmpCount *= 2;
        }
        FIFOData.mlFIFOPacketSize += tmpCount;
    }

    return ML_SUCCESS;
}

/** 
 * @brief       Returns 3-element vector of acclerometer data in body frame.
 *
 * @param[out]  data    3-element vector of acclerometer data in body frame.
 *                      One g = 2^16.
 *  @return     0 on success or an error code.
 */
tMLError IMUgetAccel(long *data)
{
    if ( data == NULL )
        return ML_ERROR;
    data[0] = FIFOData.mlCalibratedData[3];
    data[1] = FIFOData.mlCalibratedData[4];
    data[2] = FIFOData.mlCalibratedData[5];
    return ML_SUCCESS;
}

/** 
 *  @brief      Returns 4-element quaternion vector.
 *  @param[out] data    4-element quaternion vector. One is scaled to 2^30.
 *  @return     0 on success or an error code.
 */
tMLError IMUgetQuaternion(long *data)
{
    if ( data == NULL )
        return ML_ERROR;
    data[0] = FIFOData.mlQuat[0];
    data[1] = FIFOData.mlQuat[1];
    data[2] = FIFOData.mlQuat[2];
    data[3] = FIFOData.mlQuat[3];
    return ML_SUCCESS;
}

/** 
 *  @brief      Returns 3-element vector of gyro data in body frame.
 *  @param[out] data    3-element vector of gyro data in body frame 
 *                      with gravity removed. One degree per second = 2^16.
 *  @return     0 on success or an error code.
 */
tMLError IMUgetGyro(long *data)
{
    if ( data == NULL )
        return ML_ERROR;
    data[0] = FIFOData.mlCalibratedData[0];
    data[1] = FIFOData.mlCalibratedData[1];
    data[2] = FIFOData.mlCalibratedData[2];
    return ML_SUCCESS;
}

/** 
 *  @brief      Returns 3-element vector of acclerometer data in body frame
 *              with gravity removed.
 *  @param[out] data    3-element vector of acclerometer data in body frame
 *                      with gravity removed. One g = 2^16.
 *  @return     0 on success or an error code.
 */
tMLError IMUgetLinearAccel(long *data)
{
    if ( data == NULL )
        return ML_ERROR;
    data[0] = FIFOData.mlLinearAccBody[0];
    data[1] = FIFOData.mlLinearAccBody[1];
    data[2] = FIFOData.mlLinearAccBody[2];
    return ML_SUCCESS;
}
 
/** 
 *  @brief      Returns 3-element vector of acclerometer data in world frame
 *              with gravity removed.
 *  @param[out] data    3-element vector of acclerometer data in world frame
 *                      with gravity removed. One g = 2^16.
 *  @return     0 on success or an error code.
 */
tMLError IMUgetLinearAccelWorld(long *data)
{
    if ( data == NULL )
        return ML_ERROR;
    data[0] = FIFOData.mlLinearAccWorld[0];
    data[1] = FIFOData.mlLinearAccWorld[1];
    data[2] = FIFOData.mlLinearAccWorld[2];
    return ML_SUCCESS;
}
 
/** 
 *  @brief      Returns 4-element vector of control data.
 *  @param[out] data    4-element vector of control data.
 *  @return     0 for succes or an error code.
 */
tMLError IMUgetControlData(long *data)
{
    if ( data == NULL )
        return ML_ERROR;
    data[0] = mlCtrlxData.mlGridNumDMP[0];
    data[1] = mlCtrlxData.mlGridNumDMP[1];
    data[2] = mlCtrlxData.mlGridNumDMP[2];
    data[3] = mlCtrlxData.mlGridNumDMP[3];
    return ML_SUCCESS;

}


/** 
 *  @brief      Returns 3-element vector of acclerometer data in body frame.
 *  @param[out] data    3-element vector of acclerometer data in body frame in g's.
 *  @return     0 for success or an error code.
 */
tMLError IMUgetAccelFloat(float *data)
{
    data[0] = FIFOData.mlCalibratedData[3]/65536.0f;
    data[1] = FIFOData.mlCalibratedData[4]/65536.0f;
    data[2] = FIFOData.mlCalibratedData[5]/65536.0f;
    return ML_SUCCESS;
}

/** 
 *  @brief      Returns 4-element quaternion vector.
 *  @param[out] data    4-element quaternion vector.
 *  @return     0 on success, an error code otherwise.
 */
tMLError IMUgetQuaternionFloat(float *data)
{
    data[0] = FIFOData.mlQuat[0]/1073741824.0f;
    data[1] = FIFOData.mlQuat[1]/1073741824.0f;
    data[2] = FIFOData.mlQuat[2]/1073741824.0f;
    data[3] = FIFOData.mlQuat[3]/1073741824.0f;
    return ML_SUCCESS;
}


/**
 * @brief   Command to put data in the FIFO at a particular rate.
 *
 *          The DMP will add fifo entries every fifoRate + 1 IMU cycles (200 Hz).
 *          The following values apply:
 *
 *          <TABLE>
 *          <TR><TD>fifoRate</TD><TD>DMP Sample Rate</TD><TD>FIFO update frequency</TD></TR>
 *          <TR><TD>0</TD><TD>200Hz</TD><TD>200Hz</TD></TR>
 *          <TR><TD>1</TD><TD>200Hz</TD><TD>100Hz</TD></TR>
 *          <TR><TD>2</TD><TD>200Hz</TD><TD>50Hz</TD></TR>
 *          <TR><TD>4</TD><TD>200Hz</TD><TD>40Hz</TD></TR>
 *          <TR><TD>9</TD><TD>200Hz</TD><TD>20Hz</TD></TR>
 *          <TR><TD>19</TD><TD>200Hz</TD><TD>10Hz</TD></TR>
 *          </TABLE>
 *
 *  @pre    imuOpen() must have been called. imuStart() 
 *          must <b>NOT</b> have been called.
 *
 * @param   fifoRate    Divider value - 1.  Output rate is 
 *          (DMP Sample Rate) / (fifoRate + 1).
 *
 * @return  ML_SUCCESS if successful, ML error code on any failure.
 */
tMLError IMUsetFIFORate(unsigned short fifoRate)
{
    INVENSENSE_FUNC_START
    unsigned char regs[2] = {0};

    if ( isCompatible( IMU_C_OPEN | IMU_C_START, IMU_C_OPEN_CALLED ) != ML_SUCCESS)
        return ML_ERROR;    

    FIFOData.mlFIFORate = fifoRate;

    regs[0] = fifoRate/256;
    regs[1] = fifoRate%256;
    
    return MLDLSetMemoryMPU(KEY_D_0_22, 2, regs);
}

/**
 * Converts a quaternion to a rotation matrix.
 * @param[in] quat 4-element quaternion in fixed point. One is 2^30.
 * @param[out] rot Rotation matrix in fixed point. One is 2^30.
 */
void quaternionToRotationMatrix( const long *quat, long *rot )
{
    rot[0] = (long)(((long long)quat[1]*quat[1]+
                     (long long)quat[0]*quat[0])/536870912L-1073741824L);
    rot[1] = (long)(((long long)quat[1]*quat[2]-
                     (long long)quat[3]*quat[0])/536870912L);
    rot[2] = (long)(((long long)quat[1]*quat[3]+
                     (long long)quat[2]*quat[0])/536870912L);
    rot[3] = (long)(((long long)quat[1]*quat[2]+
                     (long long)quat[3]*quat[0])/536870912L);
    rot[4] = (long)(((long long)quat[2]*quat[2]+
                     (long long)quat[0]*quat[0])/536870912L-1073741824L);
    rot[5] = (long)(((long long)quat[2]*quat[3]-
                     (long long)quat[1]*quat[0])/536870912L);
    rot[6] = (long)(((long long)quat[1]*quat[3]-
                     (long long)quat[2]*quat[0])/536870912L);
    rot[7] = (long)(((long long)quat[2]*quat[3]+
                     (long long)quat[1]*quat[0])/536870912L);
    rot[8] = (long)(((long long)quat[3]*quat[3]+
                     (long long)quat[0]*quat[0])/536870912L-1073741824L);
}

  /*********************/
 /** \}*/ /* defgroup */
/*********************/
