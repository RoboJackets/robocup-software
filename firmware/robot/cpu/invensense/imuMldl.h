/*******************************************************************************
 * Copyright (c) 2010 InvenSense Corporation, All Rights Reserved.
 * See license.txt for license.
 ******************************************************************************/
#ifndef INVENSENSE_IMU_MLDL_H__
#define INVENSENSE_IMU_MLDL_H__

#include "mltypes.h"
#include "imuMlsl.h"

#define MLDL_SUCCESS 0  // Success
#define MLDL_ERROR 1    // Generic Error

#define KIONIX_AUX_SLAVEADDR 0x0F
#define ADXL345_AUX_SLAVEADDR 0x53

#define INVALID_SLAVE_ADDR 0x00

#define DEFAULT_MPU_SLAVEADDR 0x69
#define SERIAL_I2C 0

#define DATASRC_IMMEDIATE 0  // Return data immediately
#define DATASRC_FIFO 2       // Use FIFO for data

#define INTPIN_MPU 0

#define INTLOGIC_HIGH 0
#define INTLOGIC_LOW 1

#define INTTYPE_PUSHPULL 0
#define INTTYPE_OPENDRAIN 1

#define INTLATCH_DISABLE 0
#define INTLATCH_ENABLE 1

#define MPUINT_MPU_READY 0x04
#define MPUINT_DMP_DONE 0x02
#define MPUINT_DATA_READY 0x01

#define INTLATCHCLEAR_READSTATUS 0
#define INTLATCHCLEAR_ANYREAD 1

#define INT_CLEAR 0
#define INT_TRIGGERED 1

#define MAX_FIFO_LENGTH 100  // 10 sensor registers, 2 bytes each
#define FIFO_HW_SIZE (512)   // FIFO hw is 512 bytes

#define DMP_DONTRUN 0
#define DMP_RUN 1

/*---- MPU filter selections ----*/
typedef enum {
    MPUFILTER_256HZ_NOLPF2 = 0,
    MPUFILTER_188HZ,
    MPUFILTER_98HZ,
    MPUFILTER_42HZ,
    MPUFILTER_20HZ,
    MPUFILTER_10HZ,
    MPUFILTER_5HZ,
    MPUFILTER_2100HZ_NOLPF,
    NUM_OF_FILTERS
} MPU_FILTER;

/*---- MPU full scale selections ----*/
typedef enum {
    MPUFS_250DPS = 0,
    MPUFS_500DPS,
    MPUFS_1000DPS,
    MPUFS_2000DPS,
    NUM_OF_FS
} MPU_FULLSCALE;

/*---- MPU clock source settings ----*/
typedef enum {
    CLK_INTERNAL = 0,
    CLK_PLLGYROX,
    CLK_PLLGYROY,
    CLK_PLLGYROZ,
    CLK_PLLEXT32K,
    CLK_PLLEXT19M,
    CLK_RESERVED,
    CLK_STOP,
    NUM_OF_CLKS
} CLOCK_SEL;

/* --------------- */
/* - Structures. - */
/* --------------- */

typedef struct {
    /*---- basic configuration settings ----*/
    unsigned char mpuSlaveAddr;
    unsigned char auxSlaveAddr;
    unsigned char serialInterface;
    unsigned char autoProcess;
    unsigned char clkSource;
    unsigned char
        dataSource;  // immediate, new data only, or FIFO (used by MLDLGetData)
    unsigned int dataProcessed;
    unsigned char intTrigger[NUM_OF_INTSOURCES];
    unsigned char runDMP;  // non-zero if using DMP
    unsigned char i2cPassThrough;
    /*---- variables used during processing ----*/
    unsigned char fifoRecordLength;  // length of FIFO record when using FIFO
    short fifoCount;                 // number of bytes in FIFO after last read
    unsigned char rawDataStart;  // starting register to read for raw data mode
    unsigned char rawDataLen;    // length of data to read for raw data mode
    unsigned short fifoError;
    unsigned short compassPresent;
} tMLDLData,      // new type definition
    MLDL_Data_t;  // background-compatible type definition

#ifdef __cplusplus
extern "C" {
#endif
void setGetAddress(unsigned short (*func)(unsigned short key));
tMLError MLDLSetAuxParams(unsigned char auxSlaveAddr);
unsigned char MLDLGetDefaultAuxSlaveAddr(void);
void MLDLSetDefaultParams(void);
tMLError MLDLInit(void);
tMLError MLDLCfgHardware(unsigned char auxSlaveAddr, unsigned char mpuSlaveAddr,
                         unsigned char serialInterface);
unsigned char MLDLGetMPUSlaveAddr();
unsigned char MLDLGetIntTrigger(unsigned char index);
void MLDLClearIntTrigger(unsigned char index);
tMLError MLDLDmpStart(void);
tMLError MLDLCtrlDmp(unsigned char enableRun, unsigned char enableFIFO,
                     unsigned short startAddress);
tMLError MLDLCfgInt(unsigned char intPin, unsigned char logicalLevel,
                    unsigned char pinType, unsigned char latch,
                    unsigned char latchClear, unsigned char triggers);
unsigned char MLDLGetIntStatus(unsigned char intPin);
tMLError MLDLIntHandler(INT_SOURCE intSource);
tMLError MLDLCfgSamplingMPU(unsigned char lpf, unsigned char fullScale,
                            unsigned char divider);
tMLError MLDLPowerMgmtMPU(unsigned char sleep, unsigned char disable_gx,
                          unsigned char disable_gy, unsigned char disable_gz);
tMLError MLDLClockSource(unsigned char clkSource);
unsigned short MLDLGetDMPData(unsigned char src, unsigned char* buffer,
                              unsigned char len);
tMLError MLDLSetI2CPassThrough(unsigned char enable);
short MLDLGetFifoStatus(void);
tMLError MLDLSetRegisterMPU(unsigned char reg, unsigned char value);
unsigned char MLDLGetRegistersMPU(unsigned char reg, unsigned char length);
tMLError MLDLGetMemoryMPU(unsigned short key, unsigned short length,
                          unsigned char* buffer);
tMLError MLDLSetMemoryMPU(unsigned short key, unsigned short length,
                          const unsigned char* buffer);
tMLError MLDLGetFifoLength(unsigned short* len);
tMLError MLDLReadFifo(unsigned char* data, unsigned short len);
unsigned char MPUGetRegisterShadow(unsigned char reg);
tMLError loadDMP(const MLU8* buffer, unsigned short length,
                 unsigned short startAddress);
tMLError MLDLDMPInit(void);
tMLError MLDLDmpAccelInit();

#ifdef __cplusplus
}
#endif

#endif  // INVENSENSE_IMU_MLDL_H__
