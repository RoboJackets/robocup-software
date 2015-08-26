/*******************************************************************************
 * Copyright (c) 2010 InvenSense Corporation, All Rights Reserved.
 * See license.txt for license.
 ******************************************************************************/
#include "imuMlos.h"
#include "mlinclude.h"
#include "imuMldl.h"
#include "mpuregs.h"
#include <string.h>
#include "imuSetup.h"
#include "dmpKey.h"
#include "imuMlsl.h"
#include <assert.h>
#include <stdio.h>

/**
 *   @defgroup MLDL
 *   @brief ML Driver Layer
 *
 *   @{
 *       @file imuMldl.c
 *       @brief ML Driver Layer
**/

extern tReadBurst ReadBurst;
extern tMLXData mlxData;

const unsigned char gFifoFooter[2] = {0xB2, 0x6A};

/* --------------------- */
/* -    Variables.     - */
/* --------------------- */

/*---- structure containing control variables used by MLDL ----*/
tMLDLData mldlData;
/*---- keep shadow copy of all MPU registers ----*/
unsigned char mpuRegister[NUM_OF_MPU_REGISTERS];
unsigned char mpuRegisterValid[NUM_OF_MPU_REGISTERS];

/* ---------------------- */
/* -  Static Functions. - */
/* ---------------------- */

static tMLError MLDLResetDmp(void);
static unsigned short MPUGetFIFO(unsigned char length, unsigned char* buffer);

unsigned short (*sGetAddress)(unsigned short key) = NULL;

/**
 *  @internal
 *  @brief Sets the function to use to convert keys to addresses. This
 *         will changed for each DMP code loaded.
 *  @param func   Function used to convert keys to addresses.
 */
void setGetAddress(unsigned short (*func)(unsigned short key)) {
    sGetAddress = func;
}

/**
 * @brief   change aux slave address and parameters.
 *          configuration function to change AUX slave address,
 *          no motion interrupt register and no motion mask.
 * @see     MLSetAuxSlaveAddr()
 * @param   auxSlaveAddr AUX slave address
 */
tMLError MLDLSetAuxParams(unsigned char auxSlaveAddr) {
    INVENSENSE_FUNC_START
    tMLError result;

    if (auxSlaveAddr != KIONIX_AUX_SLAVEADDR)
        return ML_ERROR;  // Accelerometer not supported.

    result = MLDLSetRegisterMPU(MPUREG_AUX_SLV_ADDR, auxSlaveAddr);

    /*---- change INTERNAL record of aux slave address value ----*/
    mldlData.auxSlaveAddr = auxSlaveAddr;

    return ML_SUCCESS;
}
/**
 *  @internal
 *  @brief   get default AUX slave address used by MLOpen().
 *  @return  AUX slave address.
 */
unsigned char MLDLGetDefaultAuxSlaveAddr(void) {
    INVENSENSE_FUNC_START
    /*---- get value ----*/
    return mldlData.auxSlaveAddr;
}

/**
 * @brief set default parameters.
 *        configuration function for setups that use dmp
 *        data from RAM memory.
 */
void MLDLSetDefaultParams(void) {
    INVENSENSE_FUNC_START
    /*---- default values for MLDL variables ----*/
    mldlData.auxSlaveAddr = KIONIX_AUX_SLAVEADDR;
    mldlData.mpuSlaveAddr = DEFAULT_MPU_SLAVEADDR;
    mldlData.serialInterface = SERIAL_I2C;
    mldlData.autoProcess = 0;
    mldlData.dataSource = DATASRC_FIFO;
    mldlData.fifoCount = 0;
    mldlData.fifoError = 0;
    mldlData.compassPresent = 0;
}

/**
 *
 *  @internal
 *  @brief  initializes the Driver Layer and should be called before using
 *          the driver functions.
 *  @return Zero if the command is successful,
 *          an error code otherwise.
 */
tMLError MLDLInit(void) {
    INVENSENSE_FUNC_START

    /*---- indicate that shadow registers are not valid ----*/
    memset(mpuRegisterValid, 0, NUM_OF_MPU_REGISTERS * sizeof(unsigned char));
    /*---- clear external interrupt status ----*/
    memset(mldlData.intTrigger, INT_CLEAR,
           NUM_OF_INTSOURCES * sizeof(unsigned char));

    MLDLCfgHardware(KIONIX_AUX_SLAVEADDR, DEFAULT_MPU_SLAVEADDR, SERIAL_I2C);

    // Configure Interrupt Status;
    return MLDLSetRegisterMPU(MPUREG_INT_CFG, MPUINT_DMP_DONE);
}

/** @internal
 *  @brief  Used to set the hardware configuration.
 *          Sets the hardware configuration, allowing the drivers to properly
 *          interface to all devices.
 *          For the slave addresses, the value represents bits 1 to 7 of the
 *          address.
 *          For example, if the slave address is 0010110b, then the slave
 *          address should be set to 0x16.
 *
 *  @param  auxSlaveAddr    The 7-bit AUX slave address.
 *  @param  mpuSlaveAddr    The 7-bin MPU slave address.
 *  @param  serialInterface I2C or SPI. Can be one of SERIAL_I2C or SERIAL_SPI.
 *
 *  @return Zero if the command is successful, an error code otherwise
 **/
tMLError MLDLCfgHardware(unsigned char auxSlaveAddr, unsigned char mpuSlaveAddr,
                         unsigned char serialInterface) {
    mldlData.auxSlaveAddr = auxSlaveAddr;
    mldlData.mpuSlaveAddr = mpuSlaveAddr;
    mldlData.serialInterface = serialInterface;

    /*---- power management operation ----*/
    MLDLPowerMgmtMPU(0, 0, 0,
                     0);  // power up MPU (it is currently in sleep mode)

    /*---- reset the MPU-3000 to make sure all settings are known ----*/
    MLDLSetRegisterMPU(MPUREG_PWR_MGM, BIT_H_RESET);
    /*---- set AUX slave address so that MPU can access device ----*/
    MLDLSetRegisterMPU(MPUREG_AUX_SLV_ADDR, auxSlaveAddr);

    /*---- reset AUX interface to make slave address take effect ----*/
    MLDLSetRegisterMPU(
        MPUREG_USER_CTRL,
        (MPUGetRegisterShadow(MPUREG_USER_CTRL) | BIT_AUX_IF_RST));

    /*---- disable pass through ----*/
    if (auxSlaveAddr != INVALID_SLAVE_ADDR) {
        MLDLSetRegisterMPU(
            MPUREG_USER_CTRL,
            MPUGetRegisterShadow(MPUREG_USER_CTRL) | BIT_AUX_IF_EN);
    }

    return ML_SUCCESS;
}

/**
 *  @internal
 * @brief   Query the MPU slave address.
 * @return  The 7-bit mpu slave address.
 **/

unsigned char MLDLGetMPUSlaveAddr() {
    INVENSENSE_FUNC_START
    return mldlData.mpuSlaveAddr;
}

/**
 *  @internal
 * @brief   query the current status of an interrupt source.
 * @param   index zero-based index of the interrupt source.
 * @return  1 if the interrupt has been triggered.
 **/
unsigned char MLDLGetIntTrigger(unsigned char index) {
    INVENSENSE_FUNC_START
    // assert(index < NUM_OF_INTSOURCES);

    return mldlData.intTrigger[index];
}

/**
 *  @internal
 * @brief clear the 'triggered' status for an interrupt source.
 * @param index Zero-based index of the interrupt source.
 **/
void MLDLClearIntTrigger(unsigned char index) {
    INVENSENSE_FUNC_START
    // assert(index < NUM_OF_INTSOURCES);
    mldlData.intTrigger[index] = 0;
}

/**
 *  @internal
 * @brief Starts the DMP running
 *
 * @return ML_SUCCESS or non-zero error code
 */
tMLError MLDLDmpStart(void) {
    INVENSENSE_FUNC_START
    unsigned short startAddress;
    MLDLResetDmp();
    startAddress =
        ((unsigned short)MPUGetRegisterShadow(MPUREG_DMP_CFG_1) << 8) |
        MPUGetRegisterShadow(MPUREG_DMP_CFG_2);

    return MLDLCtrlDmp(DMP_RUN, (mldlData.dataSource == DATASRC_FIFO),
                       startAddress);
}

/**
 *  @internal
 * @brief   MLDLCfgDMP configures the Digital Motion Processor internal to
 *          the MPU. The DMP can be enabled or disabled and the start address
 *          can be set.
 *
 * @param   enableRun   Enables the DMP processing if set to TRUE.
 * @param   enableFIFO  Enables DMP output to the FIFO if set to TRUE.
 *
 * @return  Zero if the command is successful, an error code otherwise.
 **/
tMLError MLDLCtrlDmp(unsigned char enableRun, unsigned char enableFIFO,
                     unsigned short startAddress) {
    INVENSENSE_FUNC_START
    unsigned char b;

    mldlData.runDMP = enableRun;  // save DMP run status

    /*==== FIRST LOAD DMP CFG 1 and DMP CFG 2 ====*/
    MLDLSetRegisterMPU(MPUREG_DMP_CFG_1, startAddress >> 8);
    MLDLSetRegisterMPU(MPUREG_DMP_CFG_2, startAddress & 0xff);

    /*==== NOW ENABLE/DISABLE DMP ====*/

    /*---- set 'DMP_EN' based on enable/disable ----*/
    b = MPUGetRegisterShadow(MPUREG_USER_CTRL);
    if (enableRun == DMP_RUN) {
        b |= BIT_DMP_EN;
    } else {
        b &= ~BIT_DMP_EN;
    }
    b |= BIT_DMP_RST;  // always reset DMP

    if (enableFIFO) {
        b |= BIT_FIFO_EN;
    }

    MLDLSetRegisterMPU(MPUREG_USER_CTRL, b);

    return ML_SUCCESS;
}

/**
 *  @internal
 * @brief   MLDLIntHandler function should be called when an interrupt is
 *          received.  The source parameter identifies which interrupt source
 *          caused the interrupt. Note that this routine should not be called
 *          directly from the interrupt service routine.
 *
 * @param   intSource   MPU, AUX1, AUX2, or timer. Can be one of: INTSRC_MPU,
 * INTSRC_AUX1,
 *                      INTSRC_AUX2, or INT_SRC_TIMER.
 *
 * @return  Zero if the command is successful; an error code otherwise.
 */
tMLError MLDLIntHandler(INT_SOURCE intSource) {
    INVENSENSE_FUNC_START
    /*---- range check ----*/
    if (intSource >= NUM_OF_INTSOURCES) {
        return MLDL_ERROR;
    }

    /*---- save source of interrupt ----*/
    mldlData.intTrigger[intSource] = INT_TRIGGERED;

    /*---- if 'auto mode' enabled, then get and process data immediately ----*/
    if (mldlData.autoProcess) {
        // MLProcessInts();
    }

    return ML_SUCCESS;
}

/**
 *  @internal
 * @brief   MLDLCfgSamplingMPU configures the sampling method on the MPU.
 *          Three parameters control the sampling:
 *          1) Low pass filter bandwidth,
 *          2) full scale reading from gyro, and
 *          3) output sampling divider.
 *
 *          The output sampling rate is determined by the divider and the low
 *          pass filter setting. If the low pass filter is set to
 *          'MPUFILTER_256HZ_NOLPF2', then the sample rate going into the
 *          divider is 8kHz; for all other settings it is 1kHz.
 *          The 8-bit divider will divide this frequency to get the resulting
 *          sample frequency.
 *          For example, if the filter setting is not 256Hz and the divider is
 *          set to 7, then the sample rate is as follows:
 *          sample rate = internal sample rate / div = 1kHz / 8 = 125Hz (or
 * 8ms).
 *
 * @param   lpf         low pass filter, 0 to 7.
 * @param   fullScale   Output full scale, 0 to 3.
 * @param   divider     Output sampling divder, 0 to 255.
 *
 * @return  Zero if the command is successful; an error code otherwise.
 */
tMLError MLDLCfgSamplingMPU(unsigned char lpf, unsigned char fullScale,
                            unsigned char divider) {
    unsigned char b;

    /*---- do range checking ----*/
    if (lpf > 7 || fullScale > 3) {
        return MLDL_ERROR;
    }

    /*---- set sample rate clock divider ----*/
    MLDLSetRegisterMPU(MPUREG_SMPLRT_DIV, divider);

    /*---- set low pass filter and full scale in 'DLPF_FS_SYNC' register ----*/
    b = MPUGetRegisterShadow(MPUREG_DLPF_FS_SYNC) &
        ~(BITS_FS_SEL | BITS_DLPF_CFG);
    b |= ((fullScale << 3) + lpf);
    MLDLSetRegisterMPU(MPUREG_DLPF_FS_SYNC, b);

    return MLDL_SUCCESS;
}

/**
 *  @internal
 * @brief   This function controls the power management on the MPU device.
 *          The entire chip can be put to low power sleep mode, or individual
 *          gyros can be turned on/off.
 *
 * @param   sleep       Non-zero to put device into full sleep.
 * @param   disable_gx  Disable gyro X.
 * @param   disable_gy  Disable gyro Y.
 * @param   disable_gz  Disable gyro Z.
 *
 * @return  Zero if the command is successful, an error code otherwise.
 */
tMLError MLDLPowerMgmtMPU(unsigned char sleep, unsigned char disable_gx,
                          unsigned char disable_gy, unsigned char disable_gz) {
    unsigned char b;

    static unsigned char i2cPTStatus = 0;

    unsigned char sleepChange = 0;
    unsigned char standByChanges = 0;
    unsigned char standByXChanges = 0, standByYChanges = 0, standByZChanges = 0;

    b = MPUGetRegisterShadow(MPUREG_PWR_MGM);

    /*
     *  This specific transition between states needs to be reinterpreted:
     *      (1,1,1,1) -> (0,1,1,1) has to become (1,1,1,1) -> (1,0,0,0) ->
     * (0,1,1,1)
     *  where
     *      (1,1,1,1) stands for
     * (sleep=1,disable_gx=1,disable_gy=1,disable_gz=1)
     */
    if ((b & (BIT_SLEEP | BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)) ==
            (BIT_SLEEP | BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)   // (1,1,1,1)
        && ((!sleep) && disable_gx && disable_gy && disable_gz)) {  // (0,1,1,1)

        MLDLPowerMgmtMPU(1, 0, 0, 0);
        b |= BIT_SLEEP;
        b &= ~(BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG);
    }
    /* register the changes to be made */
    if ((b & BIT_SLEEP) != (sleep * BIT_SLEEP)) {
        sleepChange = 1;
        b = b ^ BIT_SLEEP;
    }
    if ((b & BIT_STBY_XG) != (disable_gx * BIT_STBY_XG)) {
        standByXChanges = 1;
        standByChanges++;
        // b= b ^ BIT_STBY_XG;   // don't record changes for stand-by
    }
    if ((b & BIT_STBY_YG) != (disable_gy * BIT_STBY_YG)) {
        standByYChanges = 1;
        standByChanges++;
        // b= b ^ BIT_STBY_YG;
    }
    if ((b & BIT_STBY_ZG) != (disable_gz * BIT_STBY_ZG)) {
        standByZChanges = 1;
        standByChanges++;
        // b= b ^ BIT_STBY_ZG;
    }

    if (sleep && sleepChange) {  // going into sleep from awake
        i2cPTStatus = mldlData.i2cPassThrough;
        MLDLSetI2CPassThrough(1);
        MLDLSetRegisterMPU(MPUREG_PWR_MGM, b);
        MLOSSleep(1);
    }
    if (!sleep && sleepChange) {  // waking up from sleep
        MLDLSetRegisterMPU(MPUREG_PWR_MGM, b);
        MLOSSleep(5);
        if (i2cPTStatus) {  // was in master mode (that is, pass-through was
                            // disabled)
            MLDLSetI2CPassThrough(0);
            MLOSSleep(45);
            i2cPTStatus = 0;
        }
    }
    if (standByChanges > 0) {
        if (standByXChanges) {
            b ^= BIT_STBY_XG;
            MLDLSetRegisterMPU(MPUREG_PWR_MGM, b);
        }
        if (standByYChanges) {
            b ^= BIT_STBY_YG;
            MLDLSetRegisterMPU(MPUREG_PWR_MGM, b);
        }
        if (standByZChanges) {
            b ^= BIT_STBY_ZG;
            MLDLSetRegisterMPU(MPUREG_PWR_MGM, b);
        }
    }

    return ML_SUCCESS;
}

/**
 *  @internal
 * @brief   MLDLClockSource function sets the clock source for the MPU gyro
 *          processing.
 *          The source can be any of the following:
 *          - Internal 8MHz oscillator,
 *          - PLL with X gyro as reference,
 *          - PLL with Y gyro as reference,
 *          - PLL with Z gyro as reference,
 *          - PLL with external 32.768Mhz reference, or
 *          - PLL with external 19.2MHz reference
 *
 *          For best accuracy and timing, it is highly recommended to use one
 *          of the gyros as the clock source; however this gyro must be
 *          enabled to use its clock (see 'MLDLPowerMgmtMPU()').
 *
 * @param   clkSource   Clock source selection.
 *                      Can be one of:
 *                      - CLK_INTERNAL,
 *                      - CLK_PLLGYROX,
 *                      - CLK_PLLGYROY,
 *                      - CLK_PLLGYROZ,
 *                      - CLK_PLLEXT32K, or
 *                      - CLK_PLLEXT19M.
 *
 * @return  Zero if the command is successful; an error code otherwise.
 */
tMLError MLDLClockSource(unsigned char clkSource) {
    INVENSENSE_FUNC_START
    unsigned char b;

    /*---- do range checking ----*/
    if (clkSource > 7) {
        return MLDL_ERROR;
    }

    /*---- get current power management register and clear clock bits ----*/
    b = MPUGetRegisterShadow(MPUREG_PWR_MGM);
    b &= ~BITS_CLKSEL;

    /*---- set new clock select in power management register ----*/
    b |= clkSource;
    MLDLSetRegisterMPU(MPUREG_PWR_MGM, b);

    mldlData.clkSource = clkSource;  // save new setting

    return MLDL_SUCCESS;
}
/**
 *  @internal
 * @brief   This function is analagous to MLDLGetData, but is for use when
 *          the DMP is running.
 *          It provides an API for the ML to use such that access to the DMP
 *          data is abstracted.
 *          Based on the src parameter, data is read either from the FIFO or
 *          from the RAM area. The read data is placed in the buffer pointed
 *          to by the buffer parameter.
 *
 * @param          src     One of DATASRC_FIFO or DATASRC_IMMEDIATE.
 * @param[in,out]  buffer  Pointer to storage for the data.
 * @param          len     Length of the data buffer.
 *
 * @return  The amount of data read.
 */
unsigned short MLDLGetDMPData(unsigned char src, unsigned char* buffer,
                              unsigned char len) {
    unsigned short length;
    unsigned char regs[1] = {1};
    length = MPUGetFIFO(len, buffer);
    if (src == DATASRC_FIFO) {
    } else {
        if (MLDLSetMemoryMPU(KEY_D_1_179, 1, regs) != ML_SUCCESS)
            return ML_ERROR;
    }
    return length;
}

/**
 *  @internal
 *  @brief  enables/disables the I2C pass through to the accelerometer device.
 *  @param  enable Non-zero to enable pass through.
 *  @return Zero if the command is successful, an error code otherwise.
 */
tMLError MLDLSetI2CPassThrough(unsigned char enable) {
    INVENSENSE_FUNC_START
    unsigned char b;

    enable = !enable;

    if (INVALID_SLAVE_ADDR == mldlData.auxSlaveAddr) {
        enable = FALSE;
    }

    /*---- get current 'USER_CTRL' into b ----*/
    b = MPUGetRegisterShadow(MPUREG_USER_CTRL);
    /*---- set 'AUX_IF_EN' based on enable/disable ----*/
    if (enable) {
        b |= BIT_AUX_IF_EN;
    } else {
        b &= ~BIT_AUX_IF_EN;
    }

    b |= BIT_AUX_IF_RST;  // always reset AUX IF

    mldlData.i2cPassThrough = b;  // save new setting

    MLDLSetRegisterMPU(MPUREG_USER_CTRL, b);
    return ML_SUCCESS;
}

/**
 * @brief Used to query the status of the FIFO.
 * @return ML_SUCCESS if the fifo is OK.  ML_ERROR otherwise.
 */
short MLDLGetFifoStatus(void) { return mldlData.fifoError; }

/**
 *  @internal
 *  @brief used to set the appropriate register in the MPU.
 *
 *  @param reg      MPU register to write
 *  @param value    Value to write
 *
 *  @return  Zero if the command is successful, an error code otherwise.
 */
tMLError MLDLSetRegisterMPU(unsigned char reg, unsigned char value) {
    INVENSENSE_FUNC_START
    tMLError retCode;

    /*---- write register to I2C port ----*/
    retCode = MLSLSerialWriteSingle(mldlData.mpuSlaveAddr, reg, value);

    /*---- Clean up any left over buffering ---- */
    if (reg == MPUREG_USER_CTRL && (value & BIT_FIFO_RST)) {
        MLSLSerialReset();
    }
    /*---- save copy in shadow registers (first clear any reset bits) ----*/
    if (reg == MPUREG_USER_CTRL) {
        value &= ~(BIT_AUX_IF_RST | BIT_DMP_RST | BIT_FIFO_RST | BIT_GYRO_RST);
    } else if (reg == MPUREG_PWR_MGM) {
        value &= ~BIT_H_RESET;
    }
    mpuRegister[reg] = value;   // shadow register
    mpuRegisterValid[reg] = 1;  // indicate shadow register is valid

    return ML_SUCCESS;
}

/**
 *  @internal
 *  @brief  used to get the specified registers in the MPU.
 *          Reads the specified registers from the MPU and stores their value
 *          into the 'mpuRegister[]' variable.
 *
 *  @param  reg         Starting MPU register to read.
 *  @param  length      Number of bytes to read.
 *
 *  @return value of first register
 */
unsigned char MLDLGetRegistersMPU(unsigned char reg, unsigned char length) {
    INVENSENSE_FUNC_START
    uint_fast8_t i;

    /*---- use single or burst depending on length ----*/
    if (length == 1) {
        /*---- get single register from MPU ----*/
        MLSLSerialReadSingle(mldlData.mpuSlaveAddr, reg, &mpuRegister[reg]);
    } else {
        /*---- get sequence of registers from MPU ----*/
        extern tReadBurst ReadBurst;
        ReadBurst(mldlData.mpuSlaveAddr, reg, length, &mpuRegister[reg]);
    }

    /*---- update valid flags ----*/
    for (i = 0; i < length; i++) {
        mpuRegisterValid[reg + i] = 1;
    }

    /*---- return value of first register read -----*/
    return (mpuRegister[reg]);
}

/**
 *  @internal
 *  @brief  used to get the specified number of bytes from the MPU location
 *          specified by the key.
 *          Reads the specified number of bytes from the MPU location
 *          specified by the key. Each set of code specifies a function
 *          that changes keys into addresses. This function is set with
 * setGetAddress().
 *
 *  @param  key     The key to use when looking up the address.
 *  @param  length  Number of bytes to read.
 *  @param  buffer  Result for data.
 *
 *  @return ML_SUCCESS if the command is successful, ML_ERROR otherwise. The key
 *          not corresponding to a memory address will result in ML_ERROR.
 */
tMLError MLDLGetMemoryMPU(unsigned short key, unsigned short length,
                          unsigned char* buffer) {
    unsigned char bank;
    tMLError ec;
    extern tReadBurst ReadBurst;
    unsigned short memAddr;

    if (sGetAddress == NULL) return ML_ERROR;

    memAddr = sGetAddress(key);
    if (memAddr >= 0xffff) return ML_ERROR;
    bank = memAddr >> 8;  // Get Bank
    memAddr &= 0xff;

    /*---- set appropriate memory bank ----*/
    if (bank !=
        MPUGetRegisterShadow(
            MPUREG_BANK_SEL)) {  // only write bank register if it's changed
        MLDLSetRegisterMPU(MPUREG_BANK_SEL, bank);
    }

    /*---- set the memory address to the start address ----*/
    MLDLSetRegisterMPU(MPUREG_MEM_START_ADDR, (unsigned char)memAddr);

    /*---- read memory from bank with a burst read ----*/
    ec = ReadBurst(mldlData.mpuSlaveAddr, MPUREG_MEM_R_W, length, buffer);

    return ec;
}

/**
 *  @internal
 *  @brief  used to set the specified number of bytes in the specified MPU
 *          memory bank.
 *          The memory bank is one of the following:
 *          - MPUMEM_RAM_BANK_0,
 *          - MPUMEM_RAM_BANK_1,
 *          - MPUMEM_RAM_BANK_2, or
 *          - MPUMEM_RAM_BANK_3.
 *
 *  @param  bank    Memory bank to write.
 *  @param  memAddr Starting address for write.
 *  @param  length  Number of bytes to write.
 *  @param  buffer  Result for data.
 *
 *  @return zero if the command is successful, an error code otherwise.
 */
static tMLError MLDLSetMemoryMPU_one_bank(unsigned char bank,
                                          unsigned short memAddr,
                                          unsigned short length,
                                          const unsigned char* buffer) {
    unsigned char b;
    tMLError ec;
    extern tWriteBurst WriteBurst;

    /*---- set appropriate memory bank ----*/
    b = MPUGetRegisterShadow(MPUREG_BANK_SEL) &
        ~BITS_MEM_SEL;  // b = masked memory bank register
    MLDLSetRegisterMPU(MPUREG_BANK_SEL, b | (bank & 0x0f));

    /*---- set the memory address to the start address ----*/
    MLDLSetRegisterMPU(MPUREG_MEM_START_ADDR, (unsigned char)memAddr);

    /*---- write memory from bank with a burst write ----*/
    ec = WriteBurst(mldlData.mpuSlaveAddr, MPUREG_MEM_R_W, length, buffer);

    return ec;
}

/**
 *  @internal
 *  @brief  used to set the specified number of bytes from the MPU location
 *          specified by the key.
 *          Set the specified number of bytes from the MPU location
 *          specified by the key. Each set of DMP code specifies a function
 *          that changes keys into addresses. This function is set with
 * setGetAddress().
 *
 *  @param  key     The key to use when looking up the address.
 *  @param  length  Number of bytes to write.
 *  @param  buffer  Result for data.
 *
 *  @return ML_SUCCESS if the command is successful, ML_ERROR otherwise. The key
 *          not corresponding to a memory address will result in ML_ERROR.
 */
tMLError MLDLSetMemoryMPU(unsigned short key, unsigned short length,
                          const unsigned char* buffer) {
    tMLError ec = ML_SUCCESS;
    unsigned short memAddr;
    unsigned char bank;

    if (sGetAddress == NULL) return ML_ERROR;

    memAddr = sGetAddress(key);

    if (memAddr >= 0xffff) return ML_ERROR;  // This key not supported

    bank = (unsigned char)(memAddr >> 8);
    memAddr &= 0xff;

    if (memAddr + length > 256) {
        // We cross a bank in the middle
        ec = MLDLSetMemoryMPU_one_bank(bank, memAddr, 256 - memAddr, buffer);
        bank++;
        length -= 256 - memAddr;
        buffer += 256 - memAddr;
        memAddr += 256 - memAddr;
    }
    ec |= MLDLSetMemoryMPU_one_bank(bank, memAddr, length, buffer);
    return ec;
}

/**
  * @internal
  * Resets the DMP by stoping the DMP, clearing the fifo and then restarting it
 */
static tMLError MLDLResetDmp(void) {
    INVENSENSE_FUNC_START
    int len = FIFO_HW_SIZE;
    unsigned char fifoBuf[2];
    unsigned char tries = 0;
    unsigned char userCtrlReg = MPUGetRegisterShadow(MPUREG_USER_CTRL);
    unsigned short prevDataMode = mlxData.mlDataMode;
    extern tReadBurst ReadBurst;

    MLSetDataMode(0);

    while (len != 0 && tries < 6) {
        MLDLSetRegisterMPU(MPUREG_USER_CTRL,
                           ((userCtrlReg & (~BIT_FIFO_EN)) | BIT_FIFO_RST));
        ReadBurst(mldlData.mpuSlaveAddr, MPUREG_FIFO_COUNTH, 2, fifoBuf);
        len = (unsigned short)fifoBuf[0] * 256 + (unsigned short)fifoBuf[1];
        tries++;
    }
    mldlData.fifoCount = 0;
    MLDLSetRegisterMPU(MPUREG_USER_CTRL, userCtrlReg);

    MLSetDataMode(prevDataMode);

    return ML_SUCCESS;
}

/**
 * @internal
 * Get the length from the fifo
 *
 * @param[out] len amount of data currently stored in the fifo.
 *
 * @return ML_SUCCESS or non-zero error code.
 */
tMLError MLDLGetFifoLength(unsigned short* len) {
    INVENSENSE_FUNC_START
    unsigned char fifoBuf[2];
    int result;
    extern tReadBurst ReadBurst;

    /*---- read the 2 'count' registers and
      burst read the data from the FIFO ----*/
    result = ReadBurst(mldlData.mpuSlaveAddr, MPUREG_FIFO_COUNTH, 2, fifoBuf);
    if (ML_SUCCESS != result) {
        MLDLResetDmp();
        mldlData.fifoError = 1;
        *len = 0;
        return result;
    }

    *len = (unsigned short)(fifoBuf[0] << 8);
    *len += (unsigned short)(fifoBuf[1]);
    return result;
}

/**
 * @internal
 * Read data from the fifo
 *
 * @param[out] data Location to store the date read from the fifo
 * @param[in] len   Amount of data to read out of the fifo
 *
 * @return ML_SUCCESS or non-zero error code
 */
tMLError MLDLReadFifo(unsigned char* data, unsigned short len) {
    INVENSENSE_FUNC_START
    tMLError result;
    result = ReadBurst(mldlData.mpuSlaveAddr, MPUREG_FIFO_R_W, len, data);
    if (ML_SUCCESS != result) {
        MLDLResetDmp();
        mldlData.fifoError = 1;
        return result;
    }
    return result;
}

/**
 *  @internal
 *  @brief  used to get the FIFO data.
 *  @param  length  Number of bytes to read from the FIFO.
 *  @param  buffer  Result for FIFO data
 *  @return number of valid bytes of data.
 */
static unsigned short MPUGetFIFO(unsigned char length, unsigned char* buffer) {
    INVENSENSE_FUNC_START
    unsigned short inFifo, toRead;
    unsigned char fifoBuf[MAX_FIFO_LENGTH + 2];  // allocate buffer to read FIFO
    tMLError result;
    unsigned short temp;
    int_fast8_t kk;

    /*---- make sure length is correct ----*/
    if (length > MAX_FIFO_LENGTH) {
        return 0;
    }

    result = MLDLGetFifoLength(&inFifo);
    if (ML_SUCCESS != result) {
        return 0;
    }

    // mldlData.fifoCount is the footer size left in the buffer.
    if (inFifo < length + mldlData.fifoCount) return 0;

    toRead = length - 2 + mldlData.fifoCount;
    result = MLDLReadFifo(fifoBuf, toRead);
    if (ML_SUCCESS != result) {
        return 0;
    }
    result = MLDLGetFifoLength(&temp);

    /* Check the Footer value to give us a chance at making sure data
     * didn't get corrupted */
    for (kk = 0; kk < mldlData.fifoCount; ++kk) {
        if (fifoBuf[kk] != gFifoFooter[kk - 2]) {
            MLDLResetDmp();
            mldlData.fifoError = 1;
            return 0;
        }
    }

    /*---- copy data into result buffer ----*/
    memcpy(buffer, &fifoBuf[mldlData.fifoCount], length - 2);

    if (mldlData.fifoCount == 0) mldlData.fifoCount = 2;

    return length - 2;
}

/** @internal
 *  @brief  used to get the value of the specified register using the MPU
 *          shadow registers.
 *          Returns the value of the specified register using the MPU shadow
 *          registers. This avoids having to read the register from the actual
 *          register map.
 *          Determines if the shadow value is available, returns zero if not.
 *
 *  @param  reg     Starting MPU register to read.
 *
 *  @return value of register, or zero if not available.
 */
unsigned char MPUGetRegisterShadow(unsigned char reg) {
    INVENSENSE_FUNC_START
    /*---- if valid, then return value ----*/
    if (mpuRegisterValid[reg]) {
        return (mpuRegister[reg]);
    } else {
        return (0);  // return 0 if invalid
    }
}

/**
 * @internal
 * Loads DMP configuration
 * @param[in] buffer Buffer to Load
 * @param[in] startAddress location to start loading
 */
tMLError loadDMP(const MLU8* buffer, unsigned short length,
                 unsigned short startAddress) {
    INVENSENSE_FUNC_START
    tMLError result;
    unsigned short toWrite;
    unsigned short memAddr = 0;
#define MAX_LOAD_WRITE_SIZE 128

    result = MLDLCtrlDmp(DMP_DONTRUN, FALSE, startAddress);
    if (result != ML_SUCCESS) return result;

    while (length > 0) {
        toWrite = length;
        if (toWrite > MAX_LOAD_WRITE_SIZE) toWrite = MAX_LOAD_WRITE_SIZE;

        result = MLDLSetMemoryMPU_one_bank(memAddr >> 8, memAddr & 0xff,
                                           toWrite, buffer);
        if (result != ML_SUCCESS) return result;

        buffer += toWrite;
        memAddr += toWrite;
        length -= toWrite;
    }
    return result;
}

/*
 * @internal
 * @brief   DMP Init Function.
 * @return  Zero if successful, an error code otherwise.
 */
tMLError MLDLDMPInit(void) {
    INVENSENSE_FUNC_START
    tMLError result;

    result = MLDLDmpAccelInit();

    /*---- set length of AUX burst read to cover status registers ----*/
    /*    result += MLDLSetRegisterMPU( MPUREG_USER_CTRL,
                                      MPUGetRegisterShadow(MPUREG_USER_CTRL) |
       BIT_AUX_RD_LENG );*/
    return result;
}

/*
 * @internal
 * @brief   Initialize Accelerometers.
 * @return  Zero if successful, an error code otherwise.
 */
tMLError MLDLDmpAccelInit() {
    INVENSENSE_FUNC_START
    tMLError result = ML_SUCCESS;
    unsigned char regs[4] = {0};

    if (mldlData.auxSlaveAddr == KIONIX_AUX_SLAVEADDR) {
        // must set MPUREG_ACCEL_BURST_ADDR high bit before accessing the accel
        // chip

        result += MLSLSerialWriteSingle(mldlData.mpuSlaveAddr,
                                        MPUREG_ACCEL_BURST_ADDR, 0x86);

        // Kionix Accel
        regs[0] = 0;
        regs[1] = 64;
        regs[2] = 0;
        regs[3] = 0;
        result += MLDLSetMemoryMPU(KEY_D_1_236, 4, regs);

        result += MLDLSetI2CPassThrough(1);  // By-pass on
        result += MLSLSerialWriteSingle(mldlData.auxSlaveAddr, 0x1d,
                                        0xcd);  // RAM reset
        MLOSSleep(10);
        result += MLSLSerialWriteSingle(mldlData.auxSlaveAddr, 0x1b,
                                        0x42);  // Wake up
        result += MLSLSerialWriteSingle(mldlData.auxSlaveAddr, 0x1b,
                                        0xc2);  // Normal operation
        MLOSSleep(50);
        result += MLSLSerialWriteSingle(
            mldlData.auxSlaveAddr, 0x1e,
            0x14);  // INT_CTRL_REG1: Configure non-latching wake-up
        result += MLSLSerialWriteSingle(
            mldlData.auxSlaveAddr, 0x5a,
            0x00);  // WUF_THRESH:    wake-up threshold (on motion)
        result +=
            MLSLSerialWriteSingle(mldlData.auxSlaveAddr, 0x21,
                                  0x06);  // DATA_CTRL_REG: output data rate
        result += MLSLSerialWriteSingle(mldlData.auxSlaveAddr, 0x29,
                                        0x02);  // WUF_TIMER:     wake-up timer
        result += MLDLSetI2CPassThrough(0);     // By-pass off
    } else {
        result = ML_ERROR;
    }
    return result;
}

/*********************/
/** \}*/ /* defgroup */
/*********************/
