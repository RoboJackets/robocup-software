/*******************************************************************************
 * Copyright (c) 2010 InvenSense Corporation, All Rights Reserved.
 * See license.txt for license.
 ******************************************************************************/

#include "imuMlsl.h"
#include <stdio.h>
#include "imuSetup.h"
#include "imuMldl.h"
#include "imuMlos.h"
#include "../board.h"

/**
 *   @defgroup MLSL
 *   @brief ML Serial Layer Interface.
 *
 *   @{
 *       @file imuMlsl.c
 *       @brief ML Serial Layer Interface.
**/

tWriteBurst WriteBurst = &MLSLSerialWriteBurst;
tReadBurst ReadBurst = &MLSLSerialReadBurst;

#define SERIAL_TIMEOUT 40 * 8

int _i2c;

tMLError IMUserialOpen() { return ML_SUCCESS; }

tMLError IMUserialClose(void) { return ML_SUCCESS; }

/**
 *  @brief  used to reset any buffering the driver may be doing
 *  @return Zero if the command is successful, an error code otherwise.
 */
tMLError MLSLSerialReset(void) { return ML_SUCCESS; }

/**
 *  @brief  used to write a single byte of data.
 *          It is called by the MPL to write a single byte of data to the IMU.
 *          This should be sent by I2C or SPI.
 *
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  registerAddr    Register address to write.
 *  @param  data            Single byte of data to write.
 *
 *  @return Zero if the command is successful, an error code otherwise.
 */
tMLError MLSLSerialWriteSingle(unsigned char slaveAddr,
                               unsigned char registerAddr, unsigned char data) {
    while (!(AT91C_BASE_TWI->TWI_SR & AT91C_TWI_TXCOMP))
        ;
    AT91C_BASE_TWI->TWI_MMR = (slaveAddr << 16) | AT91C_TWI_IADRSZ_1_BYTE;
    AT91C_BASE_TWI->TWI_IADR = registerAddr;
    AT91C_BASE_TWI->TWI_CR = AT91C_TWI_START | AT91C_TWI_STOP;
    AT91C_BASE_TWI->TWI_THR = data;
    uint32_t status;
    while (1) {
        status = AT91C_BASE_TWI->TWI_SR;
        if (status & (AT91C_TWI_TXRDY | AT91C_TWI_NACK)) {
            break;
        }
    }
    if (status & AT91C_TWI_NACK) {
        return ML_ERROR;
    } else {
        return ML_SUCCESS;
    }
}

/**
 *  @brief  used to write multiple bytes of data.
 *          This should be sent by I2C or SPI.
 *
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  registerAddr    Register address to write.
 *  @param  length          Length of burst data.
 *  @param  data            Pointer to block of data.
 *
 *  @return Zero if the command is successful, an error code otherwise.
 */
tMLError MLSLSerialWriteBurst(unsigned char slaveAddr,
                              unsigned char registerAddr, unsigned short length,
                              const unsigned char* data) {
    while (!(AT91C_BASE_TWI->TWI_SR & AT91C_TWI_TXCOMP))
        ;
    AT91C_BASE_TWI->TWI_MMR = (slaveAddr << 16) | AT91C_TWI_IADRSZ_1_BYTE;
    AT91C_BASE_TWI->TWI_IADR = registerAddr;
    AT91C_BASE_TWI->TWI_CR = AT91C_TWI_START | AT91C_TWI_STOP;
    for (int i = 0; i < length; ++i) {
        AT91C_BASE_TWI->TWI_THR = data[i];
        uint32_t status;
        while (1) {
            status = AT91C_BASE_TWI->TWI_SR;
            if (status & AT91C_TWI_NACK) {
                return ML_ERROR;
            }
            if (status & AT91C_TWI_TXRDY) {
                break;
            }
        }
    }
    return ML_SUCCESS;
}

/**
 *  @brief  used to read a single byte of data.
 *          This should be sent by I2C or SPI.
 *
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  registerAddr    Register address to read.
 *  @param  data            Single byte of data to read.
 *
 *  @return Zero if the command is successful, an error code otherwise.
 */
tMLError MLSLSerialReadSingle(unsigned char slaveAddr,
                              unsigned char registerAddr, unsigned char* data) {
    while (!(AT91C_BASE_TWI->TWI_SR & AT91C_TWI_TXCOMP))
        ;
    AT91C_BASE_TWI->TWI_MMR =
        (slaveAddr << 16) | AT91C_TWI_MREAD | AT91C_TWI_IADRSZ_1_BYTE;
    AT91C_BASE_TWI->TWI_IADR = registerAddr;
    AT91C_BASE_TWI->TWI_CR = AT91C_TWI_START | AT91C_TWI_STOP;
    uint32_t status;
    while (1) {
        status = AT91C_BASE_TWI->TWI_SR;
        if (status & (AT91C_TWI_RXRDY | AT91C_TWI_NACK)) {
            break;
        }
    }
    uint8_t result = AT91C_BASE_TWI->TWI_RHR;
    while (!(AT91C_BASE_TWI->TWI_SR & AT91C_TWI_TXCOMP))
        ;
    if (status & AT91C_TWI_NACK) {
        return ML_ERROR;
    } else {
        *data = result;
        return ML_SUCCESS;
    }
}

/**
 *  @brief  used to read multiple bytes of data.
 *          This should be sent by I2C or SPI.
 *
 *  @param  slaveAddr       I2C slave address of device.
 *  @param  registerAddr    Register address to read.
 *  @param  length          Length of burst data.
 *  @param  data            Pointer to block of data.
 *
 *  @return Zero if the command is successful; an error code otherwise
 */
tMLError MLSLSerialReadBurst(unsigned char slaveAddr,
                             unsigned char registerAddr, unsigned short length,
                             unsigned char* data) {
    while (!(AT91C_BASE_TWI->TWI_SR & AT91C_TWI_TXCOMP))
        ;
    AT91C_BASE_TWI->TWI_MMR =
        (slaveAddr << 16) | AT91C_TWI_MREAD | AT91C_TWI_IADRSZ_1_BYTE;
    AT91C_BASE_TWI->TWI_IADR = registerAddr;
    AT91C_BASE_TWI->TWI_CR = AT91C_TWI_START;
    uint32_t status;
    for (int i = 0; i < length; ++i) {
        if (i == (length - 1)) {
            AT91C_BASE_TWI->TWI_CR = AT91C_TWI_STOP;
        }

        while (1) {
            status = AT91C_BASE_TWI->TWI_SR;
            if (status & AT91C_TWI_NACK) {
                AT91C_BASE_TWI->TWI_RHR;
                return ML_ERROR;
            }
            if (status & AT91C_TWI_RXRDY) {
                break;
            }
        }
        data[i] = AT91C_BASE_TWI->TWI_RHR;
    }
    return ML_SUCCESS;
}

/**
 *  @brief  used to handle the interrupt.
 *          MLSLIntHandler is the Motion Library Interrupt Handler.
 *          It should be called when the system kernel/operating system has
 *          detected the motion processing interrupt from the interrupt
 *          output pin on the IMU (pin 12).
 *          This could also be from a timer interrupt used to trigger the
 *          motion library. The reason for the interrupt is passed in the
 *          'intSource' argument.
 *          The valid values for this are as follows:
 *          - INTSRC_IMU,
 *          - INTSRC_AUX1,
 *          - INTSRC_AUX2,
 *          - INTSRC_TIMER, and
 *          - INTSRC_UNKNOWN.
 *
 *          Note that this routine should not be called directly from the ISR
 *          (interrupt service routine).
 *
 *  @param  intSource  Interrupt source.
 *
 *  @return Zero if the command is successful, an error code otherwise.
 */
tMLError MLSLIntHandler(unsigned char intSource) {
    tMLError ec;

    ec = MLDLIntHandler(intSource);

    return ec;
}

/*********************/
/** \}*/ /* defgroup */
/*********************/
