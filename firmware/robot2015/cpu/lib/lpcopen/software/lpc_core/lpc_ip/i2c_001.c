/*
 * @brief	I2C driver functions
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "i2c_001.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Control flags */
#define I2C_CON_FLAGS (I2C_CON_AA | I2C_CON_SI | I2C_CON_STO | I2C_CON_STA)

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/
/* Match the slave address */
static int IP_I2C_SlaveMatchAddr(uint8_t addr1, uint8_t addr2, uint8_t mask)
{
	mask |= 1;
	return (addr1 & ~mask) == (addr2 & ~mask);
}

/* Get the index of the active slave */
static I2C_SLAVE_ID IP_I2C_SlaveIndexLookup(IP_I2C_001_T *pI2C, uint8_t slaveAddr)
{
	if (!(slaveAddr >> 1)) {
		return I2C_SLAVE_GENERAL;					/* General call address */
	}
	if (IP_I2C_SlaveMatchAddr(pI2C->ADR0, slaveAddr, pI2C->MASK[0])) {
		return I2C_SLAVE_0;
	}
	if (IP_I2C_SlaveMatchAddr(pI2C->ADR1, slaveAddr, pI2C->MASK[1])) {
		return I2C_SLAVE_1;
	}
	if (IP_I2C_SlaveMatchAddr(pI2C->ADR2, slaveAddr, pI2C->MASK[2])) {
		return I2C_SLAVE_2;
	}
	if (IP_I2C_SlaveMatchAddr(pI2C->ADR3, slaveAddr, pI2C->MASK[3])) {
		return I2C_SLAVE_3;
	}

	/* If everything is fine the code should never come here */
	return I2C_SLAVE_GENERAL;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/
/* Master transfer state change handler handler */
int IP_I2C_MasterXfer_StateHandler(IP_I2C_001_T *pI2C, I2C_XFER_T  *xfer)
{
	uint32_t cclr = I2C_CON_FLAGS;

	switch (IP_I2C_GetCurrentState(pI2C)) {
	case 0x08:		/* Start condition on bus */
	case 0x10:		/* Repeated start condition */
		pI2C->DAT = (xfer->slaveAddr << 1) | (xfer->txSz == 0);
		break;

	/* Tx handling */
	case 0x18:		/* SLA+W sent and ACK received */
	case 0x28:		/* DATA sent and ACK received */
		if (!xfer->txSz) {
			cclr &= ~(xfer->rxSz ? I2C_CON_STA : I2C_CON_STO);
		}
		else {
			pI2C->DAT = *xfer->txBuff++;
			xfer->txSz--;
		}
		break;

	/* Rx handling */
	case 0x58:		/* Data Received and NACK sent */
		cclr &= ~I2C_CON_STO;

	case 0x50:		/* Data Received and ACK sent */
		*xfer->rxBuff++ = pI2C->DAT;
		xfer->rxSz--;

	case 0x40:		/* SLA+R sent and ACK received */
		if (xfer->rxSz > 1) {
			cclr &= ~I2C_CON_AA;
		}
		break;

	/* NAK Handling */
	case 0x20:		/* SLA+W sent NAK received */
	case 0x30:		/* DATA sent NAK received */
	case 0x48:		/* SLA+R sent NAK received */
		xfer->status = I2C_STATUS_NAK;
		cclr &= ~I2C_CON_STO;
		break;

	case 0x38:		/* Arbitration lost */
		xfer->status = I2C_STATUS_ARBLOST;
		break;

	/* Bus Error */
	case 0x00:
		xfer->status = I2C_STATUS_BUSERR;
		cclr &= ~I2C_CON_STO;
	}

	/* Set clear control flags */
	pI2C->CONSET = cclr ^ I2C_CON_FLAGS;
	pI2C->CONCLR = cclr;

	/* If stopped return 0 */
	if (!(cclr & I2C_CON_STO) || (xfer->status == I2C_STATUS_ARBLOST)) {
		if (xfer->status == I2C_STATUS_BUSY) {
			xfer->status = I2C_STATUS_DONE;
		}
		return 0;
	}
	return 1;
}

/* Find the slave address of SLA+W or SLA+R */
I2C_SLAVE_ID IP_I2C_GetSlaveIndex(IP_I2C_001_T *pI2C)
{
	switch (IP_I2C_GetCurrentState(pI2C)) {
	case 0x60:
	case 0x68:
	case 0x70:
	case 0x78:
	case 0xA8:
	case 0xB0:
		return IP_I2C_SlaveIndexLookup(pI2C, pI2C->DAT);
	}

	/* If everything is fine code should never come here */
	return I2C_SLAVE_GENERAL;
}

/* Slave state machine handler */
int IP_I2C_SlaveXfer_StateHandler(IP_I2C_001_T *pI2C, I2C_XFER_T *xfer)
{
	uint32_t cclr = I2C_CON_FLAGS;
	int ret = RET_SLAVE_BUSY;

	xfer->status = I2C_STATUS_BUSY;
	switch (IP_I2C_GetCurrentState(pI2C)) {
	case 0x80:		/* SLA: Data received + ACK sent */
	case 0x90:		/* GC: Data received + ACK sent */
		*xfer->rxBuff++ = pI2C->DAT;
		xfer->rxSz--;
		ret = RET_SLAVE_RX;
		if (xfer->rxSz > 1) {
			cclr &= ~I2C_CON_AA;
		}
		break;

	case 0x60:		/* Own SLA+W received */
	case 0x68:		/* Own SLA+W received after losing arbitration */
	case 0x70:		/* GC+W received */
	case 0x78:		/* GC+W received after losing arbitration */
		xfer->slaveAddr = pI2C->DAT & ~1;
		if (xfer->rxSz > 1) {
			cclr &= ~I2C_CON_AA;
		}
		break;

	case 0xA8:		/* SLA+R received */
	case 0xB0:		/* SLA+R received after losing arbitration */
		xfer->slaveAddr = pI2C->DAT & ~1;

	case 0xB8:		/* DATA sent and ACK received */
		pI2C->DAT = *xfer->txBuff++;
		xfer->txSz--;
		if (xfer->txSz > 0) {
			cclr &= ~I2C_CON_AA;
		}
		ret = RET_SLAVE_TX;
		break;

	case 0xC0:		/* Data transmitted and NAK received */
	case 0xC8:		/* Last data transmitted and ACK received */
	case 0x88:		/* SLA: Data received + NAK sent */
	case 0x98:		/* GC: Data received + NAK sent */
	case 0xA0:		/* STOP/Repeated START condition received */
		ret = RET_SLAVE_IDLE;
		cclr &= ~I2C_CON_AA;
		xfer->status = I2C_STATUS_DONE;
		if (xfer->slaveAddr & 1) {
			cclr &= ~I2C_CON_STA;
		}
		break;
	}

	/* Set clear control flags */
	pI2C->CONSET = cclr ^ I2C_CON_FLAGS;
	pI2C->CONCLR = cclr;

	return ret;
}
