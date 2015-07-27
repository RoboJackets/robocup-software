/*
 * @brief LPC17xx/40xx I2C driver
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

#include "chip.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define SLAVE_ACTIVE(iic) (((iic)->flags & 0xFF00) != 0)

#ifdef CHIP_LPC175X_6X
static const CHIP_SYSCTL_PCLK_T I2C_PeriphClk[I2C_NUM_INTERFACE] = {
	SYSCTL_PCLK_I2C0,
	SYSCTL_PCLK_I2C1,
	SYSCTL_PCLK_I2C2
};
#define I2C_GetClk(x) Chip_Clock_GetPeripheralClockRate(I2C_PeriphClk[x])
#else
#define I2C_GetClk(x) Chip_Clock_GetPeripheralClockRate()
#endif

/* I2C common interface structure */
struct i2c_interface {
	LPC_I2C_T *ip;		/* IP base address of the I2C device */
	CHIP_SYSCTL_CLOCK_T clk;	/* Clock used by I2C */
	I2C_EVENTHANDLER_T mEvent;	/* Current active Master event handler */
	I2C_EVENTHANDLER_T sEvent;	/* Slave transfer events */
	I2C_XFER_T *mXfer;	/* Current active xfer pointer */
	I2C_XFER_T *sXfer;	/* Pointer to store xfer when bus is busy */
	uint32_t flags;		/* Flags used by I2C master and slave */
};

/* Slave interface structure */
struct i2c_slave_interface {
	I2C_XFER_T *xfer;
	I2C_EVENTHANDLER_T event;
};

/* I2C interfaces */
static struct i2c_interface i2c[I2C_NUM_INTERFACE] = {
	{LPC_I2C0, SYSCTL_CLOCK_I2C0, Chip_I2C_EventHandler, NULL, NULL, NULL, 0},
	{LPC_I2C1, SYSCTL_CLOCK_I2C1, Chip_I2C_EventHandler, NULL, NULL, NULL, 0},
	{LPC_I2C2, SYSCTL_CLOCK_I2C2, Chip_I2C_EventHandler, NULL, NULL, NULL, 0}
};

static struct i2c_slave_interface i2c_slave[I2C_NUM_INTERFACE][I2C_SLAVE_NUM_INTERFACE];

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/
/* Chip event handler interrupt based */
void Chip_I2C_EventHandler(I2C_ID_T id, I2C_EVENT_T event)
{
	struct i2c_interface *iic = &i2c[id];
	volatile I2C_STATUS_T *stat;

	/* Only WAIT event needs to be handled */
	if (event != I2C_EVENT_WAIT) {
		return;
	}

	stat = &iic->mXfer->status;
	/* Wait for the status to change */
	while (*stat == I2C_STATUS_BUSY) {}
}

/* Chip polling event handler */
void Chip_I2C_EventHandlerPolling(I2C_ID_T id, I2C_EVENT_T event)
{
	struct i2c_interface *iic = &i2c[id];
	volatile I2C_STATUS_T *stat;

	/* Only WAIT event needs to be handled */
	if (event != I2C_EVENT_WAIT) {
		return;
	}

	stat = &iic->mXfer->status;
	/* Call the state change handler till xfer is done */
	while (*stat == I2C_STATUS_BUSY) {
		if (IP_I2C_IsStateChanged(iic->ip)) {
			Chip_I2C_MasterStateHandler(id);
		}
	}
}

/* Initializes the LPC_I2C peripheral with specified parameter */
void Chip_I2C_Init(I2C_ID_T id)
{
	/* Enable I2C Clocking */
	Chip_Clock_EnablePeriphClock(i2c[id].clk);

	IP_I2C_Init(i2c[id].ip);
}

/* De-initializes the I2C peripheral registers to their default reset values */
void Chip_I2C_DeInit(I2C_ID_T id)
{
	IP_I2C_DeInit(i2c[id].ip);

	/* Disable I2C clocking */
	Chip_Clock_DisablePeriphClock(i2c[id].clk);
}

/* Set up clock rate for LPC_I2C peripheral */
void Chip_I2C_SetClockRate(I2C_ID_T id, uint32_t clockrate)
{
	IP_I2C_SetClockRate(i2c[id].ip, I2C_GetClk(id) / clockrate);
}

/* Get current clock rate for LPC_I2C peripheral */
uint32_t Chip_I2C_GetClockRate(I2C_ID_T id)
{
	return I2C_GetClk(id) / IP_I2C_GetClockDiv(i2c[id].ip);
}

/* Set the master event handler */
int Chip_I2C_SetMasterEventHandler(I2C_ID_T id, I2C_EVENTHANDLER_T event)
{
	struct i2c_interface *iic = &i2c[id];
	if (!iic->mXfer) {
		iic->mEvent = event;
	}
	return iic->mEvent == event;
}

/* Get the master event handler */
I2C_EVENTHANDLER_T Chip_I2C_GetMasterEventHandler(I2C_ID_T id)
{
	return i2c[id].mEvent;
}

/* Transmit and Receive data in master mode */
int Chip_I2C_MasterTransfer(I2C_ID_T id, I2C_XFER_T *xfer)
{
	struct i2c_interface *iic = &i2c[id];

	iic->mEvent(id, I2C_EVENT_LOCK);
	xfer->status = I2C_STATUS_BUSY;
	iic->mXfer = xfer;

	/* If slave xfer not in progress */
	if (!iic->sXfer) {
		IP_I2C_Master_StartXfer(iic->ip);
	}
	iic->mEvent(id, I2C_EVENT_WAIT);
	iic->mXfer = 0;

	/* Wait for stop condition to appear on bus */
	while (!IP_I2C_BusFree(iic->ip)) {}

	/* Start slave if one is active */
	if (SLAVE_ACTIVE(iic)) {
		IP_I2C_Slave_StartXfer(iic->ip);
	}

	iic->mEvent(id, I2C_EVENT_UNLOCK);
	return (int) xfer->status;
}

/* Master tx only */
int Chip_I2C_MasterSend(I2C_ID_T id, uint8_t slaveAddr, const uint8_t *buff, uint8_t len)
{
	I2C_XFER_T xfer = {0};
	xfer.slaveAddr = slaveAddr;
	xfer.txBuff = buff;
	xfer.txSz = len;
	while (Chip_I2C_MasterTransfer(id, &xfer) == I2C_STATUS_ARBLOST) {}
	return len - xfer.txSz;
}

/* Transmit one byte and receive an array of bytes after a repeated start condition is generated in Master mode.
 * This function is useful for communicating with the I2C slave registers
 */
int Chip_I2C_MasterCmdRead(I2C_ID_T id, uint8_t slaveAddr, uint8_t cmd, uint8_t *buff, int len)
{
	I2C_XFER_T xfer = {0};
	xfer.slaveAddr = slaveAddr;
	xfer.txBuff = &cmd;
	xfer.txSz = 1;
	xfer.rxBuff = buff;
	xfer.rxSz = len;
	while (Chip_I2C_MasterTransfer(id, &xfer) == I2C_STATUS_ARBLOST) {}
	return len - xfer.rxSz;
}

/* Sequential master read */
int Chip_I2C_MasterRead(I2C_ID_T id, uint8_t slaveAddr, uint8_t *buff, int len)
{
	I2C_XFER_T xfer = {0};
	xfer.slaveAddr = slaveAddr;
	xfer.rxBuff = buff;
	xfer.rxSz = len;
	while (Chip_I2C_MasterTransfer(id, &xfer) == I2C_STATUS_ARBLOST) {}
	return len - xfer.rxSz;
}

/* Check if master state is active */
int Chip_I2C_IsMasterActive(I2C_ID_T id)
{
	return IP_I2C_IsMasterState(i2c[id].ip);
}

/* State change handler for master transfer */
void Chip_I2C_MasterStateHandler(I2C_ID_T id)
{
	if (!IP_I2C_MasterXfer_StateHandler(i2c[id].ip, i2c[id].mXfer)) {
		i2c[id].mEvent(id, I2C_EVENT_DONE);
	}
}

/* Setup slave function */
void Chip_I2C_SlaveSetup(I2C_ID_T id,
						 I2C_SLAVE_ID sid,
						 I2C_XFER_T *xfer,
						 I2C_EVENTHANDLER_T event,
						 uint8_t addrMask)
{
	struct i2c_interface *iic = &i2c[id];
	struct i2c_slave_interface *si2c = &i2c_slave[id][sid];
	si2c->xfer = xfer;
	si2c->event = event;

	/* Set up the slave address */
	if (sid != I2C_SLAVE_GENERAL) {
		IP_I2C_SetSlaveAddress(iic->ip, sid, xfer->slaveAddr, addrMask);
	}

	if (!SLAVE_ACTIVE(iic) && !iic->mXfer) {
		IP_I2C_Slave_StartXfer(iic->ip);
	}
	iic->flags |= 1 << (sid + 8);
}

/* I2C Slave event handler */
void Chip_I2C_SlaveStateHandler(I2C_ID_T id)
{
	int ret;
	struct i2c_interface *iic = &i2c[id];

	/* Get the currently addressed slave */
	if (!iic->sXfer) {
		struct i2c_slave_interface *si2c;

		I2C_SLAVE_ID sid = IP_I2C_GetSlaveIndex(iic->ip);
		si2c = &i2c_slave[id][sid];
		iic->sXfer = si2c->xfer;
		iic->sEvent = si2c->event;
	}

	iic->sXfer->slaveAddr |= iic->mXfer != 0;
	ret = IP_I2C_SlaveXfer_StateHandler(iic->ip, iic->sXfer);
	if (ret) {
		if (iic->sXfer->status == I2C_STATUS_DONE) {
			iic->sXfer = 0;
		}
		iic->sEvent(id, (I2C_EVENT_T) ret);
	}
}

/* Disable I2C device */
void Chip_I2C_Disable(I2C_ID_T id)
{
	IP_I2C_Disable(i2c[id].ip);
}

/* State change checking */
int Chip_I2C_IsStateChanged(I2C_ID_T id)
{
	return IP_I2C_IsStateChanged(i2c[id].ip);
}
