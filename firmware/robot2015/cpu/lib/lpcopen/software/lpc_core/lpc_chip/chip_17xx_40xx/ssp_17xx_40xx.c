/*
 * @brief LPC17xx/40xx SSP driver
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

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Write byte(s) to FIFO buffer */
static void SSP_Write2Fifo(LPC_SSP_T *pSSP, Chip_SSP_DATA_SETUP_T *xf_setup, uint32_t num_bytes)
{
	uint16_t data2write = 0xFFFF;

	if (num_bytes == 2) {
		data2write = xf_setup->tx_data ? (*(uint16_t *) ((uint32_t) xf_setup->tx_data + xf_setup->tx_cnt)) : 0xFFFF;
	}
	else {
		data2write = xf_setup->tx_data ? (*(uint8_t *) ((uint32_t) xf_setup->tx_data + xf_setup->tx_cnt)) : 0xFF;
	}

	IP_SSP_SendFrame(pSSP, data2write);
	xf_setup->tx_cnt += num_bytes;
}

/* Read byte(s) from FIFO buffer */
static void SSP_ReadFromFifo(LPC_SSP_T *pSSP, Chip_SSP_DATA_SETUP_T *xf_setup, uint16_t rDat, uint32_t num_bytes)
{
	while (IP_SSP_GetStatus(pSSP, SSP_STAT_RNE) == SET && xf_setup->rx_cnt < xf_setup->length) {
		rDat = IP_SSP_ReceiveFrame(pSSP);
		if (xf_setup->rx_data) {
			if (num_bytes == 2) {
				*(uint16_t *) ((uint32_t) xf_setup->rx_data + xf_setup->rx_cnt) = rDat;
			}
			else {
				*(uint8_t *) ((uint32_t) xf_setup->rx_data + xf_setup->rx_cnt) = rDat;
			}
		}
		xf_setup->rx_cnt += num_bytes;
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* SSP Polling Read/Write in blocking mode */
uint32_t Chip_SSP_RWFrames_Blocking(LPC_SSP_T *pSSP, Chip_SSP_DATA_SETUP_T *xf_setup)
{
	uint16_t rDat = 0x0000;

	/* Clear all remaining frames in RX FIFO */
	while (IP_SSP_GetStatus(pSSP, SSP_STAT_RNE)) {
		IP_SSP_ReceiveFrame(pSSP);
	}

	/* Clear status */
	IP_SSP_ClearIntPending(pSSP, SSP_INT_CLEAR_BITMASK);

	if (IP_SSP_GetDataSize(pSSP) > SSP_BITS_8) {
		while (xf_setup->rx_cnt < xf_setup->length || xf_setup->tx_cnt < xf_setup->length) {
			/* write data to buffer */
			if (( IP_SSP_GetStatus(pSSP, SSP_STAT_TNF) == SET) && ( xf_setup->tx_cnt < xf_setup->length) ) {
				SSP_Write2Fifo(pSSP, xf_setup, 2);
			}

			/* Check overrun error */
			if (IP_SSP_GetRawIntStatus(pSSP, SSP_RORRIS) == SET) {
				return ERROR;
			}

			/* Check for any data available in RX FIFO */
			SSP_ReadFromFifo(pSSP, xf_setup, rDat, 2);
		}

		if (xf_setup->tx_data) {
			return xf_setup->tx_cnt;
		}
		else if (xf_setup->rx_data) {
			return xf_setup->rx_cnt;
		}
		return 0;
	}
	else {
		while (xf_setup->rx_cnt < xf_setup->length || xf_setup->tx_cnt < xf_setup->length) {
			/* write data to buffer */
			if (( IP_SSP_GetStatus(pSSP, SSP_STAT_TNF) == SET) && ( xf_setup->tx_cnt < xf_setup->length) ) {
				SSP_Write2Fifo(pSSP, xf_setup, 1);
			}

			/* Check overrun error */
			if (IP_SSP_GetRawIntStatus(pSSP, SSP_RORRIS) == SET) {
				return ERROR;
			}

			/* Check for any data available in RX FIFO */
			SSP_ReadFromFifo(pSSP, xf_setup, rDat, 1);
		}

		if (xf_setup->tx_data) {
			return xf_setup->tx_cnt;
		}
		else if (xf_setup->rx_data) {
			return xf_setup->rx_cnt;
		}
		return 0;
	}
}

/* SSP Polling Write in blocking mode */
uint32_t Chip_SSP_WriteFrames_Blocking(LPC_SSP_T *pSSP, uint8_t *buffer, uint32_t buffer_len)
{
	uint32_t tx_cnt = 0, rx_cnt = 0;

	/* Clear all remaining frames in RX FIFO */
	while (IP_SSP_GetStatus(pSSP, SSP_STAT_RNE)) {
		IP_SSP_ReceiveFrame(pSSP);
	}

	/* Clear status */
	IP_SSP_ClearIntPending(pSSP, SSP_INT_CLEAR_BITMASK);

	if (IP_SSP_GetDataSize(pSSP) > SSP_BITS_8) {
		uint16_t *wdata16;

		wdata16 = (uint16_t *) buffer;

		while (tx_cnt < buffer_len || rx_cnt < buffer_len) {
			/* write data to buffer */
			if ((IP_SSP_GetStatus(pSSP, SSP_STAT_TNF) == SET) && (tx_cnt < buffer_len)) {
				IP_SSP_SendFrame(pSSP, *wdata16);
				wdata16++;
				tx_cnt += 2;
			}

			/* Check overrun error */
			if (IP_SSP_GetRawIntStatus(pSSP, SSP_RORRIS) == SET) {
				return ERROR;
			}

			/* Check for any data available in RX FIFO */
			while (IP_SSP_GetStatus(pSSP, SSP_STAT_RNE) == SET) {
				IP_SSP_ReceiveFrame(pSSP);	/* read dummy data */
				rx_cnt += 2;
			}
		}

		return tx_cnt;
	}
	else {
		uint8_t *wdata8;

		wdata8 = buffer;

		while (tx_cnt < buffer_len || rx_cnt < buffer_len) {
			/* write data to buffer */
			if ((IP_SSP_GetStatus(pSSP, SSP_STAT_TNF) == SET) && (tx_cnt < buffer_len)) {
				IP_SSP_SendFrame(pSSP, *wdata8);
				wdata8++;
				tx_cnt++;
			}

			/* Check overrun error */
			if (IP_SSP_GetRawIntStatus(pSSP, SSP_RORRIS) == SET) {
				return ERROR;
			}

			/* Check for any data available in RX FIFO */
			while (IP_SSP_GetStatus(pSSP, SSP_STAT_RNE) == SET && rx_cnt < buffer_len) {
				IP_SSP_ReceiveFrame(pSSP);	/* read dummy data */
				rx_cnt++;
			}
		}

		return tx_cnt;
	}
}

/* SSP Polling Read in blocking mode */
uint32_t Chip_SSP_ReadFrames_Blocking(LPC_SSP_T *pSSP, uint8_t *buffer, uint32_t buffer_len)
{
	uint32_t rx_cnt = 0, tx_cnt = 0;

	/* Clear all remaining frames in RX FIFO */
	while (IP_SSP_GetStatus(pSSP, SSP_STAT_RNE)) {
		IP_SSP_ReceiveFrame(pSSP);
	}

	/* Clear status */
	IP_SSP_ClearIntPending(pSSP, SSP_INT_CLEAR_BITMASK);

	if (IP_SSP_GetDataSize(pSSP) > SSP_BITS_8) {
		uint16_t *rdata16;

		rdata16 = (uint16_t *) buffer;

		while (tx_cnt < buffer_len || rx_cnt < buffer_len) {
			/* write data to buffer */
			if ((IP_SSP_GetStatus(pSSP, SSP_STAT_TNF) == SET) && (tx_cnt < buffer_len)) {
				IP_SSP_SendFrame(pSSP, 0xFFFF);	/* just send dummy data */
				tx_cnt += 2;
			}

			/* Check overrun error */
			if (IP_SSP_GetRawIntStatus(pSSP, SSP_RORRIS) == SET) {
				return ERROR;
			}

			/* Check for any data available in RX FIFO */
			while (IP_SSP_GetStatus(pSSP, SSP_STAT_RNE) == SET && rx_cnt < buffer_len) {
				*rdata16 = IP_SSP_ReceiveFrame(pSSP);
				rdata16++;
				rx_cnt += 2;
			}
		}

		return rx_cnt;
	}
	else {
		uint8_t *rdata8;

		rdata8 = buffer;

		while (tx_cnt < buffer_len || rx_cnt < buffer_len) {
			/* write data to buffer */
			if ((IP_SSP_GetStatus(pSSP, SSP_STAT_TNF) == SET) && (tx_cnt < buffer_len)) {
				IP_SSP_SendFrame(pSSP, 0xFF);	/* just send dummy data		 */
				tx_cnt++;
			}

			/* Check overrun error */
			if (IP_SSP_GetRawIntStatus(pSSP, SSP_RORRIS) == SET) {
				return ERROR;
			}

			/* Check for any data available in RX FIFO */
			while (IP_SSP_GetStatus(pSSP, SSP_STAT_RNE) == SET && rx_cnt < buffer_len) {
				*rdata8 = IP_SSP_ReceiveFrame(pSSP);
				rdata8++;
				rx_cnt++;
			}
		}

		return rx_cnt;
	}
}

/* Clean all data in RX FIFO of SSP */
void Chip_SSP_Int_FlushData(LPC_SSP_T *pSSP)
{
	if (IP_SSP_GetStatus(pSSP, SSP_STAT_BSY)) {
		while (IP_SSP_GetStatus(pSSP, SSP_STAT_BSY)) {}
	}

	/* Clear all remaining frames in RX FIFO */
	while (IP_SSP_GetStatus(pSSP, SSP_STAT_RNE)) {
		IP_SSP_ReceiveFrame(pSSP);
	}

	/* Clear status */
	IP_SSP_ClearIntPending(pSSP, SSP_INT_CLEAR_BITMASK);
}

/* SSP Interrupt Read/Write with 8-bit frame width */
Status Chip_SSP_Int_RWFrames8Bits(LPC_SSP_T *pSSP, Chip_SSP_DATA_SETUP_T *xf_setup)
{
	uint16_t rDat = 0x0000;

	/* Check overrun error in RIS register */
	if (IP_SSP_GetRawIntStatus(pSSP, SSP_RORRIS) == SET) {
		return ERROR;
	}

	if ((xf_setup->tx_cnt != xf_setup->length) || (xf_setup->rx_cnt != xf_setup->length)) {
		/* check if RX FIFO contains data */
		SSP_ReadFromFifo(pSSP, xf_setup, rDat, 1);

		while ((IP_SSP_GetStatus(pSSP, SSP_STAT_TNF)) && (xf_setup->tx_cnt != xf_setup->length)) {
			/* Write data to buffer */
			SSP_Write2Fifo(pSSP, xf_setup, 1);

			/* Check overrun error in RIS register */
			if (IP_SSP_GetRawIntStatus(pSSP, SSP_RORRIS) == SET) {
				return ERROR;
			}

			/*  Check for any data available in RX FIFO */
			SSP_ReadFromFifo(pSSP, xf_setup, rDat, 1);
		}
		return SUCCESS;
	}

	return ERROR;
}

/* SSP Interrupt Read/Write with 16-bit frame width */
Status Chip_SSP_Int_RWFrames16Bits(LPC_SSP_T *pSSP, Chip_SSP_DATA_SETUP_T *xf_setup)
{
	uint16_t rDat = 0x0000;

	/* Check overrun error in RIS register */
	if (IP_SSP_GetRawIntStatus(pSSP, SSP_RORRIS) == SET) {
		return ERROR;
	}

	if ((xf_setup->tx_cnt != xf_setup->length) || (xf_setup->rx_cnt != xf_setup->length)) {
		/* check if RX FIFO contains data */
		SSP_ReadFromFifo(pSSP, xf_setup, rDat, 2);

		while ((IP_SSP_GetStatus(pSSP, SSP_STAT_TNF)) && (xf_setup->tx_cnt != xf_setup->length)) {
			/* Write data to buffer */
			SSP_Write2Fifo(pSSP, xf_setup, 2);

			/* Check overrun error in RIS register */
			if (IP_SSP_GetRawIntStatus(pSSP, SSP_RORRIS) == SET) {
				return ERROR;
			}

			/*  Check for any data available in RX FIFO			 */
			SSP_ReadFromFifo(pSSP, xf_setup, rDat, 2);
		}
		return SUCCESS;
	}

	return ERROR;
}

/* Set the SSP operating modes, master or slave */
void Chip_SSP_SetMaster(LPC_SSP_T *pSSP, bool master)
{
	if (master) {
		IP_SSP_Set_Mode(pSSP, SSP_MODE_MASTER);
	}
	else {
		IP_SSP_Set_Mode(pSSP, SSP_MODE_SLAVE);
	}
}

/* Determine clock for uart BASED ON SELECTED uart */
static CHIP_SYSCTL_CLOCK_T Chip_SSP_DetermineClk(LPC_SSP_T *pSSP) {

	CHIP_SYSCTL_CLOCK_T clk;

	/* Pick clock for uart BASED ON SELECTED uart */
	if (pSSP == LPC_SSP1) {
		clk = SYSCTL_CLOCK_SSP1;
	}
#if !defined(CHIP_LPC175X_6X)
	else if (pSSP == LPC_SSP2) {
		clk = SYSCTL_CLOCK_SSP2;
	}
#endif
	else {
		clk = SYSCTL_CLOCK_SSP0;
	}
	return clk;
}

/* Determine SSP clock based in selected SSP */
static uint32_t Chip_SSP_GetClockRate(LPC_SSP_T *pSSP)
{
#if !defined(CHIP_LPC175X_6X)
	return Chip_Clock_GetPeripheralClockRate();
#else
	/* Pick clock for uart BASED ON SELECTED uart */
	if (pSSP == LPC_SSP1) {
		return Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_SSP1);
	}

	else {
		return Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_SSP0);
	}
#endif
}

/* Set the clock frequency for SSP interface */
void Chip_SSP_SetBitRate(LPC_SSP_T *pSSP, uint32_t bitRate)
{
	uint32_t ssp_clk, cr0_div, cmp_clk, prescale;

	/* Get SSP clock rate */
	ssp_clk = Chip_SSP_GetClockRate(pSSP);

	cr0_div = 0;
	cmp_clk = 0xFFFFFFFF;
	prescale = 2;

	while (cmp_clk > bitRate) {
		cmp_clk = ssp_clk / ((cr0_div + 1) * prescale);
		if (cmp_clk > bitRate) {
			cr0_div++;
			if (cr0_div > 0xFF) {
				cr0_div = 0;
				prescale += 2;
			}
		}
	}

	IP_SSP_Set_ClockRate(pSSP, cr0_div, prescale);
}

/* Initialize the SSP */
void Chip_SSP_Init(LPC_SSP_T *pSSP)
{
	/* Enable SSP clocking. SSP base clock(s) must already be enabled */
	Chip_Clock_EnablePeriphClock(Chip_SSP_DetermineClk(pSSP));

	IP_SSP_Set_Mode(pSSP, SSP_MODE_MASTER);
	IP_SSP_SetFormat(pSSP, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_CPHA0_CPOL0);
	Chip_SSP_SetBitRate(pSSP, 100000);
}

/* De-initializes the SSP peripheral */
void Chip_SSP_DeInit(LPC_SSP_T *pSSP)
{
	IP_SSP_DeInit(pSSP);

	/* Disable SSP clocking */
	Chip_Clock_DisablePeriphClock(Chip_SSP_DetermineClk(pSSP));
}
