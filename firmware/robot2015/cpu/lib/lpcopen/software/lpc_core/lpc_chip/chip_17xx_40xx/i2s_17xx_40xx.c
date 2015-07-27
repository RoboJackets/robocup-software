/*
 * @brief LPC17xx/40xx I2S driver
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

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Initialize the I2S interface */
void Chip_I2S_Init(LPC_I2S_T *pI2S)
{
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_I2S);
	IP_I2S_Init(pI2S);
}

/* Shutdown I2S */
void Chip_I2S_DeInit(LPC_I2S_T *pI2S)
{
	IP_I2S_DeInit(pI2S);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_I2S);
}

/* Configure I2S for Audio Format input */
Status Chip_I2S_Config(LPC_I2S_T *pI2S, uint8_t TRMode, Chip_I2S_Audio_Format_T *audio_format)
{
	uint32_t pClk;
	uint32_t x, y;
	uint64_t divider;
	uint16_t dif;
	uint16_t x_divide = 0, y_divide = 0;
	uint32_t N;
	uint16_t err, ErrorOptimal = 0xFFFF;

	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_I2S);
#if defined(CHIP_LPC175X_6X)
	pClk = Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_I2S);
#else
	pClk = Chip_Clock_GetPeripheralClockRate();
#endif

	/* divider is a fixed point number with 16 fractional bits */
	divider = (((uint64_t) (audio_format->SampleRate) * 2 * (audio_format->WordWidth) * 2) << 16) / pClk;
	/* find N that make x/y <= 1 -> divider <= 2^16 */
	for (N = 64; N > 0; N--) {
		if ((divider * N) < (1 << 16)) {
			break;
		}
	}
	if (N == 0) {
		return ERROR;
	}
	divider *= N;
	for (y = 255; y > 0; y--) {
		x = y * divider;
		if (x & (0xFF000000)) {
			continue;
		}
		dif = x & 0xFFFF;
		if (dif > 0x8000) {
			err = 0x10000 - dif;
		}
		else {
			err = dif;
		}
		if (err == 0) {
			y_divide = y;
			break;
		}
		else if (err < ErrorOptimal) {
			ErrorOptimal = err;
			y_divide = y;
		}
	}
	x_divide = ((uint64_t) y_divide * (audio_format->SampleRate) * 2 * (audio_format->WordWidth) * N * 2) / pClk;
	if (x_divide >= 256) {
		x_divide = 0xFF;
	}
	if (x_divide == 0) {
		x_divide = 1;
	}
	if (audio_format->WordWidth <= 8) {
		IP_I2S_SetWordWidth(pI2S, TRMode, I2S_WORDWIDTH_8);
	}
	else if (audio_format->WordWidth <= 16) {
		IP_I2S_SetWordWidth(pI2S, TRMode, I2S_WORDWIDTH_16);
	}
	else {
		IP_I2S_SetWordWidth(pI2S, TRMode, I2S_WORDWIDTH_32);
	}
	IP_I2S_SetMono(pI2S, TRMode, (audio_format->ChannelNumber) == 1 ? I2S_MONO : I2S_STEREO);
	IP_I2S_SetMasterSlaveMode(pI2S, TRMode, I2S_MASTER_MODE);
	IP_I2S_SetWS_Halfperiod(pI2S, TRMode, audio_format->WordWidth - 1);
	IP_I2S_ModeConfig(pI2S, TRMode, I2S_TXMODE_CLKSEL(0), !I2S_TXMODE_4PIN_ENABLE, !I2S_TXMODE_MCENA);
	IP_I2S_SetBitRate(pI2S, TRMode, N - 1);
	IP_I2S_SetXYDivider(pI2S, TRMode, x_divide, y_divide);
	return SUCCESS;
}

/* Enable/Disable Interrupt with a specific FIFO depth */
void Chip_I2S_Int_Cmd(LPC_I2S_T *pI2S, uint8_t TRMode, FunctionalState NewState, uint8_t FIFO_Depth)
{
	IP_I2S_InterruptCmd(pI2S, TRMode, NewState);
	IP_I2S_SetFIFODepthIRQ(pI2S, TRMode, FIFO_Depth);
}

/* Enable/Disable DMA with a specific FIFO depth */
void    Chip_I2S_DMA_Cmd(LPC_I2S_T *pI2S,
						 uint8_t TRMode,
						 uint8_t DMANum,
						 FunctionalState NewState,
						 uint8_t FIFO_Depth)
{
	IP_I2S_SetFIFODepthDMA(pI2S, TRMode, (IP_I2S_DMARequestNumber_T) DMANum, FIFO_Depth);
	IP_I2S_DMACmd(pI2S, (IP_I2S_DMARequestNumber_T) DMANum, TRMode, NewState);
}
