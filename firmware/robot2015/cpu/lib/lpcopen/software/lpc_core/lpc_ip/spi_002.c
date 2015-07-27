/*
 * @brief SPI Registers and control functions
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

#include "spi_002.h"

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

/* Initialize SPI peripheral block */
void IP_SPI_Init(IP_SPI_002_T *pSPI, IP_SPI_CONFIG_T *pConfig)
{
	uint32_t EnStat = pSPI->CFG & SPI_CFG_SPI_EN;

	/* Disable before update CFG register */
	if (EnStat) {
		IP_SPI_Disable(pSPI);
	}

	/* SPI Configurate */
	pSPI->CFG = ((uint32_t) pConfig->ClockMode) | ((uint32_t) pConfig->DataOrder) | ((uint32_t) pConfig->Mode) |
				((uint32_t) pConfig->SSELPol);

	/* Rate Divider setting */
	pSPI->DIV = SPI_DIV_VAL(pConfig->ClkDiv);

	/* Clear status flag*/
	IP_SPI_ClearStatus(pSPI, SPI_STAT_CLR_RXOV | SPI_STAT_CLR_TXUR | SPI_STAT_CLR_SSA | SPI_STAT_CLR_SSD);

	/* Return the previous state */
	if (EnStat) {
		IP_SPI_Enable(pSPI);
	}
}

/* Configure SPI Delay parameters */
void IP_SPI_DelayConfig(IP_SPI_002_T *pSPI, IP_SPI_DELAY_CONFIG_T *pConfig)
{
	pSPI->DLY = SPI_DLY_PRE_DELAY(pConfig->PreDelay);
	pSPI->DLY |= SPI_DLY_POST_DELAY(pConfig->PostDelay);
	pSPI->DLY |= SPI_DLY_FRAME_DELAY(pConfig->FrameDelay);
	if (pConfig->TransferDelay) {
		pSPI->DLY |= SPI_DLY_TRANSFER_DELAY(pConfig->TransferDelay - 1);
	}
}
