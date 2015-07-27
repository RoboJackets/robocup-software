/*
 * @brief LPC18xx/43xx A/D conversion driver
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

/* Returns the clock for the selected ADC */
STATIC CHIP_CCU_CLK_T Chip_ADC_GetClk(LPC_ADC_T *pADC)
{
	CHIP_CCU_CLK_T adcclk;

	if (pADC == LPC_ADC0) {
		adcclk = CLK_APB3_ADC0;
	}
	else {
		adcclk = CLK_APB3_ADC1;
	}

	return adcclk;
}

/* Get divider value */
STATIC uint8_t getAdcClkDiv(LPC_ADC_T *pADC, bool burstMode, uint32_t adcRate,uint8_t clks)
{
	uint32_t adcBlockFreq;
	uint32_t fullAdcRate;
	uint8_t div;

	/* The APB clock (PCLK_ADC0) is divided by (CLKDIV+1) to produce the clock for
	   A/D converter, which should be less than or equal to 4.5MHz.
	   A fully conversion requires (bits_accuracy+1) of these clocks.
	   ADC Clock = PCLK_ADC0 / (CLKDIV + 1);
	   ADC rate = ADC clock / (the number of clocks required for each conversion);
	 */
	adcBlockFreq = Chip_Clock_GetRate(Chip_ADC_GetClk(pADC));
	if(burstMode)
		fullAdcRate = adcRate * clks;
	else
		fullAdcRate = adcRate * 11;

	/* Get the round value by fomular: (2*A + B)/(2*B) */
	div = ((adcBlockFreq * 2 + fullAdcRate) / (fullAdcRate * 2)) - 1;
	return div;
}


/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Shutdown ADC */
void Chip_ADC_DeInit(LPC_ADC_T *pADC)
{
	IP_ADC_DeInit(pADC);
	Chip_Clock_Disable(Chip_ADC_GetClk(pADC));
}

/* Initialize the ADC peripheral and the ADC setup structure to default value */
void Chip_ADC_Init(LPC_ADC_T *pADC, ADC_Clock_Setup_T *ADCSetup)
{
	uint8_t div;
	CHIP_CCU_CLK_T adcclk = Chip_ADC_GetClk(pADC);

	/* Enable ADC clocking */
	Chip_Clock_EnableOpts(adcclk, true, true, 1);

	ADCSetup->adcRate = 400000;
	ADCSetup->bitsAccuracy = ADC_10BITS;
	ADCSetup->burstMode = false;
	div = getAdcClkDiv(pADC, false, 400000, 11);
	IP_ADC_Init(pADC, div, ADC_10BITS, ADC_CR_PDN);
}

/* Select the mode starting the AD conversion */
void Chip_ADC_Set_StartMode(LPC_ADC_T *pADC, CHIP_ADC_START_MODE_T mode, CHIP_ADC_EDGE_CFG_T EdgeOption)
{
	if ((mode != ADC_START_NOW) && (mode != ADC_NO_START)) {
		IP_ADC_EdgeStartConfig(pADC, (uint8_t) EdgeOption);
	}
	IP_ADC_SetStartMode(pADC, (uint8_t) mode);
}

/* Set the ADC Sample rate */
void Chip_ADC_Set_SampleRate(LPC_ADC_T *pADC, ADC_Clock_Setup_T *ADCSetup, uint32_t rate)
{
	uint8_t div;
	ADCSetup->adcRate = rate;
	div = getAdcClkDiv(pADC, ADCSetup->burstMode, rate, (11- ADCSetup->bitsAccuracy));
	IP_ADC_Init(pADC, div, ADCSetup->bitsAccuracy, ADC_CR_PDN);
}

/* Set the ADC accuracy bits */
void Chip_ADC_Set_Resolution(LPC_ADC_T *pADC, ADC_Clock_Setup_T *ADCSetup, CHIP_ADC_RESOLUTION_T resolution)
{
	ADCSetup->bitsAccuracy = resolution;
	Chip_ADC_Set_SampleRate(pADC, ADCSetup,ADCSetup->adcRate);
}

/* Enable or disable the ADC channel on ADC peripheral */
void Chip_ADC_Channel_Enable_Cmd(LPC_ADC_T *pADC, CHIP_ADC_CHANNEL_T channel, FunctionalState NewState)
{
	IP_ADC_SetChannelNumber(pADC, channel, NewState);
}

/* Enable burst mode */
void Chip_ADC_Burst_Cmd(LPC_ADC_T *pADC, FunctionalState NewState)
{
	IP_ADC_SetStartMode(pADC, ADC_NO_START);
	IP_ADC_SetBurstMode(pADC, NewState);
}

/* Read the ADC value and convert it to 8bits value */
Status Chip_ADC_Read_Byte(LPC_ADC_T *pADC, CHIP_ADC_CHANNEL_T channel, uint8_t *data)
{
	uint16_t temp;
	Status rt;

	rt = IP_ADC_Get_Val(pADC, channel, &temp);
	*data = (uint8_t) temp;

	return rt;
}
