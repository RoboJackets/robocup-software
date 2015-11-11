/*
 * @brief ADC Registers and control functions
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

#include "adc_001.h"

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

/* Initialize the ADC */
void IP_ADC_Init(IP_ADC_001_T *pADC, uint8_t div, uint8_t bitsAcc, uint32_t flag)
{
	uint32_t cr;
#if defined(ADC_TRIM_SUPPORT)
	pADC->ADTRM = 0xF00;
#endif
	pADC->INTEN = 0;		/* Disable all interrupts */

	cr = pADC->CR & (~ADC_CONFIG_MASK);
#if defined(ADC_ACC_12BITS)
	cr |= ADC_CR_CLKDIV(div) | flag;
#else
	cr |= ADC_CR_CLKDIV(div) | ADC_CR_BITACC(bitsAcc) | flag;
#endif /*defined(ADC_ACC_12BITS)*/
	
	pADC->CR = cr;
}

/* Shutdown ADC */
void IP_ADC_DeInit(IP_ADC_001_T *pADC)
{
	pADC->INTEN = 0x00000100;
	pADC->CR = 0;
}

/* Set burst mode for ADC */
void IP_ADC_SetBurstMode(IP_ADC_001_T *pADC, FunctionalState NewState)
{
#if defined(CHIP_LPC1347)
	pADC->INTEN &= ~(1<<8);
#endif
	if (NewState == DISABLE) {
		pADC->CR &= ~ADC_CR_BURST;
	}
	else {
		pADC->CR |= ADC_CR_BURST;
	}
}

/* Get the ADC value */
Status IP_ADC_Get_Val(IP_ADC_001_T *pADC, uint8_t channel, uint16_t *data)
{
	uint32_t temp;
	temp = pADC->DR[channel];
	if (!ADC_DR_DONE(temp)) {
		return ERROR;
	}
	/*	if(ADC_DR_OVERRUN(temp) && (pADC->CR & ADC_CR_BURST)) */
	/*	return ERROR; */
	*data = (uint16_t) ADC_DR_RESULT(temp);
	return SUCCESS;
}

/* Get ADC Channel status from ADC data register */
FlagStatus IP_ADC_GetStatus(IP_ADC_001_T *pADC, uint8_t channel, uint32_t StatusType)
{
	switch (StatusType) {
	case ADC_DR_DONE_STAT:
		return (pADC->STAT & (1UL << channel)) ? SET : RESET;

	case ADC_DR_OVERRUN_STAT:
		channel += 8;
		return (pADC->STAT & (1UL << channel)) ? SET : RESET;

	case ADC_DR_ADINT_STAT:
		return pADC->STAT >> 16 ? SET : RESET;

	default:
		break;
	}
	return RESET;
}

/* Set the edge start condition */
void IP_ADC_EdgeStartConfig(IP_ADC_001_T *pADC, uint8_t edge_mode)
{
	if (edge_mode) {
		pADC->CR |= ADC_CR_EDGE;
	}
	else {
		pADC->CR &= ~ADC_CR_EDGE;
	}
}

/* Enable/Disable ADC channel number */
void IP_ADC_SetChannelNumber(IP_ADC_001_T *pADC, uint8_t channel, FunctionalState NewState)
{
	if (NewState == ENABLE) {
		pADC->CR |= ADC_CR_CH_SEL(channel);
	}
	else {
		pADC->CR &= ~ADC_CR_START_MASK;
		pADC->CR &= ~ADC_CR_CH_SEL(channel);
	}
}

/* Set start mode for ADC */
void IP_ADC_SetStartMode(IP_ADC_001_T *pADC, uint8_t start_mode)
{
	uint32_t temp;
	temp = pADC->CR & (~ADC_CR_START_MASK);
	pADC->CR = temp | (ADC_CR_START_MODE_SEL((uint32_t) start_mode));
}

/* Enable/Disable interrupt for ADC channel */
void IP_ADC_Int_Enable(IP_ADC_001_T *pADC, uint8_t channel, FunctionalState NewState)
{
	if (NewState == ENABLE) {
		pADC->INTEN |= (1UL << channel);
	}
	else {
		pADC->INTEN &= (~(1UL << channel));
	}
}
