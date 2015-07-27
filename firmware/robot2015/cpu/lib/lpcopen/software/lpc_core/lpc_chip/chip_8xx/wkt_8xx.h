/*
 * @brief LPC8xx Self Wakeup Timer (WKT) chip driver
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

#ifndef __WKT_8XX_H_
#define __WKT_8XX_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup WKT_8XX CHIP: LPC8xx Self Wakeup Timer (WKT) driver
 * @ingroup CHIP_8XX_Drivers
 * This driver provides Self Wakeup Timer support for the device.
 * The driver requires the following IP drivers:<br>
 * @ref IP_WKT_001<br>
 * @{
 */

/**
 * WKT Clock source values enum
 */
typedef enum CHIP_WKT_CLKSRC {
	WKT_CLKSRC_DIVIRC = 0,	/*!< Divided IRC clock - runs at 750kHz */
	WKT_CLKSRC_10KHZ = 1	/*!< Low power clock - runs at 10kHz */
} CHIP_WKT_CLKSRC_T;

/**
 * @brief	Clear WKT interrupt status
 * @param	pWKT	: Pointer to WKT register block
 * @return	Nothing
 */
STATIC INLINE void Chip_WKT_ClearIntStatus(LPC_WKT_T *pWKT)
{
	IP_WKT_ClearIntStatus(pWKT);
}

/**
 * @brief	Clear and stop WKT counter
 * @param	pWKT	: Pointer to WKT register block
 * @return	Nothing
 */
STATIC INLINE void Chip_WKT_Stop(LPC_WKT_T *pWKT)
{
	IP_WKT_Stop(pWKT);
}

/**
 * @brief	Start wake-up timer interrupt, set clock source, set timer interval
 * @param	pWKT	: Pointer to WKT register block
 * @param	clkSrc	: Clock source
 * @param	cntVal	: Timer interval
 * @return	None
 */
void Chip_WKT_Start(LPC_WKT_T *pWKT, CHIP_WKT_CLKSRC_T clkSrc, uint32_t cntVal);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __WKT_8XX_H_ */
