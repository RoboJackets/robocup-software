/*
 * @brief LPC8xx Multi-Rate Timer (MRT) registers and driver functions
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

#ifndef __MRT_8XX_H_
#define __MRT_8XX_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup MRT_8XX CHIP: LPC8xx MRT register block and driver
 * @ingroup CHIP_8XX_Drivers
 * This driver provides Multi-Rate Timer (MRT) support for the device.
 * The driver requires the following IP drivers:<br>
 * @ref IP_MRT_001<br>
 * @{
 */

/**
 * @brief LPC8xx MRT chip configuration
 */
#define LPC_8XX_MRT_CHANNELS      (4)
#define LPC_MRT_NO_IDLE_CHANNEL   (0x40)

/**
 * @brief MRT register block structure
 */
typedef struct {
	IP_MRT_001_T CHANNEL[LPC_8XX_MRT_CHANNELS];
	uint32_t unused[45];
	__O  uint32_t IDLE_CH;
	__IO uint32_t IRQ_FLAG;
} LPC_MRT_T;

typedef IP_MRT_001_T        LPC_MRT_CHANNEL_T;
typedef IP_MRT_001_MODE_T   LPC_MRT_MODE_T;

#define LPC_MRT_CH0         ((IP_MRT_001_T *) &LPC_MRT->CHANNEL[0])
#define LPC_MRT_CH1         ((IP_MRT_001_T *) &LPC_MRT->CHANNEL[1])
#define LPC_MRT_CH2         ((IP_MRT_001_T *) &LPC_MRT->CHANNEL[2])
#define LPC_MRT_CH3         ((IP_MRT_001_T *) &LPC_MRT->CHANNEL[3])

#define MRT0_INTFLAG        (1)
#define MRT1_INTFLAG        (2)
#define MRT2_INTFLAG        (4)
#define MRT3_INTFLAG        (8)

/**
 * @brief	Initializes the MRT
 * @return	Nothing
 */
STATIC INLINE void Chip_MRT_Init(void)
{
	/* Enable the clock to the register interface */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_MRT);

	/* Reset MRT */
	Chip_SYSCTL_PeriphReset(RESET_MRT);

	IP_MRT_Init();
}

/**
 * @brief	De-initializes the MRT Channel
 * @return	Nothing
 */
STATIC INLINE void Chip_MRT_DeInit(void)
{
	IP_MRT_DeInit();

	/* Disable the clock to the MRT */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_MRT);
}

/**
 * @brief	Returns the timer time interval value
 * @param	pMRT	: Pointer to selected MRT Channel
 * @return	Timer time interval value
 */
STATIC INLINE uint32_t Chip_MRT_GetInterval(LPC_MRT_CHANNEL_T *pMRT)
{
	return IP_MRT_GetInterval(pMRT);
}

/**
 * @brief	Sets the timer time interval value
 * @param	pMRT	 : Pointer to selected MRT Channel
 * @param   interval : The interval timeout (24-bits).
 * @return	Nothing
 * @note	Setting bit 31 in timer time interval register causes the time interval value
 * to load immediately, otherwise the time interval value will be loaded in
 * next timer cycle
 */
STATIC INLINE void Chip_MRT_SetInterval(LPC_MRT_CHANNEL_T *pMRT, uint32_t interval)
{
	IP_MRT_SetInterval(pMRT, interval);
}

/**
 * @brief	Returns the current timer value
 * @param	pMRT	: Pointer to selected MRT Channel
 * @return	The current timer value
 */
STATIC INLINE uint32_t Chip_MRT_GetTimer(LPC_MRT_CHANNEL_T *pMRT)
{
	return IP_MRT_GetTimer(pMRT);
}

/**
 * @brief	Returns true if the timer is enabled
 * @param	pMRT	: Pointer to selected MRT Channel
 * @return	True if enabled, Flase if not enabled
 */
STATIC INLINE bool Chip_MRT_GetEnabled(LPC_MRT_CHANNEL_T *pMRT)
{
	return IP_MRT_GetEnabled(pMRT);
}

/**
 * @brief	Enables the timer
 * @param	pMRT	: Pointer to selected MRT Channel
 * @return	Nothing
 */
STATIC INLINE void Chip_MRT_SetEnabled(LPC_MRT_CHANNEL_T *pMRT)
{
	IP_MRT_SetEnabled(pMRT);
}

/**
 * @brief	Returns the timer mode (repeat or one-shot)
 * @param	pMRT	: Pointer to selected MRT Channel
 * @return	The mode (repeat or one-shot).
 */
STATIC INLINE LPC_MRT_MODE_T Chip_MRT_GetMode(LPC_MRT_CHANNEL_T *pMRT)
{
	return (LPC_MRT_MODE_T) IP_MRT_GetMode(pMRT);
}

/**
 * @brief	Sets the timer mode (repeat or one-shot)
 * @param	pMRT	: Pointer to selected MRT Channel
 * @param   mode    : 0 = repeat, 1 = one-shot
 * @return	Nothing
 */
STATIC INLINE void Chip_MRT_SetMode(LPC_MRT_CHANNEL_T *pMRT, LPC_MRT_MODE_T mode)
{
	IP_MRT_SetMode(pMRT, (IP_MRT_001_MODE_T) mode);
}

/**
 * @brief	Check if the timer is configured in repeat mode
 * @param	pMRT	: Pointer to selected MRT Channel
 * @return	True if in repeat mode, False if in one-shot mode
 */
STATIC INLINE bool Chip_MRT_IsRepeatMode(LPC_MRT_CHANNEL_T *pMRT)
{
	return IP_MRT_IsRepeatMode(pMRT);
}

/**
 * @brief	Check if the timer is configured in one-shot mode
 * @param	pMRT	: Pointer to selected MRT Channel
 * @return	True if in one-shot mode, False if in repeat mode
 */
STATIC INLINE bool Chip_MRT_IsOneShotMode(LPC_MRT_CHANNEL_T *pMRT)
{
	return IP_MRT_IsOneShotMode(pMRT);
}

/**
 * @brief	Check if the timer has an interrupt pending
 * @param	pMRT	: Pointer to selected MRT Channel
 * @return	True if interrupt is pending, False if no interrupt is pending
 */
STATIC INLINE bool Chip_MRT_IntPending(LPC_MRT_CHANNEL_T *pMRT)
{
	return IP_MRT_IntPending(pMRT);
}

/**
 * @brief	Clears the pending interrupt (if any)
 * @param	pMRT	: Pointer to selected MRT Channel
 * @return	Nothing
 */
STATIC INLINE void Chip_MRT_IntClear(LPC_MRT_CHANNEL_T *pMRT)
{
	IP_MRT_IntClear(pMRT);
}

/**
 * @brief	Check if the timer is running
 * @param	pMRT	: Pointer to selected MRT Channel
 * @return	True if running, False if stopped
 */
STATIC INLINE bool Chip_MRT_Running(LPC_MRT_CHANNEL_T *pMRT)
{
	return IP_MRT_Running(pMRT);
}

/**
 * @brief	Returns the IDLE channel value
 * @return	IDLE channel value
 */
STATIC INLINE uint8_t Chip_MRT_GetIdleChannel(void)
{
	return (uint8_t) (LPC_MRT->IDLE_CH);
}

/**
 * @brief	Returns the channel number of IRQ pending channel
 * @return	IRQ pending channel number
 */
STATIC INLINE uint32_t Chip_MRT_GetIntPending(void)
{
	return LPC_MRT->IRQ_FLAG;
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __MRT_8XX_H_ */
