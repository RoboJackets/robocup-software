/*
 * @brief LPC8xx GPIO driver
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

#ifndef __GPIO_8XX_H_
#define __GPIO_8XX_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup GPIO_8XX CHIP: LPC8xx GPIO Driver
 * @ingroup CHIP_8XX_Drivers
 * This driver provides GPIO support for the device.
 * The driver requires the following IP drivers:<br>
 * @ref IP_GPIO_001<br>
 * @{
 */

#define LPC8XX_PORT_NUM     (0)

typedef enum CHIP_PIN_DIR {PIN_DIR_INPUT = 0, PIN_DIR_OUTPUT = 1} CHIP_PIN_DIR_T;

/**
 * @brief	Initialize GPIO block
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @return	Nothing
 */
STATIC INLINE void Chip_GPIO_Init(LPC_GPIO_T *pGPIO)
{
	IP_GPIO_Init(pGPIO);
}

/**
 * @brief	Set a GPIO port/bit state
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: GPIO port to set
 * @param	bit		: GPIO bit to set
 * @param	setting	: true for high, false for low
 * @return	Nothing
 */
STATIC INLINE void Chip_GPIO_WritePortBit(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t bit, bool setting)
{
	IP_GPIO_WritePortBit(pGPIO, port, bit, setting);
}

/**
 * @brief	Seta GPIO direction
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: GPIO port to set
 * @param	bit		: GPIO bit to set
 * @param	setting	: true for output, false for input
 * @return	Nothing
 */
STATIC INLINE void Chip_GPIO_WriteDirBit(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t bit, bool setting)
{
	IP_GPIO_WriteDirBit(pGPIO, port, bit, setting);
}

/**
 * @brief	Read a GPIO state
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: GPIO port to read
 * @param	bit		: GPIO bit to read
 * @return	true of the GPIO is high, false if low
 */
STATIC INLINE bool Chip_GPIO_ReadPortBit(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t bit)
{
	return IP_GPIO_ReadPortBit(pGPIO, port, bit);
}

/**
 * @brief	Read a GPIO direction (out or in)
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: GPIO port to read
 * @param	bit		: GPIO bit to read
 * @return	true of the GPIO is an output, false if input
 */
STATIC INLINE bool Chip_GPIO_ReadDirBit(LPC_GPIO_T *pGPIO, uint32_t port, uint8_t bit)
{
	return IP_GPIO_ReadDirBit(pGPIO, port, bit);
}

/**
 * @brief	Set Direction for a GPIO port
 * @param	pGPIO		: The base of GPIO peripheral on the chip
 * @param	portNum		: Port Number
 * @param	bitValue	: GPIO bit to set
 * @param	out			: Direction value, 0 = input, !0 = output
 * @return	None
 * @note	Bits set to '0' are not altered.
 */
void Chip_GPIO_SetDir(LPC_GPIO_T *pGPIO, uint8_t portNum, uint32_t bitValue, uint8_t out);

/**
 * @brief	Set a GPIO port/bit to the high state
 * @param	pGPIO		: The base of GPIO peripheral on the chip
 * @param	portNum		: Port number
 * @param	bitValue	: Bit(s) in the port to set high
 * @return	None
 * @note	Any bit set as a '0' will not have it's state changed. This only
 * applies to ports configured as an output.
 */
STATIC INLINE void Chip_FIO_SetValue(LPC_GPIO_T *pGPIO, uint8_t portNum, uint32_t bitValue)
{
	/* Same with GPIO_SetValue() */
	pGPIO->SET[portNum] = bitValue;
}

/**
 * @brief	Set a GPIO port/bit to the low state
 * @param	pGPIO		: The base of GPIO peripheral on the chip
 * @param	portNum		: Port number
 * @param	bitValue	: Bit(s) in the port to set low
 * @return	None
 * @note	Any bit set as a '0' will not have it's state changed. This only
 * applies to ports configured as an output.
 */
STATIC INLINE void Chip_FIO_ClearValue(LPC_GPIO_T *pGPIO, uint8_t portNum, uint32_t bitValue)
{
	/* Same with GPIO_ClearValue() */
	pGPIO->CLR[portNum] = bitValue;
}

/**
 * @brief	Read current bit states for the selected port
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	portNum	: Port number to read
 * @return	Current value of GPIO port
 * @note	The current states of the bits for the port are read, regardless of
 * whether the GPIO port bits are input or output.
 */
STATIC INLINE uint32_t Chip_FIO_ReadValue(LPC_GPIO_T *pGPIO, uint8_t portNum)
{
	/* Same with GPIO_ReadValue() */
	return pGPIO->PIN[portNum];
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __GPIO_8XX_H_ */
