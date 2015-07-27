/*
 * @brief LPC11xx GPIO driver
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

#ifndef __GPIO_11XX_H_
#define __GPIO_11XX_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup GPIO_11XX CHIP: LPC11xx GPIO Driver
 * @ingroup CHIP_11XX_Drivers
 * @{
 */
#if !(defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX))
#define LPC_GPIO_PORTn_BASE(base, port)         ((LPC_GPIO_T *) (((uint32_t) base) + (port * 0x10000)))
#endif

/**
 * @brief	Initialize GPIO block
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @return	Nothing
 */
void Chip_GPIO_Init(LPC_GPIO_T *pGPIO);

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
#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
	IP_GPIO_WritePortBit(pGPIO, port, bit, setting);
#else
	IP_GPIO_WritePortBit(LPC_GPIO_PORTn_BASE(pGPIO, port), bit, setting);
#endif
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
#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
	IP_GPIO_WriteDirBit(pGPIO, port, bit, setting);
#else
	IP_GPIO_WriteDirBit(LPC_GPIO_PORTn_BASE(pGPIO, port), bit, setting);
#endif
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
#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
	return IP_GPIO_ReadPortBit(pGPIO, port, bit);
#else
	return IP_GPIO_ReadPortBit(LPC_GPIO_PORTn_BASE(pGPIO, port), bit);
#endif
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
#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
	return IP_GPIO_ReadDirBit(pGPIO, port, bit);
#else
	return IP_GPIO_ReadDirBit(LPC_GPIO_PORTn_BASE(pGPIO, port), bit);
#endif
}

/**
 * @brief	Enable GPIO Interrupt
 * @param	pGPIO		: The base of GPIO peripheral on the chip
 * @param	portNum		: GPIO port number interrupt, should be: 0 to 7
 * @param	bitValue	: GPIO bit to enable (Not used)
 * @param	mode		: Interrupt mode
 * @return	None
 */
STATIC INLINE void Chip_GPIO_IntCmd(LPC_GPIO_T *pGPIO, uint8_t portNum, uint8_t bitValue, IP_GPIOPININT_MODE_T mode)
{
#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
	IP_GPIOPININT_IntCmd(LPC_GPIO_PIN_INT, portNum, mode);
#else
	IP_GPIO_IntCmd(LPC_GPIO_PORTn_BASE(pGPIO, portNum), bitValue, mode);
#endif
}

/**
 * @brief	Get GPIO Interrupt Status
 * @param	pGPIO		: The base of GPIO peripheral on the chip
 * @param	portNum		: GPIO port number interrupt, should be: 0 to 7
 * @param	pinNum		: GPIO pin to check (Not used)
 * @param	mode		: Interrupt mode (Not used)
 * @return	true if interrupt is pending, otherwise false
 */
STATIC INLINE bool Chip_GPIO_IntGetStatus(LPC_GPIO_T *pGPIO, uint8_t portNum, uint8_t pinNum, uint8_t mode)
{
#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
	return IP_GPIOPININT_IntGetStatus(LPC_GPIO_PIN_INT, portNum);
#else
	return IP_GPIO_IntGetStatus(LPC_GPIO_PORTn_BASE(pGPIO, portNum), pinNum);
#endif
}

/**
 * @brief	Clear GPIO Interrupt (Edge interrupt cases only)
 * @param	pGPIO		: The base of GPIO peripheral on the chip
 * @param	portNum		: GPIO port number interrupt, should be: 0 to 7
 * @param	bitValue	: GPIO bit to clear (Not used)
 * @return	None
 */
STATIC INLINE void Chip_GPIO_IntClear(LPC_GPIO_T *pGPIO, uint8_t portNum, uint8_t bitValue)
{
#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
	IP_GPIOPININT_IntClear(LPC_GPIO_PIN_INT, portNum);
#else
	IP_GPIO_IntClear(LPC_GPIO_PORTn_BASE(pGPIO, portNum), bitValue);
#endif
}

#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
/**
 * @brief	GPIO Group Interrupt Pin Initialization
 * @param	pGPIOGPINT	: Pointer to GPIOIR register block
 * @param	portComb	: GPIO group combined enable, should be: 0 (OR functionality) and 1 (AND functionality)
 * @param	portTrigger	: GPIO group interrupt trigger, should be: 0 (Edge-triggered) 1 (Level triggered)
 * @return	None
 */
STATIC INLINE void Chip_GPIOGP_IntInit(IP_GPIOGROUPINT_001_T *pGPIOGPINT, uint8_t portComb, uint8_t portTrigger)
{
	IP_GPIOGP_IntInit(pGPIOGPINT, portComb, portTrigger);
}

/**
 * @brief	GPIO Group Interrupt Pin Add to Group
 * @param	pGPIOGPINT	: Pointer to GPIOIR register block
 * @param	portNum		: GPIO port number, should be 0 to 7
 * @param	pinNum		: GPIO pin number, should be 0 to 31
 * @param	activeMode	: GPIO active mode, should be 0 (active LOW) and 1 (active HIGH)
 * @return	None
 */
STATIC INLINE void Chip_GPIOGP_IntPinAdd(IP_GPIOGROUPINT_001_T *pGPIOGPINT,
										 uint8_t portNum,
										 uint8_t pinNum,
										 bool activeMode)
{
	IP_GPIOGP_IntPinAdd(pGPIOGPINT, portNum, pinNum, activeMode);
}

/**
 * @brief	GPIO Group Interrupt Pin Remove from Group
 * @param	pGPIOGPINT	: Pointer to GPIOIR register block
 * @param	portNum		: GPIO port number, should be 0 to 7
 * @param	pinNum		: GPIO pin number, should be 0 to 31
 * @return	None
 */
STATIC INLINE void Chip_GPIOGP_IntPinRemove(IP_GPIOGROUPINT_001_T *pGPIOGPINT, uint8_t portNum, uint8_t pinNum)
{
	IP_GPIOGP_IntPinRemove(pGPIOGPINT, portNum, pinNum);
}

/**
 * @brief	Get GPIO Group Interrupt Get Status
 * @param	pGPIOGPINT	: Pointer to GPIOIR register block
 * @return	true if interrupt is pending, otherwise false
 */
STATIC INLINE bool Chip_GPIOGP_IntGetStatus(IP_GPIOGROUPINT_001_T *pGPIOGPINT)
{
	return IP_GPIOGP_IntGetStatus(pGPIOGPINT);
}

/**
 * @brief	Clear GPIO Group Interrupt
 * @param	pGPIOGPINT	: Pointer to GPIOIR register block
 * @return	None
 */
STATIC INLINE void Chip_GPIOGP_IntClear(IP_GPIOGROUPINT_001_T *pGPIOGPINT)
{
	IP_GPIOGP_IntClear(pGPIOGPINT);
}

#endif /* defined(CHIP_LPC11UXX) || defined (CHIP_LPC11EXX) || defined (CHIP_LPC11AXX)*/
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

#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
/**
 * @brief	Set Direction for a GPIO port
 * @param	pGPIO		: The base of GPIO peripheral on the chip
 * @param	portNum		: Port Number
 * @param	bitValue	: GPIO bit to set
 * @param	out			: Direction value, 0 = input, !0 = output
 * @return	None
 * @note	Bits set to '0' are not altered.
 */
STATIC INLINE void Chip_FIO_SetDir(LPC_GPIO_T *pGPIO, uint8_t portNum, uint32_t bitValue, uint8_t out)
{
	/* Same with Chip_GPIO_SetDir() */
	Chip_GPIO_SetDir(pGPIO, portNum, bitValue, out);
}

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

#endif /*defined(CHIP_LPC11UXX) || defined (CHIP_LPC11EXX) || defined (CHIP_LPC11AXX)*/

/**
 * @brief	Set a GPIO port/bit to the high state
 * @param	pGPIO		: The base of GPIO peripheral on the chip
 * @param	portNum		: Port number
 * @param	bitValue	: Bit(s) in the port to set high
 * @return	None
 * @note	Any bit set as a '0' will not have it's state changed. This only
 * applies to ports configured as an output.
 */
STATIC INLINE void Chip_GPIO_SetValue(LPC_GPIO_T *pGPIO, uint8_t portNum, uint32_t bitValue)
{
#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
	pGPIO->SET[portNum] = bitValue;
#else
	IP_GPIO_WritePort(LPC_GPIO_PORTn_BASE(pGPIO, portNum), bitValue, bitValue);
#endif
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
STATIC INLINE void Chip_GPIO_ClearValue(LPC_GPIO_T *pGPIO, uint8_t portNum, uint32_t bitValue)
{
#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
	pGPIO->CLR[portNum] = bitValue;
#else
	IP_GPIO_WritePort(LPC_GPIO_PORTn_BASE(pGPIO, portNum), bitValue, ~bitValue);
#endif
}

/**
 * @brief	Read current bit states for the selected port
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	portNum	: Port number to read
 * @return	Current value of GPIO port
 * @note	The current states of the bits for the port are read, regardless of
 * whether the GPIO port bits are input or output.
 */
STATIC INLINE uint32_t Chip_GPIO_ReadValue(LPC_GPIO_T *pGPIO, uint8_t portNum)
{
#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
	return pGPIO->PIN[portNum];
#else
	return IP_GPIO_ReadPort(LPC_GPIO_PORTn_BASE(pGPIO, portNum));
#endif
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __GPIO_11XX_H_ */
