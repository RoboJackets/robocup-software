/*
 * @brief LPC8xx UART driver
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

#ifndef __UART_8XX_H_
#define __UART_8XX_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup UART_8XX CHIP: LPC8xx UART Driver
 * @ingroup CHIP_8XX_Drivers
 * This driver provides UART support for the device.
 * The driver requires the following IP drivers:<br>
 * @ref IP_USART_002<br>
 * @{
 */

/**
 * @brief	Configure the UART protocol
 * @param	pUART	: The base of UART peripheral on the chip
 * @param	Databits: Data size for the USART
 * @param	Parity	: Parity to be used by the USART
 * @param	Stopbits: Number of stop bits appended to transmitted data
 * @return	Nothing
 */
STATIC INLINE void Chip_UART_ConfigData(LPC_USART_T *pUART, IP_UART_002_DATALEN_T Databits,
										IP_UART_002_PARITY_T Parity,
										IP_UART_002_STOPLEN_T Stopbits)
{
	IP_UART_Config(pUART, Databits, Parity, Stopbits);
}

/**
 * @brief	Enable/Disable UART Interrupts
 * @param	pUART			: The base of UART peripheral on the chip
 * @param	UARTIntCfg		: Specifies the interrupt flag
 * @param	NewState		: New state, ENABLE or DISABLE
 * @return	Nothing
 */
STATIC INLINE void Chip_UART_IntEnable(LPC_USART_T *pUART,
									   uint32_t UARTIntCfg,
									   FunctionalState NewState)
{
	IP_UART_IntEnable(pUART, UARTIntCfg, NewState);
}

/**
 * @brief	Get UART interrupt status
 * @param	pUART			: The base of UART peripheral on the chip
 * @return	The Interrupt status register of UART
 */
STATIC INLINE uint32_t Chip_UART_GetIntStatus(LPC_USART_T *pUART)
{
	return IP_UART_GetIntStatus(pUART);
}

/**
 * @brief	Initialize the UART peripheral
 * @param	pUART	: The base of UART peripheral on the chip
 * @return	Nothing
 */
void Chip_UART_Init(LPC_USART_T *pUART);

/**
 * @brief	Set baud rate for UART
 * @param	pUART	: The base of UART peripheral on the chip
 * @param	baudrate: Baud rate to be set
 * @return	Nothing
 */
void Chip_UART_SetBaudRate(LPC_USART_T *pUART, uint32_t baudrate);

/**
 * @brief	Transmit a 8 bits data
 * @param	pUART	: The base of UART peripheral on the chip
 * @param	data	: 8 bits data to be sent
 * @return	SUCCESS or ERROR
 */
Status Chip_UART_SendByte(LPC_USART_T *pUART, uint8_t data);

/**
 * @brief	Receive a 8 bits data
 * @param	pUART	: The base of UART peripheral on the chip
 * @param	Data	: 8 bits data received
 * @return	SUCCESS or ERROR
 */
Status Chip_UART_ReceiveByte(LPC_USART_T *pUART, uint8_t *Data);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __UART_8XX_H_ */
