/*
 * @brief LPC11xx UART chip driver
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

#ifndef __UART_11XX_H_
#define __UART_11XX_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup PERIPH_11XX_UART CHIP: LPC11xx UART Driver
 * @ingroup CHIP_11XX_Drivers
 * @{
 */

/**
 * @brief	Initializes the pUART peripheral.
 * @param	pUART		: Pointer to selected pUART peripheral
 * @return	Nothing
 */
void Chip_UART_Init(LPC_USART_T *pUART);

/**
 * @brief	De-initializes the pUART peripheral.
 * @param	pUART		: Pointer to selected pUART peripheral
 * @return	Nothing
 */
void Chip_UART_DeInit(LPC_USART_T *pUART);

/**
 * @brief	Enable transmission on UART TxD pin
 * @param	pUART	: Pointer to selected pUART peripheral
 * @return Nothing
 */
STATIC INLINE void Chip_UART_TXEnable(LPC_USART_T *pUART)
{
	IP_UART_TXEnable(pUART);
}

/**
 * @brief	Disable transmission on UART TxD pin
 * @param	pUART	: Pointer to selected pUART peripheral
 * @return Nothing
 */
STATIC INLINE void Chip_UART_TXDisable(LPC_USART_T *pUART)
{
	IP_UART_TXDisable(pUART);
}

/**
 * @brief	Transmit a single byte through the UART peripheral
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	data	: Byte to transmit
 * @return	Nothing
 * @note	This function attempts to place a byte into the UART transmit
 *			FIFO or transmit hold register regard regardless of UART state.
 */
STATIC INLINE void Chip_UART_SendByte(LPC_USART_T *pUART, uint8_t data)
{
	IP_UART_SendByte(pUART, data);
}

/**
 * @brief	Get a single data from UART peripheral
 * @param	pUART	: Pointer to selected UART peripheral
 * @return	A single byte of data read
 * @note	This function reads a byte from the UART receive FIFO or
 *			receive hold register regard regardless of UART state. The
 *			FIFO status should be read first prior to using this function.
 */
STATIC INLINE uint8_t Chip_UART_ReadByte(LPC_USART_T *pUART)
{
	return IP_UART_ReadByte(pUART);
}

/**
 * @brief	Transmit a byte array through the UART peripheral (non-blocking)
 * @param	pUART		: Pointer to selected UART peripheral
 * @param	data		: Pointer to bytes to transmit
 * @param	numBytes	: Number of bytes to transmit
 * @return	The actual number of bytes placed into the FIFO
 * @note	This function places data into the transmit FIFO until either
 *			all the data is in the FIFO or the FIFO is full. This function
 *			will not block in the FIFO is full. The actual number of bytes
 *			placed into the FIFO is returned. This function ignores errors.
 */
int Chip_UART_Send(LPC_USART_T *pUART, const void *data, int numBytes);

/**
 * @brief	Read data through the UART peripheral (non-blocking)
 * @param	pUART		: Pointer to selected UART peripheral
 * @param	data		: Pointer to bytes array to fill
 * @param	numBytes	: Size of the passed data array
 * @return	The actual number of bytes read
 * @note	This function reads data from the receive FIFO until either
 *			all the data has been read or the passed buffer is completely full.
 *			This function will not block. This function ignores errors.
 */
int Chip_UART_Read(LPC_USART_T *pUART, void *data, int numBytes);

/**
 * @brief	Enable UART interrupts
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	intMask	: Or'ed Interrupts to enable in the Interrupt Enable Register (IER)
 * @return	Nothing
 * @note	Use an Or'ed value of UART_IER_* definitions with this call
 *			to enable specific UART interrupts. The Divisor Latch Access Bit
 *			(DLAB) in LCR must be cleared in order to access the IER register.
 *			This function doesn't alter the DLAB state.
 */
STATIC INLINE void Chip_UART_IntEnable(LPC_USART_T *pUART, uint32_t intMask)
{
	IP_UART_IntEnable(pUART, intMask);
}

/**
 * @brief	Disable UART interrupts
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	intMask	: Or'ed Interrupts to disable in the Interrupt Enable Register (IER)
 * @return	Nothing
 * @note	Use an Or'ed value of UART_IER_* definitions with this call
 *			to disable specific UART interrupts. The Divisor Latch Access Bit
 *			(DLAB) in LCR must be cleared in order to access the IER register.
 *			This function doesn't alter the DLAB state.
 */
STATIC INLINE void Chip_UART_IntDisable(LPC_USART_T *pUART, uint32_t intMask)
{
	IP_UART_IntDisable(pUART, intMask);
}

/**
 * @brief	Read the Interrupt Identification Register (IIR)
 * @param	pUART	: Pointer to selected UART peripheral
 * @return	Current pending interrupt status per the IIR register
 */
STATIC INLINE uint32_t Chip_UART_ReadIntIDReg(LPC_USART_T *pUART)
{
	return IP_UART_ReadIntIDReg(pUART);
}

/**
 * @brief	Setup the UART FIFOs
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	fcr		: FIFO control register setup OR'ed flags
 * @return	Nothing
 * @note	Use an Or'ed value of UART_FCR_* definitions with this call
 *			to select specific options. For example, to enable the FIFOs
 *			with a RX trip level of 8 characters, use something like
 *			(UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2).
 */
STATIC INLINE void Chip_UART_SetupFIFOS(LPC_USART_T *pUART, uint32_t fcr)
{
	IP_UART_SetupFIFOS(pUART, fcr);
}

/**
 * @brief	Configure data width, parity mode and stop bits
 * @param	pUART	: Pointer to selected pUART peripheral
 * @param	config	: UART configuration, Or'ed values of UART_LCR_* defines
 * @return	Nothing
 * @note	Select OR'ed config options for the UART from the UART_LCR_*
 *			definitions. For example, a configuration of 8 data bits, 1
 *			stop bit, and even (enabled) parity would be
 *			(UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_EN | UART_LCR_PARITY_EVEN).
 */
STATIC INLINE void Chip_UART_ConfigData(LPC_USART_T *pUART,
										uint32_t config)
{
	IP_UART_SetMode(pUART, config);
}

/**
 * @brief	Enable access to Divisor Latches
 * @param	pUART	: Pointer to selected UART peripheral
 * @return	Nothing
 */
STATIC INLINE void Chip_UART_EnableDivisorAccess(LPC_USART_T *pUART)
{
	IP_UART_EnableDivisorAccess(pUART);
}

/**
 * @brief	Disable access to Divisor Latches
 * @param	pUART	: Pointer to selected UART peripheral
 * @return	Nothing
 */
STATIC INLINE void Chip_UART_DisableDivisorAccess(LPC_USART_T *pUART)
{
	IP_UART_EnableDivisorAccess(pUART);
}

/**
 * @brief	Set LSB and MSB divisor latch registers
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	dll		: Divisor Latch LSB value
 * @param	dlm		: Divisor Latch MSB value
 * @return	Nothing
 * @note	The Divisor Latch Access Bit (DLAB) in LCR must be set in
 *			order to access the USART Divisor Latches. This function
 *			doesn't alter the DLAB state.
 */
STATIC INLINE void Chip_UART_SetDivisorLatches(LPC_USART_T *pUART, uint8_t dll, uint8_t dlm)
{
	IP_UART_SetDivisorLatches(pUART, dll, dlm);
}

/**
 * @brief	Return modem control register/status
 * @param	pUART	: Pointer to selected UART peripheral
 * @return	Modem control register (status)
 * @note	Mask bits of the returned status value with UART_MCR_*
 *			definitions for specific statuses.
 */
STATIC INLINE uint32_t Chip_UART_ReadModemControl(LPC_USART_T *pUART)
{
	return IP_UART_ReadModemControl(pUART);
}

/**
 * @brief	Set modem control register/status
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	mcr		: Modem control register flags to set
 * @return	Nothing
 * @note	Use an Or'ed value of UART_MCR_* definitions with this
 *			call to set specific options.
 */
STATIC INLINE void Chip_UART_SetModemControl(LPC_USART_T *pUART, uint32_t mcr)
{
	IP_UART_SetModemControl(pUART, mcr);
}

/**
 * @brief	Clear modem control register/status
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	mcr		: Modem control register flags to clear
 * @return	Nothing
 * @note	Use an Or'ed value of UART_MCR_* definitions with this
 *			call to clear specific options.
 */
STATIC INLINE void Chip_UART_ClearModemControl(LPC_USART_T *pUART, uint32_t mcr)
{
	IP_UART_ClearModemControl(pUART, mcr);
}

/**
 * @brief	Return Line Status register/status (LSR)
 * @param	pUART	: Pointer to selected UART peripheral
 * @return	Line Status register (status)
 * @note	Mask bits of the returned status value with UART_LSR_*
 *			definitions for specific statuses.
 */
STATIC INLINE uint32_t Chip_UART_ReadLineStatus(LPC_USART_T *pUART)
{
	return IP_UART_ReadLineStatus(pUART);
}

/**
 * @brief	Return Modem Status register/status (MSR)
 * @param	pUART	: Pointer to selected UART peripheral
 * @return	Modem Status register (status)
 * @note	Mask bits of the returned status value with UART_MSR_*
 *			definitions for specific statuses.
 */
STATIC INLINE uint32_t Chip_UART_ReadModemStatus(LPC_USART_T *pUART)
{
	return IP_UART_ReadModemStatus(pUART);
}

/**
 * @brief	Write a byte to the scratchpad register
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	data	: Byte value to write
 * @return	Nothing
 */
STATIC INLINE void Chip_UART_SetScratch(LPC_USART_T *pUART, uint8_t data)
{
	IP_UART_SetScratch(pUART, data);
}

/**
 * @brief	Returns current byte value in the scratchpad register
 * @param	pUART	: Pointer to selected UART peripheral
 * @return	Byte value read from scratchpad register
 */
STATIC INLINE uint8_t Chip_UART_ReadScratch(LPC_USART_T *pUART)
{
	return IP_UART_ReadScratch(pUART);
}

/**
 * @brief	Set autobaud register options
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	acr		: Or'ed values to set for ACR register
 * @return	Nothing
 * @note	Use an Or'ed value of UART_ACR_* definitions with this
 *			call to set specific options.
 */
STATIC INLINE void Chip_UART_SetAutoBaudReg(LPC_USART_T *pUART, uint32_t acr)
{
	IP_UART_SetAutoBaudReg(pUART, acr);
}

/**
 * @brief	Clear autobaud register options
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	acr		: Or'ed values to clear for ACR register
 * @return	Nothing
 * @note	Use an Or'ed value of UART_ACR_* definitions with this
 *			call to clear specific options.
 */
STATIC INLINE void Chip_UART_ClearAutoBaudReg(LPC_USART_T *pUART, uint32_t acr)
{
	IP_UART_ClearAutoBaudReg(pUART, acr);
}

/**
 * @brief	Set RS485 control register options
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	ctrl	: Or'ed values to set for RS485 control register
 * @return	Nothing
 * @note	Use an Or'ed value of UART_RS485CTRL_* definitions with this
 *			call to set specific options.
 */
STATIC INLINE void Chip_UART_SetRS485Flags(LPC_USART_T *pUART, uint32_t ctrl)
{
	IP_UART_SetRS485Flags(pUART, ctrl);
}

/**
 * @brief	Clear RS485 control register options
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	ctrl	: Or'ed values to clear for RS485 control register
 * @return	Nothing
 * @note	Use an Or'ed value of UART_RS485CTRL_* definitions with this
 *			call to clear specific options.
 */
STATIC INLINE void Chip_UART_ClearRS485Flags(LPC_USART_T *pUART, uint32_t ctrl)
{
	IP_UART_ClearRS485Flags(pUART, ctrl);
}

/**
 * @brief	Set RS485 address match value
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	addr	: Address match value for RS-485/EIA-485 mode
 * @return	Nothing
 */
STATIC INLINE void Chip_UART_SetRS485Addr(LPC_USART_T *pUART, uint8_t addr)
{
	IP_UART_SetRS485Addr(pUART, addr);
}

/**
 * @brief	Read RS485 address match value
 * @param	pUART	: Pointer to selected UART peripheral
 * @return	Address match value for RS-485/EIA-485 mode
 */
STATIC INLINE uint8_t Chip_UART_GetRS485Addr(LPC_USART_T *pUART)
{
	return IP_UART_GetRS485Addr(pUART);
}

/**
 * @brief	Set RS485 direction control (RTS or DTR) delay value
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	dly		: direction control (RTS or DTR) delay value
 * @return	Nothing
 * @note	This delay time is in periods of the baud clock. Any delay
 *			time from 0 to 255 bit times may be programmed.
 */
STATIC INLINE void Chip_UART_SetRS485Delay(LPC_USART_T *pUART, uint8_t dly)
{
	IP_UART_SetRS485Delay(pUART, dly);
}

/**
 * @brief	Read RS485 direction control (RTS or DTR) delay value
 * @param	pUART	: Pointer to selected UART peripheral
 * @return	direction control (RTS or DTR) delay value
 * @note	This delay time is in periods of the baud clock. Any delay
 *			time from 0 to 255 bit times may be programmed.
 */
STATIC INLINE uint8_t Chip_UART_GetRS485Delay(LPC_USART_T *pUART)
{
	return IP_UART_GetRS485Delay(pUART);
}

/**
 * @brief	Sets best dividers to get a target bit rate (without fractional divider)
 * @param	pUART		: Pointer to selected UART peripheral
 * @param	baudrate	: Target baud rate (baud rate = bit rate)
 * @return	The actual baud rate, or 0 if no rate can be found
 */
uint32_t Chip_UART_SetBaud(LPC_USART_T *pUART, uint32_t baudrate);

/**
 * @brief	Sets best dividers to get a target bit rate (with fractional divider)
 * @param	pUART		: Pointer to selected UART peripheral
 * @param	baudrate	: Target baud rate (baud rate = bit rate)
 * @return	The actual baud rate, or 0 if no rate can be found
 */
STATIC INLINE uint32_t Chip_UART_SetBaudFDR(LPC_USART_T *pUART, uint32_t baudrate)
{
	return IP_UART_SetBaud(pUART, baudrate, Chip_Clock_GetMainClockRate());
}

/** @defgroup PERIPH_11XX_UART_BLOCK CHIP: LPC11xx UART Driver blocking transfer functions
 * These functions provide data transmit and receive via the UART and
 * will block until the all the data has been sent or a specified amount
 * of data has been received.
 * @{
 */

/**
 * @brief	Transmit a byte array through the UART peripheral (blocking)
 * @param	pUART		: Pointer to selected UART peripheral
 * @param	data		: Pointer to data to transmit
 * @param	numBytes	: Number of bytes to transmit
 * @return	The number of bytes transmitted
 * @note	This function will send or place all bytes into the transmit
 *			FIFO. This function will block until the last bytes are in the FIFO.
 */
int Chip_UART_SendBlocking(LPC_USART_T *pUART, const void *data, int numBytes);

/**
 * @brief	Read data through the UART peripheral (blocking)
 * @param	pUART		: Pointer to selected UART peripheral
 * @param	data		: Pointer to data array to fill
 * @param	numBytes	: Size of the passed data array
 * @return	The size of the dat array
 * @note	This function reads data from the receive FIFO until the passed
 *			buffer is completely full. The function will block until full.
 *			This function ignores errors.
 */
int Chip_UART_ReadBlocking(LPC_USART_T *pUART, void *data, int numBytes);

/**
 * @}
 */

/** @defgroup PERIPH_11XX_UART_RB CHIP: LPC11xx UART Driver interrupt transfer functions
 * These functions provide data transmit/receive and support functions
 * for the UART for use with a ring buffer.
 * @{
 */

/**
 * @brief	UART receive-only interrupt handler for ring buffers
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	pRB		: Pointer to ring buffer structure to use
 * @return	Nothing
 * @note	If ring buffer support is desired for the receive side
 *			of data transfer, the UART interrupt should call this
 *			function for a receive based interrupt status.
 */
void Chip_UART_RXIntHandlerRB(LPC_USART_T *pUART, RINGBUFF_T *pRB);

/**
 * @brief	UART transmit-only interrupt handler for ring buffers
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	pRB		: Pointer to ring buffer structure to use
 * @return	Nothing
 * @note	If ring buffer support is desired for the transmit side
 *			of data transfer, the UART interrupt should call this
 *			function for a transmit based interrupt status.
 */
void Chip_UART_TXIntHandlerRB(LPC_USART_T *pUART, RINGBUFF_T *pRB);

/**
 * @brief	Populate a transmit ring buffer and start UART transmit
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	pRB		: Pointer to ring buffer structure to use
 * @param	data	: Pointer to buffer to move to ring buffer
 * @param	bytes	: Number of bytes to move
 * @return	The number of bytes placed into the ring buffer
 * @note	Will move the data into the TX ring buffer and start the
 *			transfer. If the number of bytes returned is less than the
 *			number of bytes to send, the ring buffer is considered full.
 */
uint32_t Chip_UART_SendRB(LPC_USART_T *pUART, RINGBUFF_T *pRB, const void *data, int bytes);

/**
 * @brief	Copy data from a receive ring buffer
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	pRB		: Pointer to ring buffer structure to use
 * @param	data	: Pointer to buffer to fill from ring buffer
 * @param	bytes	: Size of the passed buffer in bytes
 * @return	The number of bytes placed into the ring buffer
 * @note	Will move the data from the RX ring buffer up to the
 *			the maximum passed buffer size. Returns 0 if there is
 *			no data in the ring buffer.
 */
int Chip_UART_ReadRB(LPC_USART_T *pUART, RINGBUFF_T *pRB, void *data, int bytes);

/**
 * @brief	UART receive/transmit interrupt handler for ring buffers
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	pRXRB	: Pointer to transmit ring buffer
 * @param	pTXRB	: Pointer to receive ring buffer
 * @return	Nothing
 * @note	This provides a basic implementation of the UART IRQ
 *			handler for support of a ring buffer implementation for
 *			transmit and receive.
 */
void Chip_UART_IRQRBHandler(LPC_USART_T *pUART, RINGBUFF_T *pRXRB, RINGBUFF_T *pTXRB);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __UART_11XX_H_ */
