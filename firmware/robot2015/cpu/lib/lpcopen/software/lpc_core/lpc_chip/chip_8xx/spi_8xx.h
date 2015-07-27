/*
 * @brief LPC8xx SPI inclusion file
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

#ifndef __SPI_8XX_H_
#define __SPI_8XX_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup SPI_8XX CHIP: LPC8xx SPI driver
 * @ingroup CHIP_8XX_Drivers
 * This driver provides SPI support for the device.
 * The driver requires the following IP drivers:<br>
 * @ref IP_SPI_002<br>
 * @{
 */

typedef IP_SPI_CONFIG_T          SPI_CONFIG_T;
typedef IP_SPI_DELAY_CONFIG_T    SPI_DELAY_CONFIG_T;

/**
 * @brief SPI data setup structure
 */
typedef struct {
	uint16_t  *pTx;	/**< Pointer to data buffer*/
	uint32_t  TxCnt;/* Transmit Counter */
	uint16_t  *pRx;	/**< Pointer to data buffer*/
	uint32_t  RxCnt;/* Transmit Counter */
	uint32_t  Length;	/**< Data Length*/
	uint16_t  DataSize;	/** < The size of a frame (1-16)*/
} CHIP_SPI_DATA_SETUP_T;

/**
 * @brief   Initialize the SPI
 * @param	pSPI			: The base SPI peripheral on the chip
 * @param	pConfig			: SPI Configuration
 * @return	Nothing
 */
void Chip_SPI_Init(LPC_SPI_T *pSPI, SPI_CONFIG_T *pConfig);

/**
 * @brief	Calculate the divider for SPI clock
 * @param	pSPI		: The base of SPI peripheral on the chip
 * @param	bitRate	: Expected clock rate
 * @return	Divider value
 */
uint32_t Chip_SPI_CalClkRateDivider(LPC_SPI_T *pSPI, uint32_t bitRate);

/**
 * @brief	Config SPI Delay parameters
 * @param	pSPI	: The base of SPI peripheral on the chip
 * @param	pConfig	: SPI Delay Configure Struct
 * @return	 Nothing
 * @note	The SPI controller is disabled
 */
STATIC INLINE void Chip_SPI_DelayConfig(LPC_SPI_T *pSPI, SPI_DELAY_CONFIG_T *pConfig)
{
	IP_SPI_DelayConfig(pSPI, pConfig);
}

/**
 * @brief	Disable SPI operation
 * @param	pSPI	: The base of SPI peripheral on the chip
 * @return	Nothing
 * @note	The SPI controller is disabled
 */
void Chip_SPI_DeInit(LPC_SPI_T *pSPI);

/**
 * @brief   Enable/Disable SPI interrupt
 * @param	pSPI			: The base SPI peripheral on the chip
 * @param	IntMask		: Interrupt mask
 * @param	NewState		: ENABLE or DISABLE interrupt
 * @return	Nothing
 */
void Chip_SPI_Int_Cmd(LPC_SPI_T *pSPI, uint32_t IntMask, FunctionalState NewState);

/**
 * @brief	Enable SPI peripheral
 * @param	pSPI	: The base of SPI peripheral on the chip
 * @return	Nothing
 */
STATIC INLINE void Chip_SPI_Enable(IP_SPI_002_T *pSPI)
{
	IP_SPI_Enable(pSPI);
}

/**
 * @brief	Disable SPI peripheral
 * @param	pSPI	: The base of SPI peripheral on the chip
 * @return	Nothing
 */
STATIC INLINE void Chip_SPI_Disable(IP_SPI_002_T *pSPI)
{
	IP_SPI_Disable(pSPI);
}

/**
 * @brief	Enable loopback mode
 * @param	pSPI		: The base of SPI peripheral on the chip
 * @return	Nothing
 * @note	Serial input is taken from the serial output (MOSI or MISO) rather
 * than the serial input pin
 */
STATIC INLINE void Chip_SPI_EnableLoopBack(LPC_SPI_T *pSPI)
{
	IP_SPI_EnableLoopBack(pSPI);
}

/**
 * @brief	Disable loopback mode
 * @param	pSPI		: The base of SPI peripheral on the chip
 * @return	Nothing
 * @note	Serial input is taken from the serial output (MOSI or MISO) rather
 * than the serial input pin
 */
STATIC INLINE void Chip_SPI_DisableLoopBack(LPC_SPI_T *pSPI)
{
	IP_SPI_DisableLoopBack(pSPI);
}

/**
 * @brief	Get the current status of SPI controller
 * @param	pSPI	: The base of SPI peripheral on the chip
 * @return	SPI Status (Or-ed bit value of SPI_STAT_*)
 */
STATIC INLINE uint32_t Chip_SPI_GetStatus(LPC_SPI_T *pSPI)
{
	return IP_SPI_GetStatus(pSPI);
}

/**
 * @brief	 Send the first Frame of a transfer (Rx Ignore)
 * @param	pSPI		: The base of SPI peripheral on the chip
 * @param	Data	:  Transmit data
 * @param	DataSize	:  Data Size (1-16)
 * @return	Nothing
 */
STATIC INLINE void Chip_SPI_SendFirstFrame_RxIgnore(LPC_SPI_T *pSPI, uint16_t Data, uint8_t DataSize)
{
	IP_SPI_SendFrameAndControl(pSPI,
							   Data,
							   DataSize,
							   SPI_TXDATCTL_ASSERT_SSEL | SPI_TXDATCTL_EOF | SPI_TXDATCTL_RXIGNORE);
}

/**
 * @brief	 Send the first Frame of a transfer
 * @param	pSPI		: The base of SPI peripheral on the chip
 * @param	Data	:  Transmit data
 * @param	DataSize	:  Data Size (1-16)
 * @return	Nothing
 */
STATIC INLINE void Chip_SPI_SendFirstFrame(LPC_SPI_T *pSPI, uint16_t Data, uint8_t DataSize)
{
	IP_SPI_SendFrameAndControl(pSPI, Data, DataSize, SPI_TXDATCTL_ASSERT_SSEL | SPI_TXDATCTL_EOF);
}

/**
 * @brief	 Send the middle Frame of a transfer
 * @param	pSPI		: The base of SPI peripheral on the chip
 * @param	Data	:  Transmit data
 * @return	Nothing
 */
STATIC INLINE void Chip_SPI_SendMidFrame(LPC_SPI_T *pSPI, uint16_t Data)
{
	IP_SPI_SendFrame(pSPI, Data);
}

/**
 * @brief	 Send the last Frame of a transfer (Rx Ignore)
 * @param	pSPI		: The base of SPI peripheral on the chip
 * @param	Data	:  Transmit data
 * @param	DataSize	:  Data Size (1-16)
 * @return	Nothing
 */
STATIC INLINE void Chip_SPI_SendLastFrame_RxIgnore(LPC_SPI_T *pSPI, uint16_t Data, uint8_t DataSize)
{
	IP_SPI_SendFrameAndControl(pSPI,
							   Data,
							   DataSize,
							   SPI_TXDATCTL_ASSERT_SSEL | SPI_TXDATCTL_EOF | SPI_TXDATCTL_EOT | SPI_TXDATCTL_RXIGNORE);
}

/**
 * @brief	 Send the last Frame of a transfer
 * @param	pSPI		: The base of SPI peripheral on the chip
 * @param	Data	:  Transmit data
 * @param	DataSize	:  Data Size (1-16)
 * @return	Nothing
 */
STATIC INLINE void Chip_SPI_SendLastFrame(LPC_SPI_T *pSPI, uint16_t Data, uint8_t DataSize)
{
	IP_SPI_SendFrameAndControl(pSPI, Data, DataSize, SPI_TXDATCTL_ASSERT_SSEL | SPI_TXDATCTL_EOF | SPI_TXDATCTL_EOT);
}

/**
 * @brief	 Read data received
 * @param	pSPI		: The base of SPI peripheral on the chip
 * @return	Receive data
 */
STATIC INLINE uint16_t Chip_SPI_ReceiveFrame(LPC_SPI_T *pSPI)
{
	return IP_SPI_ReceiveFrame(pSPI);
}

/**
 * @brief   SPI Interrupt Read/Write
 * @param	pSPI			: The base SPI peripheral on the chip
 * @param	xf_setup		: Pointer to a SPI_DATA_SETUP_T structure that contains specified
 *                          information about transmit/receive data	configuration
 * @return	SUCCESS or ERROR
 */
Status Chip_SPI_Int_RWFrames(LPC_SPI_T *pSPI, CHIP_SPI_DATA_SETUP_T *xf_setup);

/**
 * @brief   SPI Polling Read/Write in blocking mode
 * @param	pSPI			: The base SPI peripheral on the chip
 * @param	pXfSetup		: Pointer to a SPI_DATA_SETUP_T structure that contains specified
 *                          information about transmit/receive data	configuration
 * @return	Actual data length has been transferred
 * @note
 * This function can be used in both master and slave mode. It starts with writing phase and after that,
 * a reading phase is generated to read any data available in RX_FIFO. All needed information is prepared
 * through xf_setup param.
 */
uint32_t Chip_SPI_RWFrames_Blocking(LPC_SPI_T *pSPI, CHIP_SPI_DATA_SETUP_T *pXfSetup);

/**
 * @brief   SPI Polling Write in blocking mode
 * @param	pSPI			: The base SPI peripheral on the chip
 * @param	pXfSetup			:Pointer to a SPI_DATA_SETUP_T structure that contains specified
 *                          information about transmit/receive data	configuration
 * @return	Actual data length has been transferred
 * @note
 * This function can be used in both master and slave mode. First, a writing operation will send
 * the needed data. After that, a dummy reading operation is generated to clear data buffer
 */
uint32_t Chip_SPI_WriteFrames_Blocking(LPC_SPI_T *pSPI, CHIP_SPI_DATA_SETUP_T *pXfSetup);

/**
 * @brief   SPI Polling Read in blocking mode
 * @param	pSPI			: The base SPI peripheral on the chip
 * @param	pXfSetup			:Pointer to a SPI_DATA_SETUP_T structure that contains specified
 *                          information about transmit/receive data	configuration
 * @return	Actual data length has been read
 * @note
 * This function can be used in both master and slave mode. First, a writing operation will send
 * the needed data. After that, a dummy reading operation is generated to clear data buffer
 */
uint32_t Chip_SPI_ReadFrames_Blocking(LPC_SPI_T *pSPI, CHIP_SPI_DATA_SETUP_T *pXfSetup);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_8XX_H_ */
