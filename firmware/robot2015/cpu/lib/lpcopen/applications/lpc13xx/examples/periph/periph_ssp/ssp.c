/*
 * @brief SSP example
 * This example show how to use the SSP in 2 modes : Polling, Interrupt
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

#include "board.h"

/** @defgroup EXAMPLES_PERIPH_13XX_SSP LPC13xx SSP/SPI example
 * @ingroup EXAMPLES_PERIPH_13XX
 * <b>Example description</b><br>
 * This example describes how to use SPI in POLLING or INTERRUPT  mode.<br>
 * The LOOPBACK_TEST macro is used to enable Loop-back mode of SPI peripheral. It allows
 * a simple software testing. The transmit and receive data is connected together. No
 * more connection is required in this case.
 * If LOOPBACK_TEST macro is disabled, it is needed to connect 2 hardware boards,
 * one for Master and one for Slave. Use SSP_MODE_TEST macro to select the SPI mode.<br>
 *      SPI configuration:<br>
 *          - CPHA = 0: Data is sampled on the first clock edge of SCK.<br>
 *          - CPOL = 0: The rest state of the clock (between frames) is low.<br>
 *          - Sample rate = 100KHz.<br>
 *          - DSS = 8: 8 bits per transfer.<br>
 *      After initialize transmit buffer, SPI master/slave will transfer a number of bytes
 *      to SPI slave/master and receive data concurrently.<br>
 *      When transferring completed, receive and transmit buffer will be compared. If they match,
 *      the led is ON. If not the led is OFF.<br>
 *      This example supports 2 transfer modes: POLLING mode and INTERRUPT mode.<br>
 *
 * <b>Special connection requirements</b><br>
 * Connects SPI pins of the master boards to the slave boards.<br>
 *  - LPCXpresso LPC1347:
 *		- P1_29: SPI_SCK<br>
 *		- P0.2: SPI_SSEL<br>
 *		- P0.8: SPI_MISO<br>
 *		- P0.9: SPI_MOSI<br>
 *  - LPCXpresso LPC1343:
 *		- P2_11: SPI_SCK<br>
 *		- P0.2: SPI_SSEL<br>
 *		- P0.8: SPI_MISO<br>
 *		- P0.9: SPI_MOSI<br>
 *		The SCK pin of master board is connected to SCK of slave board.<br>
 *		The SSEL pin of master board is connected to SSEL of slave board.<br>
 *		The MISO pin of master board is connected to MISO of slave board.<br>
 *		The MOSI pin of master board is connected to MOSI of slave board.<br>
 *      Common ground must be connected together between two boards.<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_13XX_BUILDPROCS_XPRESSO<br>
 * @ref LPCOPEN_13XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_13XX_BUILDPROCS_IAR<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_13XX_BOARD_XPRESSO_1347<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define LOOPBACK_TEST       1
#define SSP_MODE_TEST       1	/*1: Master, 0: Slave */
#define POLLING_MODE        0
#if POLLING_MODE
#define INTERRUPT_MODE      0
#else
#define INTERRUPT_MODE      1
#endif
#define BUFFER_SIZE                         (0x100)
#define SSP_DATA_BITS                       (SSP_BITS_8)
#define SSP_DATA_BIT_NUM(databits)          (databits + 1)
#define SSP_DATA_BYTES(databits)            (((databits) > SSP_BITS_8) ? 2 : 1)
#define SSP_LO_BYTE_MSK(databits)           ((SSP_DATA_BYTES(databits) > 1) ? 0xFF : (0xFF >> \
																					  (8 - SSP_DATA_BIT_NUM(databits))))
#define SSP_HI_BYTE_MSK(databits)           ((SSP_DATA_BYTES(databits) > 1) ? (0xFF >> \
																			   (16 - SSP_DATA_BIT_NUM(databits))) : 0)
#define LPC_SSP           LPC_SSP0
#define SSP_IRQ           SSP0_IRQn
#define SSPIRQHANDLER     SSP0_IRQHandler

/* Tx buffer */
static uint8_t Tx_Buf[BUFFER_SIZE];

/* Rx buffer */
static uint8_t Rx_Buf[BUFFER_SIZE];

static SSP_ConfigFormat ssp_format;
static Chip_SSP_DATA_SETUP_T xf_setup;
static volatile uint8_t  isXferCompleted = 0;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Initialize buffer */
static void Buffer_Init(void)
{
	uint16_t i;
	uint8_t ch = 0;

	for (i = 0; i < BUFFER_SIZE; i++) {
		Tx_Buf[i] = ch++;
		Rx_Buf[i] = 0xAA;
	}
}

/* Verify buffer after transfer */
static uint8_t Buffer_Verify(void)
{
	uint16_t i;
	uint8_t *src_addr = (uint8_t *) &Tx_Buf[0];
	uint8_t *dest_addr = (uint8_t *) &Rx_Buf[0];

	for ( i = 0; i < BUFFER_SIZE; i++ ) {

		if (((*src_addr) & SSP_LO_BYTE_MSK(ssp_format.bits)) !=
			((*dest_addr) & SSP_LO_BYTE_MSK(ssp_format.bits))) {
			return 1;
		}
		src_addr++;
		dest_addr++;

		if (SSP_DATA_BYTES(ssp_format.bits) == 2) {
			if (((*src_addr) & SSP_HI_BYTE_MSK(ssp_format.bits)) !=
				((*dest_addr) & SSP_HI_BYTE_MSK(ssp_format.bits))) {
				return 1;
			}
			src_addr++;
			dest_addr++;
			i++;
		}
	}
	return 0;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/
#if INTERRUPT_MODE
/**
 * @brief	SSP interrupt handler sub-routine
 * @return	Nothing
 */
void SSPIRQHANDLER(void)
{
	Chip_SSP_Int_Disable(LPC_SSP);	/* Disable all interrupt */
	if (SSP_DATA_BYTES(ssp_format.bits) == 1) {
		Chip_SSP_Int_RWFrames8Bits(LPC_SSP, &xf_setup);
	}
	else {
		Chip_SSP_Int_RWFrames16Bits(LPC_SSP, &xf_setup);
	}

	if ((xf_setup.rx_cnt != xf_setup.length) || (xf_setup.tx_cnt != xf_setup.length)) {
		Chip_SSP_Int_Enable(LPC_SSP);	/* enable all interrupts */
	}
	else {
		isXferCompleted = 1;
	}
}

#endif /*INTERRUPT_MODE*/

/**
 * @brief	Main routine for SSP example
 * @return	Nothing
 */
int main(void)
{
	Board_Init();

	/* SSP initialization */
	Board_SSP_Init(LPC_SSP);
	Board_LED_Set(0, false);
	Chip_SSP_Init(LPC_SSP);

	ssp_format.frameFormat = SSP_FRAMEFORMAT_SPI;
	ssp_format.bits = SSP_DATA_BITS;
	ssp_format.clockMode = SSP_CLOCK_MODE0;
	Chip_SSP_SetFormat(LPC_SSP, &ssp_format);
	Chip_SSP_SetMaster(LPC_SSP, SSP_MODE_TEST);
	Chip_SSP_Enable(LPC_SSP);

#if INTERRUPT_MODE
	/* Setting SSP interrupt */
	NVIC_EnableIRQ(SSP_IRQ);
#endif

#if LOOPBACK_TEST
	Chip_SSP_EnableLoopBack(LPC_SSP);
#endif
	Buffer_Init();
	xf_setup.length = BUFFER_SIZE;
	xf_setup.tx_data = Tx_Buf;
	xf_setup.rx_data = Rx_Buf;
	xf_setup.rx_cnt = xf_setup.tx_cnt = 0;

#if POLLING_MODE
	Chip_SSP_RWFrames_Blocking(LPC_SSP, &xf_setup);
#elif INTERRUPT_MODE
	Chip_SSP_Int_FlushData(LPC_SSP);/* flush dummy data from SSP FiFO */
	if (SSP_DATA_BYTES(ssp_format.bits) == 1) {
		Chip_SSP_Int_RWFrames8Bits(LPC_SSP, &xf_setup);
	}
	else {
		Chip_SSP_Int_RWFrames16Bits(LPC_SSP, &xf_setup);
	}

	Chip_SSP_Int_Enable(LPC_SSP);			/* enable interrupt */
	while (!isXferCompleted) {}
#endif /*INTERRUPT_MODE*/

	if (Buffer_Verify()) {
		Board_LED_Set(0, false);	/* Error */
	}
	else {
		Board_LED_Set(0, true);	/* Success */
	}

#if LOOPBACK_TEST
	Chip_SSP_DisableLoopBack(LPC_SSP);
#endif
	/* DeInitialize SPI peripheral */
	Chip_SSP_DeInit(LPC_SSP);

	while (1) {
		__WFI();
	}
}

/**
 * @}
 */
