/*
 * @brief SSP example
 * This example show how to use the SSP in 3 modes : Polling, Interrupt and DMA
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
#include "stdio.h"

/** @defgroup EXAMPLES_PERIPH_18XX43XX_SSP LPC18xx/43xx SSP/SPI example
 * @ingroup EXAMPLES_PERIPH_18XX43XX
 * <b>Example description</b><br>
 * This example describes how to use SSP in POLLING, INTERRUPT or DMA mode.
 * It is needed to connect 2 hardware boards, one for Master and one for Slave.<br>
 *
 *      SSP configuration:<br>
 *          - CPHA = 0: data is sampled on the first clock edge of SCK.<br>
 *          - CPOL = 0: SCK is active high.<br>
 *          - Sample rate = 400kHz.<br>
 *          - DSS = 8: 8 bits per transfer.<br>
 *          - FRF= 0: SPI Frame format.<br>
 *      After initialize transmit buffer, SPI master/slave will transfer a number of bytes 
 *      to SPI slave/master and receive data concurrently.<br>
 *      After a transfer completed, receive and transmit buffer will be compared and
 *      the result will be print out via UART port.<br>
 *      This example supports 3 transfer modes: POLLING mode, INTERRUPT mode and DMA mode.<br>
 *
 *  - Connect UART port on the master board and slave board to COM ports on your PC.<br>
 *  - Configure terminal program on the PC per the above Serial display configuration<br>
 *  - Configure hardware, connect master board and slave board as below<br>
 *  - Build and run the example. Following guidance on terminals of master and slave to do test.<br>
 *
 * <b>Special connection requirements</b><br>
 *  - Hitex A4 LPC1850 and LPC4350 boards<br>
 *    - PF_0 : X16 pin 43 - SCK0 master(slave) - connect to SCK slave(master) board <br>
 *    - PF_1 : X16 pin 46 - SEL0 master(slave) - connect to SEL slave(master) board<br>
 *    - PF_2 : X16 pin 45 - MISO0 master(slave) - connect to MISO slave(master) board<br>
 *    - PF_3 : X16 pin 44 - MOSI0 master(slave) - connect to MOSI slave(master) board<br>
 *  - Keil MCB1857 and MCB4357 boards<br>
 *    - PF_4 - SCK1  master(slave) - connect to SCK slave(master) board<br>
 *    - PF_5 - SSEL1  master(slave) - connect to SEL slave(master) board<br>
 *    - PF_6 - MISO1  master(slave) - connect to MISO slave(master) board<br>
 *    - PF_7 - MOSI1  master(slave) - connect to MOSI slave(master) board<br>
 *  - NGX Xplorer 1830 and 4330 boards<br>
 *    - CLK0 : J8 pin 7 - SCK1 master(slave) - connect to SCK slave(master) board <br>
 *    - P1_2 : J8 pin 8 - SSEL1 master(slave) - connect to SEL slave(master) board<br>
 *    - P1_3 : J8 pin 6 - MISO1  master(slave) - connect to MISO slave(master) board<br>
 *    - P1_4 : J8 pin 5 - MOSI1  master(slave) - connect to MOSI slave(master) board<br>
 *
 * Notes : <br>
 *  - Common ground must be connected together between two boards.<br>
 *  - The application default mode is Master mode. Press '1' on main menu to change Slave mode<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_18XX43XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_18XX43XX_BUILDPROCS_IAR<br>
 * @ref LPCOPEN_18XX43XX_BUILDPROCS_XPRESSO<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_18XX_BOARD_HITEX1850<br>
 * @ref LPCOPEN_43XX_BOARD_HITEX4350<br>
 * @ref LPCOPEN_18XX_BOARD_KEIL1857<br>
 * @ref LPCOPEN_43XX_BOARD_KEIL4357<br>
 * @ref LPCOPEN_18XX_BOARD_NGX1830<br>
 * @ref LPCOPEN_43XX_BOARD_NGX4330<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#if defined(BOARD_KEIL_MCB_18574357) || defined(BOARD_NGX_XPLORER_18304330)
#define LPC_SSP           LPC_SSP1
#define SSP_IRQ           SSP1_IRQn
#define LPC_GPDMA_SSP_TX  GPDMA_CONN_SSP1_Tx
#define LPC_GPDMA_SSP_RX  GPDMA_CONN_SSP1_Rx
#elif defined(BOARD_HITEX_EVA_18504350)
#define LPC_SSP           LPC_SSP0
#define SSP_IRQ           SSP0_IRQn
#define LPC_GPDMA_SSP_TX  GPDMA_CONN_SSP0_Tx
#define LPC_GPDMA_SSP_RX  GPDMA_CONN_SSP0_Rx
#else
#warning Unsupported Board
#endif
#define BUFFER_SIZE                         (0x100)
#define SSP_DATA_BITS                       (SSP_BITS_8)
#define SSP_DATA_BIT_NUM(databits)          (databits+1)
#define SSP_DATA_BYTES(databits)            (((databits) > SSP_BITS_8) ? 2:1)
#define SSP_LO_BYTE_MSK(databits)           ((SSP_DATA_BYTES(databits) > 1) ? 0xFF:(0xFF>>(8-SSP_DATA_BIT_NUM(databits))))
#define SSP_HI_BYTE_MSK(databits)           ((SSP_DATA_BYTES(databits) > 1) ? (0xFF>>(16-SSP_DATA_BIT_NUM(databits))):0)

#define SSP_MODE_SEL                        (0x31)
#define SSP_TRANSFER_MODE_SEL               (0x32)
#define SSP_MASTER_MODE_SEL                 (0x31)
#define SSP_SLAVE_MODE_SEL                  (0x32)
#define SSP_POLLING_SEL                     (0x31)
#define SSP_INTERRUPT_SEL                   (0x32)
#define SSP_DMA_SEL                         (0x33)

/* Tx buffer */
static uint8_t Tx_Buf[BUFFER_SIZE];

/* Rx buffer */
static uint8_t Rx_Buf[BUFFER_SIZE];

static SSP_ConfigFormat ssp_format;
static Chip_SSP_DATA_SETUP_T xf_setup;
static volatile uint8_t  isXferCompleted = 0;
static uint8_t dmaChSSPTx, dmaChSSPRx;
static volatile uint8_t isDmaTxfCompleted = 0;
static volatile uint8_t isDmaRxfCompleted = 0;

#if defined(DEBUG_ENABLE)
static char sspWaitingMenu[] = "SSP Polling: waiting for transfer ...\n\r";
static char sspIntWaitingMenu[]  = "SSP Interrupt: waiting for transfer ...\n\r";
static char sspDMAWaitingMenu[]  = "SSP DMA: waiting for transfer ...\n\r";

static char sspPassedMenu[] = "SSP: Transfer PASSED\n\r";
static char sspFailedMenu[] = "SSP: Transfer FAILED\n\r";

static char sspTransferModeSel[] = "\n\rPress 1-3 or 'q' to exit\n\r"
								   "\t 1: SSP Polling Read Write\n\r"
								   "\t 2: SSP Int Read Write\n\r"
								   "\t 3: SSP DMA Read Write\n\r";

static char helloMenu[] = "Hello NXP Semiconductors \n\r";
static char sspMenu[] = "SSP demo \n\r";
static char sspMainMenu[] = "\t 1: Select SSP Mode (Master/Slave)\n\r"
							"\t 2: Select Transfer Mode\n\r";
static char sspSelectModeMenu[] = "\n\rPress 1-2 to select or 'q' to exit:\n\r"
								  "\t 1: Master \n\r"
								  "\t 2: Slave\n\r";
#endif /* defined(DEBUG_ENABLE) */
#if defined(BOARD_KEIL_MCB_18574357) || defined(BOARD_NGX_XPLORER_18304330)
#define SSPIRQHANDLER SSP1_IRQHandler
#elif defined(BOARD_HITEX_EVA_18504350)
#define SSPIRQHANDLER SSP0_IRQHandler
#endif

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

/* Select the Transfer mode : Polling, Interrupt or DMA */
static void appSSPTest(void)
{
	int key;

	DEBUGOUT(sspTransferModeSel);

	dmaChSSPTx = Chip_DMA_GetFreeChannel(LPC_GPDMA, LPC_GPDMA_SSP_TX);
	dmaChSSPRx = Chip_DMA_GetFreeChannel(LPC_GPDMA, LPC_GPDMA_SSP_RX);

	xf_setup.length = BUFFER_SIZE;
	xf_setup.tx_data = Tx_Buf;
	xf_setup.rx_data = Rx_Buf;

	while (1) {
		key = 0xFF;
		do {
			key = DEBUGIN();
		} while ((key & 0xFF) == 0xFF);

		Buffer_Init();

		switch (key) {
		case SSP_POLLING_SEL:	/* SSP Polling Read Write Mode */
			DEBUGOUT(sspWaitingMenu);
			xf_setup.rx_cnt = xf_setup.tx_cnt = 0;

			Chip_SSP_RWFrames_Blocking(LPC_SSP, &xf_setup);

			if (Buffer_Verify() == 0) {
				DEBUGOUT(sspPassedMenu);
			}
			else {
				DEBUGOUT(sspFailedMenu);
			}
			break;

		case SSP_INTERRUPT_SEL:
			DEBUGOUT(sspIntWaitingMenu);

			isXferCompleted = 0;
			xf_setup.rx_cnt = xf_setup.tx_cnt = 0;

			Chip_SSP_Int_FlushData(LPC_SSP);/* flush dummy data from SSP FiFO */
			if (SSP_DATA_BYTES(ssp_format.bits) == 1) {
				Chip_SSP_Int_RWFrames8Bits(LPC_SSP, &xf_setup);
			}
			else {
				Chip_SSP_Int_RWFrames16Bits(LPC_SSP, &xf_setup);
			}

			Chip_SSP_Int_Enable(LPC_SSP);	/* enable interrupt */
			while (!isXferCompleted) {}

			if (Buffer_Verify() == 0) {
				DEBUGOUT(sspPassedMenu);
			}
			else {
				DEBUGOUT(sspFailedMenu);
			}
			break;

		case SSP_DMA_SEL:	/* SSP DMA Read and Write: fixed on 8bits */
			DEBUGOUT(sspDMAWaitingMenu);
			isDmaTxfCompleted = isDmaRxfCompleted = 0;

			Chip_SSP_DMA_Enable(LPC_SSP);
			/* data Tx_Buf --> SSP */
			Chip_DMA_Transfer(LPC_GPDMA, dmaChSSPTx,
							  (uint32_t) &Tx_Buf[0],
							  LPC_GPDMA_SSP_TX,
							  GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA,
							  BUFFER_SIZE);
			/* data SSP --> Rx_Buf */
			Chip_DMA_Transfer(LPC_GPDMA, dmaChSSPRx,
							  LPC_GPDMA_SSP_RX,
							  (uint32_t) &Rx_Buf[0],
							  GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
							  BUFFER_SIZE);

			while (!isDmaTxfCompleted || !isDmaRxfCompleted) {}
			if (Buffer_Verify() == 0) {
				DEBUGOUT(sspPassedMenu);
			}
			else {
				DEBUGOUT(sspFailedMenu);
			}
			Chip_SSP_DMA_Disable(LPC_SSP);
			break;

		case 'q':
		case 'Q':
			Chip_DMA_Stop(LPC_GPDMA, dmaChSSPTx);
			Chip_DMA_Stop(LPC_GPDMA, dmaChSSPRx);
			return;

		default:
			break;
		}

		DEBUGOUT(sspTransferModeSel);
	}

}

/* Select the SSP mode : Master or Slave */
static void appSSPSelectModeMenu(void)
{
	int key;

	DEBUGOUT(sspSelectModeMenu);

	while (1) {
		key = 0xFF;
		do {
			key = DEBUGIN();
		} while ((key & 0xFF) == 0xFF);

		switch (key) {
		case SSP_MASTER_MODE_SEL:	/* Master */
			Chip_SSP_SetMaster(LPC_SSP, 1);
			DEBUGOUT("Master Mode\n\r");
			return;

		case SSP_SLAVE_MODE_SEL:	/* Slave */
			Chip_SSP_SetMaster(LPC_SSP, 0);
			DEBUGOUT("Slave Mode\n\r");
			return;

		case 'q':
			return;

		default:
			break;
		}
		DEBUGOUT(sspSelectModeMenu);
	}

}

/* The main menu of the example. Allow user select the SSP mode (master or slave) and Transfer
   mode (Polling, Interrupt or DMA) */
static void appSSPMainMenu(void)
{
	int key;

	DEBUGOUT(helloMenu);
	DEBUGOUT(sspMenu);
	DEBUGOUT(sspMainMenu);

	while (1) {
		key = 0xFF;
		do {
			key = DEBUGIN();
		} while ((key & 0xFF) == 0xFF);

		switch (key) {
		case SSP_MODE_SEL:	/* Select SSP Mode */
			appSSPSelectModeMenu();
			break;

		case SSP_TRANSFER_MODE_SEL:	/* Select Transfer Mode */
			appSSPTest();
			break;

		default:
			break;
		}
		DEBUGOUT(sspMainMenu);
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

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

/**
 * @brief	DMA interrupt handler sub-routine. Set the waiting flag when transfer is successful
 * @return	Nothing
 */
void DMA_IRQHandler(void)
{
	if (Chip_DMA_Interrupt(LPC_GPDMA, dmaChSSPTx) == SUCCESS) {
		isDmaTxfCompleted = 1;
	}

	if (Chip_DMA_Interrupt(LPC_GPDMA, dmaChSSPRx) == SUCCESS) {
		isDmaRxfCompleted = 1;
	}
}

/**
 * @brief	Main routine for SSP example
 * @return	Nothing
 */
int main(void)
{
	Board_Init();

	/* SSP initialization */
	Board_SSP_Init(LPC_SSP);

	Chip_SSP_Init(LPC_SSP);

	ssp_format.frameFormat = SSP_FRAMEFORMAT_SPI;
	ssp_format.bits = SSP_DATA_BITS;
	ssp_format.clockMode = SSP_CLOCK_MODE0;
	Chip_SSP_SetFormat(LPC_SSP, &ssp_format);

	Chip_SSP_Enable(LPC_SSP);

	/* Initialize GPDMA controller */
	Chip_GPDMA_Init(LPC_GPDMA);

	/* Setting GPDMA interrupt */
	NVIC_DisableIRQ(DMA_IRQn);
	NVIC_SetPriority(DMA_IRQn, ((0x01 << 3) | 0x01));
	NVIC_EnableIRQ(DMA_IRQn);

	/* Setting SSP interrupt */
	NVIC_EnableIRQ(SSP_IRQ);

#if defined(BOARD_HITEX_EVA_18504350)
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, 0x6, 10, true);	/* SSEL_MUX_A */
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, 0x6, 11, true);	/* SSEL_MUX_B */
	Chip_GPIO_WritePortBit(LPC_GPIO_PORT, 0x6, 10, true);
	Chip_GPIO_WritePortBit(LPC_GPIO_PORT, 0x6, 11, false);
#endif

	appSSPMainMenu();
	return 0;
}

/**
 * @}
 */
