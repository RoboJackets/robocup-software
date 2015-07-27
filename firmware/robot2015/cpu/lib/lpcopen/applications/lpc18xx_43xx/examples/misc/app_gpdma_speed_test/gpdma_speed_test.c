/*
 * @brief GPDMA Speed test example
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

#include <stdlib.h>
#include <string.h>
#include "board.h"

/** @defgroup EXAMPLES_MISC_18XX43XX_GPDMA_SPEED_TEST LPC18xx/43xx GPDMA Speed test example
 * @ingroup EXAMPLES_MISC_18XX43XX
 * <b>Example description</b><br>
 * This example benchmarks the data transfer speed of gpdma against cpu based transfer
 * function memcpy.<br>
 *
 * UART needs to be setup prior to running the example as the example produces the output
 * to the UART console.<br>
 *
 * <b>Special connection requirements</b><br>
 * There are no special connection requirements for this example.<br>
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
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Data transfer information */
#define TRANSFER_DST_ADDR          0x28500000
#define TRANSFER_SRC_ADDR          0x28100000
#define TRANSFER_SIZE              0x200000

#define TRANSFER_BLOCK_SZ          (4 * 3 * 1024) /* 3K of data transfered per LLI */
#define DMA_DESCRIPTOR_COUNT       256

static uint8_t ch_no;
static DMA_TransferDescriptor_t desc_array[DMA_DESCRIPTOR_COUNT];
static volatile int dma_xfer_complete;
/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/
static void prepare_src_data(uint32_t *src, int sz)
{
	int i;
	for (i = 0; i < sz; i++) {
		src[i] = i | (~i << (16 - (i & 15))); /* Fill it with some pattern */
	}
}

/* Function to prepare memory to memory transfer descriptors */
static int prepare_dma_desc(uint32_t* dst, const uint32_t* src, uint32_t sz)
{
	int i;
	DMA_TransferDescriptor_t *desc;
	int num_desc = sz / TRANSFER_BLOCK_SZ;

	/* Add one more if size is not in 3K blocks */
	num_desc += sz * TRANSFER_BLOCK_SZ != sz;
	desc = &desc_array[0];
	if (num_desc >= DMA_DESCRIPTOR_COUNT) {
		DEBUGOUT("Transfer needs %d descriptors, but has "
			"%d descriptors only\r\n", num_desc, DMA_DESCRIPTOR_COUNT);
		return 0;
	}

	for (i = 0; i < num_desc; i ++)
	{
		Chip_DMA_PrepareDescriptor(LPC_GPDMA, &desc[i],
			(uint32_t) src + (i * TRANSFER_BLOCK_SZ),
			(uint32_t) dst + (i * TRANSFER_BLOCK_SZ),
			(uint32_t) (sz < TRANSFER_BLOCK_SZ ? sz : TRANSFER_BLOCK_SZ),
			GPDMA_TRANSFERTYPE_M2M_CONTROLLER_DMA,
			(i + 1 == num_desc) ? NULL: &desc[i + 1]);
		sz -= TRANSFER_BLOCK_SZ;
	}
	return num_desc;
}

/* Print the result of a data transfer */
static void print_result(const void *dst, const void *src, int sz, uint32_t stime, uint32_t etime, const char *mode)
{
	uint32_t clk = SystemCoreClock/1000000;
	int invalid;
	invalid = memcmp(dst, src, sz);
	DEBUGOUT("\r\nData transfer results [MODE: %s]\r\n"
		"=========================================\r\n", mode);
	DEBUGOUT("SOURCE  ADDR: 0x%08X\r\n", src);
	DEBUGOUT("DESTN   ADDR: 0x%08X\r\n", dst);
	DEBUGOUT("XFER    SIZE: %lu (Bytes)\r\n", sz);
	DEBUGOUT("CPU    SPEED: %lu.%lu (MHz)\r\n", clk, (SystemCoreClock / 10000) - (clk * 100));
	DEBUGOUT("START   TIME: %lu (ticks)\r\n", stime);
	DEBUGOUT("END     TIME: %lu (ticks)\r\n", etime);
	DEBUGOUT("TIME   TAKEN: %lu (uSec(s))\r\n", (etime - stime) / clk);
	DEBUGOUT("SRC/DST COMP: %s\r\n", invalid ? "NOT MATCHING" : "MATCHING");
}
/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	RIT interrupt handler
 * @return	Nothing
 */
void DMA_IRQHandler(void)
{
	dma_xfer_complete = 1;
	Chip_DMA_Interrupt(LPC_GPDMA, ch_no);
}

/**
 * @brief	Main entry point
 * @return	Nothing
 */
int main(void)
{
	uint32_t start_time;
	uint32_t end_time;
	uint32_t *dst = (uint32_t *) TRANSFER_DST_ADDR;
	uint32_t *src = (uint32_t *) TRANSFER_SRC_ADDR;

	Board_Init();

	/* Initialize the DMA */
	Chip_GPDMA_Init(LPC_GPDMA);
	
	/* Prepare the source buffer for transfer */
	prepare_src_data(src, TRANSFER_SIZE/sizeof(*src));
	DEBUGSTR("***** DATA TRANSFER TEST *******\r\n");
	start_time = Chip_RIT_GetCounter(LPC_RITIMER);
	memcpy(dst, src, TRANSFER_SIZE);
	end_time = Chip_RIT_GetCounter(LPC_RITIMER);
	print_result(dst, src, TRANSFER_SIZE, start_time, end_time, "CPU (memcpy)");
	memset(dst, 0, TRANSFER_SIZE);
	ch_no = Chip_DMA_GetFreeChannel(LPC_GPDMA, 0);
	if (prepare_dma_desc(dst, src, TRANSFER_SIZE) <= 0) {
		DEBUGSTR("Unable to create DMA Descriptors\r\n");
		while (1) {}
	}
	start_time = Chip_RIT_GetCounter(LPC_RITIMER);
	Chip_DMA_SGTransfer(LPC_GPDMA, ch_no, &desc_array[0], GPDMA_TRANSFERTYPE_M2M_CONTROLLER_DMA);
	NVIC_EnableIRQ(DMA_IRQn);
	while(!dma_xfer_complete); /* Set by ISR */
	end_time = Chip_RIT_GetCounter(LPC_RITIMER);
	print_result(dst, src, TRANSFER_SIZE, start_time, end_time, "DMA");
	/* LED is toggled in interrupt handler */
	while (1) {}
}

/**
 * @}
 */
