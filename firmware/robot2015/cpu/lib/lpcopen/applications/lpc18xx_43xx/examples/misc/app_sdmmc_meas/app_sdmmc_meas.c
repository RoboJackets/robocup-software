/*
 * @brief SD/MMC benchmark example
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

#include <string.h>
#include "board.h"
#include "chip.h"

/** @defgroup EXAMPLES_MISC_18XX43XX_SDMMC_MEAS LPC18xx/43xx SDMMC benchmark example
 * @ingroup EXAMPLES_MISC_18XX43XX
 * <b>Example description</b><br>
 * This example measures the SDMMC raw (without file system) I/O perfromance. The SDMMC
 * read & write performance will be measured. Please note that the code will be 
 * executed from IARM memory.<br>
 *
 * To use the example, plug a SD card (Hitex A4 board) or microSD card (NGX or Keil
 * boards) and connect a serial cable to the board's RS232/UART port start a terminal 
 * program to monitor the port.  The terminal program on the host PC should be setup 
 * for 115K8N1. <br>
 *
 * <b>Special connection requirements</b><br>
 *  - Hitex LPC1850EVA-A4-2 and LPC4350EVA-A4-2 boards<br>
 *     - Close pin 1-2 of JP9<br>
 *     - Open all SV3, SV6<br>
 *     - Close all SV12<br>
 *     - Uart1 terminal port: SV11-p2(U1_RXD) connect to SV1-p7, SV11-p4 (U1_TXD) connect to SV1-p5<br>
 *  - Keil MCB1857 and MCB4357 boards (debug output terminal on Uart3)<br>
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
 /* Compiler specific attributes */
#if defined(__IAR_SYSTEMS_ICC__)
	#define LOCATE_ATX(x)      _Pragma(#x)
	#define LOCATE_ATXX(x)     LOCATE_ATX(location=x)
	#define LOCATE_AT(x)       LOCATE_ATXX(x)
	#define WEAK_SYMBOL        __weak
#elif defined(__ARMCC_VERSION)
	#define LOCATE_AT(x)     __attribute__((at(x)))
	#define WEAK_SYMBOL      __attribute__((weak))
#elif (defined(__CODE_RED))
	#define LOCATE_ATX(x)     __attribute__((section(".shmem_"#x ",\"aw\",%nobits@")))
	#define LOCATE_AT(x) LOCATE_ATX(x)
	#define WEAK_SYMBOL      __attribute__((weak))
#else
	#error "Unsupported Compiler/Tool-Chain!"
#endif

#ifdef BOARD_HITEX_EVA_18504350
#define debugstr(str)  board_uart_out_string(str)
#else
#define debugstr(str)  DEBUGSTR(str)
#endif

/* Number of sectors to read/write */
#define NUM_SECTORS     2048

/* Number of iterations of measurement */
#define NUM_ITER        10

/* Starting sector number for read/write */
#define START_SECTOR    32

/* Buffer size (in bytes) for R/W operations */
#define BUFFER_SIZE     (NUM_SECTORS * MMC_SECTOR_SIZE)

/* Buffers to store original data of SD/MMC card.
 * The data will be stored in this buffer, once read/write measurement
 * completed, the original contents will be restored into SD/MMC card.
 * This is done in order avoid corurupting the SD/MMC card
 */
LOCATE_AT(0x28200000) uint32_t Buff_Backup[BUFFER_SIZE/sizeof(uint32_t)];

/* Buffers for read/write operation */
LOCATE_AT(0x28000000) uint32_t Buff_Rd[BUFFER_SIZE/sizeof(uint32_t)];
LOCATE_AT(0x28100000) uint32_t Buff_Wr[BUFFER_SIZE/sizeof(uint32_t)];

/* Measurement data */
static uint32_t rd_ticks[NUM_ITER];
static uint32_t wr_ticks[NUM_ITER];

/* SD/MMC card information */
/* Number of sectors in SD/MMC card */
static int32_t tot_secs;

/* SDIO wait flag */
static volatile int32_t sdio_wait_exit = 0;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/* SDMMC card info structure */
mci_card_struct sdcardinfo;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Delay callback for timed SDIF/SDMMC functions */
static void sdmmc_waitms(uint32_t time)
{
	/* In an RTOS, the thread would sleep allowing other threads to run. 
	   For standalone operation, we just spin on RI timer */
	int32_t curr = (int32_t) Chip_RIT_GetCounter(LPC_RITIMER);
	int32_t final = curr + ((SystemCoreClock / 1000) * time);
	
	if (final == curr) return;

	if ((final < 0) && (curr > 0)) {
		while (Chip_RIT_GetCounter(LPC_RITIMER) < (uint32_t) final) {}
	}
	else {
		while ((int32_t) Chip_RIT_GetCounter(LPC_RITIMER) < final) {}
	}

	return;
}

/**
 * @brief	Sets up the SD event driven wakeup
 * @param	bits : Status bits to poll for command completion
 * @return	Nothing
 */
static void sdmmc_setup_wakeup(void *bits)
{
	uint32_t bit_mask = *((uint32_t *)bits);
	/* Wait for IRQ - for an RTOS, you would pend on an event here with a IRQ based wakeup. */
	NVIC_ClearPendingIRQ(SDIO_IRQn);
	sdio_wait_exit = 0;
	Chip_SDIF_SetIntMask(LPC_SDMMC, bit_mask);
	NVIC_EnableIRQ(SDIO_IRQn);
}

/**
 * @brief	A better wait callback for SDMMC driven by the IRQ flag
 * @return	0 on success, or failure condition (-1)
 */
static uint32_t sdmmc_irq_driven_wait(void)
{
	uint32_t status;

	/* Wait for event, would be nice to have a timeout, but keep it  simple */
	while (sdio_wait_exit == 0) {}

	/* Get status and clear interrupts */
	status = Chip_SDIF_GetIntStatus(LPC_SDMMC);

	return status;
}

/* Initialize SD/MMC */
static void App_SDMMC_Init()
{
	memset(&sdcardinfo, 0, sizeof(sdcardinfo));
	sdcardinfo.card_info.evsetup_cb = sdmmc_setup_wakeup;
	sdcardinfo.card_info.waitfunc_cb = sdmmc_irq_driven_wait;
	sdcardinfo.card_info.msdelay_func = sdmmc_waitms;

	/*  SD/MMC initialization */
	Board_SDMMC_Init();

	/* The SDIO driver needs to know the SDIO clock rate */
	Chip_SDIF_Init(LPC_SDMMC);
}

/* Buffer initialisation function */
static void Prepare_Buffer(uint32_t value)
{
    uint32_t i;
    
    for(i = 0; i < (BUFFER_SIZE/sizeof(uint32_t)); i++)
    {
        Buff_Rd[i] = 0x0;
        Buff_Wr[i] = i + value;
    }
}

#ifdef BOARD_HITEX_EVA_18504350
/* Initialize the UART for debugging */
static void board_uart_debug_init(void)
{
	/*  Hitex EVA A4 is sharing Uart port 0 and SDIO pin so it is needed to use Uart port 1 */
	Board_UART_Init(LPC_UART1);
	Chip_UART_SetBaud(LPC_UART1, 115200);
	Chip_UART_ConfigData(LPC_UART1, UART_DATABIT_8, UART_PARITY_NONE, UART_STOPBIT_1);	/*  default 8-N-1 */

	/*  Enable UART Transmit */
	Chip_UART_TxCmd(LPC_UART1, ENABLE);
}

/* Sends a character on the UART */
static void board_uart_out_ch(char ch)
{
	while (Chip_UART_SendByte(LPC_UART1, (uint8_t) ch) == ERROR) {}
}

/* Sends a string on the UART */
static void board_uart_out_string(char *str)
{
	while (*str != '\0') {
		board_uart_out_ch(*str++);
	}
}
#endif

/* Print the result of a data transfer */
static void print_meas_data(void)
{
	static char debugBuf[64];
    uint64_t tot_sum_rd, tot_sum_wr;
    uint32_t i, rd_ave, wr_ave, rd_time, wr_time;
	uint32_t clk = SystemCoreClock/1000000;

    /* Print Number of Interations */
	debugstr("\r\n=====================\r\n");
    debugstr("SDMMC Measurements \r\n");
	debugstr("=====================\r\n");
	sprintf(debugBuf, "No. of Iterations: %u \r\n", NUM_ITER);
	debugstr(debugBuf);
    sprintf(debugBuf, "No. of Sectors for R/W: %u \r\n", NUM_SECTORS);
    debugstr(debugBuf);
	sprintf(debugBuf, "Sector size : %u bytes \r\n", MMC_SECTOR_SIZE);
	debugstr(debugBuf);
	sprintf(debugBuf, "Data Transferred : %u bytes\r\n", (MMC_SECTOR_SIZE * NUM_SECTORS));
	debugstr(debugBuf);
    tot_sum_rd = tot_sum_wr = 0;
    for(i = 0; i < NUM_ITER; i++) {
        tot_sum_rd += rd_ticks[i];
        tot_sum_wr += wr_ticks[i];
    }
    rd_ave = tot_sum_rd / NUM_ITER; 
    wr_ave = tot_sum_wr / NUM_ITER;
	sprintf(debugBuf, "CPU Speed: %lu.%lu MHz\r\n", clk, (SystemCoreClock / 10000) - (clk * 100));
	debugstr(debugBuf);
    sprintf(debugBuf, "Ave Ticks for Read: %u \r\n", rd_ave);
    debugstr(debugBuf);
    sprintf(debugBuf, "Aver Ticks for Write: %u \r\n", wr_ave);
    debugstr(debugBuf);
	rd_time = (rd_ave / clk);
	wr_time = (wr_ave / clk);
	sprintf(debugBuf, "READ: Ave Time: %u usecs Ave Speed : %u KB/sec\r\n", rd_time, ((NUM_SECTORS * MMC_SECTOR_SIZE * 1000)/rd_time));
    debugstr(debugBuf);
	sprintf(debugBuf, "WRITE:Ave Time: %u usecs Ave Speed : %u KB/sec \r\n", wr_time, ((NUM_SECTORS * MMC_SECTOR_SIZE * 1000)/wr_time));
    debugstr(debugBuf);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	SDIO controller interrupt handler
 * @return	Nothing
 */
void SDIO_IRQHandler(void)
{
	/* All SD based register handling is done in the callback
	   function. The SDIO interrupt is not enabled as part of this
	   driver and needs to be enabled/disabled in the callbacks or
	   application as needed. This is to allow flexibility with IRQ
	   handling for applicaitons and RTOSes. */
	/* Set wait exit flag to tell wait function we are ready. In an RTOS,
	   this would trigger wakeup of a thread waiting for the IRQ. */
	NVIC_DisableIRQ(SDIO_IRQn);
	sdio_wait_exit = 1;
}

/**
 * @brief	Main routine for SDMMC example
 * @return	Nothing
 */
int main(void)
{
	uint32_t rc;	/* Result code */
    int32_t act_read, act_written;
    uint32_t i, ite_cnt;
	uint32_t start_time;
	uint32_t end_time;
	static char debugBuf[64];
    uint32_t backup = 0;

	Board_Init();
#ifdef BOARD_HITEX_EVA_18504350
	board_uart_debug_init();
#endif

    /* Disable SD/MMC interrupt */
	NVIC_DisableIRQ(SDIO_IRQn);

    /* Initialise SD/MMC card */ 
	App_SDMMC_Init();
	
	debugstr("\r\n==============================\r\n");
	debugstr("SDMMC CARD measurement demo\r\n");
	debugstr("==============================\r\n");
	    
	/* Enable SD/MMC Interrupt */
	NVIC_DisableIRQ(SDIO_IRQn);

	/* Wait for a card to be inserted (note CD is not on the
	   SDMMC power rail and can be polled without enabling
	   SD slot power */
	debugstr("\r\nWait till SD/MMC card inserted...\r\n");
	while (Chip_SDIF_CardNDetect(LPC_SDMMC));
	debugstr("\r\nSD/MMC Card inserted...\r\n");

	/* Enable slot power */
    Chip_SDIF_PowerOn(LPC_SDMMC);

	/* Enumerate the SDMMC card once detected. 
     * Note this function may block for a little while. */
	rc = Chip_SDMMC_Acquire(LPC_SDMMC, &sdcardinfo);
	if (!rc) {
	    debugstr("SD/MMC Card enumeration failed! ..\r\n");
		goto error_exit;
	}
   
    /* Check if Write Protected */ 
    rc = Chip_SDIF_CardWpOn(LPC_SDMMC);
	if (rc) {
        debugstr("SDMMC Card is write protected!, so tests can not continue..\r\n");
		goto error_exit;
    }

    /* Read Card information */
    tot_secs = Chip_SDMMC_GetDeviceBlocks(LPC_SDMMC);

    /* Make sure that the sectors are withing the card size */
    if((START_SECTOR + NUM_SECTORS) >= tot_secs) {
        debugstr("Out of range parameters! ..\r\n");
		goto error_exit;
    }

    /* Take back up of SD/MMC card contents so that
     * it can be restored so that SD/MMC card is not corrupted
     */
    debugstr("\r\nTaking back up of card.. \r\n");
    act_read = Chip_SDMMC_ReadBlocks(LPC_SDMMC, (void *)Buff_Backup, START_SECTOR, NUM_SECTORS);
    if(act_read == 0) {
        debugstr("Taking back up of card failed!.. \r\n");
		goto error_exit;
    }

    ite_cnt = 0;
    backup = 1;
    while(ite_cnt < NUM_ITER) {
        /* Prepare R/W buffers */
        Prepare_Buffer(ite_cnt);

        /* Write data to SD/MMC card */
	    start_time = Chip_RIT_GetCounter(LPC_RITIMER);
        act_written = Chip_SDMMC_WriteBlocks(LPC_SDMMC, (void *)Buff_Wr, START_SECTOR, NUM_SECTORS);
	    end_time = Chip_RIT_GetCounter(LPC_RITIMER);
        if(act_written == 0) {
            sprintf(debugBuf, "WriteBlocks failed for Iter: %u! \r\n", ite_cnt);
	        debugstr(debugBuf);
		    goto error_exit;
        }
        
        if(end_time < start_time) {
            continue;
        }
        else {
            wr_ticks[ite_cnt] = end_time - start_time;
        }
        
        /* Read data from SD/MMC card */
	    start_time = Chip_RIT_GetCounter(LPC_RITIMER);
        act_written = Chip_SDMMC_ReadBlocks(LPC_SDMMC, (void *)Buff_Rd, START_SECTOR, NUM_SECTORS);
	    end_time = Chip_RIT_GetCounter(LPC_RITIMER);
        if(act_read == 0) {
            sprintf(debugBuf, "ReadBlocks failed for Iter: %u! \r\n", ite_cnt);
	        debugstr(debugBuf);
		    goto error_exit;
        }
        
        if(end_time < start_time) {
            continue;
        }
        else {
            rd_ticks[ite_cnt] = end_time - start_time;
        }

        /* Comapre data */
        for(i = 0; i < (BUFFER_SIZE/sizeof(uint32_t)); i++)
        {
            if(Buff_Rd[i] != Buff_Wr[i])
            {
                sprintf(debugBuf, "Data mismacth: ind: %u Rd: 0x%x Wr: 0x%x \r\n", i, Buff_Rd[i], Buff_Wr[i]);
	            debugstr(debugBuf);
		        goto error_exit;
            }
        }
        ite_cnt++;
    }

	/* Print Measurement onto UART */
    print_meas_data();
 
error_exit:
    /* Restore if back up taken */
    if(backup) {
        debugstr("\r\nRestoring the contents of SDMMC card... \r\n");
        act_written = Chip_SDMMC_WriteBlocks(LPC_SDMMC, (void *)Buff_Backup, START_SECTOR, NUM_SECTORS);
        if(act_written == 0) {
            debugstr("Restoring contents failed!.. \r\n");
        }
    }
	
	debugstr("\r\n========================================\r\n");
	debugstr("SDMMC CARD measurement demo completed\r\n");
	debugstr("========================================\r\n");
	
    /* Wait forever */
	for (;; ) {}
}

/**
 * @}
 */
