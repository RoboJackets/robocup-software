/*
 * @brief Startup/Main file for Dual core demos
 *
 * Startup file (having reset and main routines)
 * This file provides functions necessary to start all the example tasks
 * based on the configuration.
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

/* General includes */
#include <stdio.h>
#include "lpc43xx_dualcore_config.h"
#include "ipc_msg.h"

#if defined(OS_FREE_RTOS)
#include "FreeRTOS.h"
#include "task.h"

#elif defined(OS_UCOS_III)
#include "os.h"
#endif

/** @defgroup EXAMPLE_DUALCORE_CMN_MAIN LPC43xx startup code for dual-core demos
 * @ingroup EXAMPLES_DUALCORE_43XX_COMMON
 * <b>Example description</b><br>
 * The main tasks for the M0 and M4 cores are processed from here.<br>
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

/* Macro that calculates the start address of M0 image */
#define M0_IMAGE_ADDR  (IMAGE_BASE_ADDR + M0_IMAGE_OFFSET)

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

#ifdef OS_UCOS_III
/* UCOS-III Idle task hook */
static void APP_Task_Idle_Hook(void)
{
	__WFI();
}
#endif

/* initialization routine for dual core examples */
static void prvSetupHardware(void)
{
#ifdef CORE_M4
	/* Re-initialize CGU for proper operation */
	Board_Init();

	/* Time to Start M0 */
	if (M0Image_Boot((uint32_t) M0_IMAGE_ADDR) < 0) {
		while (1) {
			__WFI();
		}
	}
	MSleep(100);
#elif defined(CORE_M0)
	extern void prvSetupTimerInterrupt(void);

	/* Needs to be called coz durinig initializtion the
	 * global variable would have initialized to 0
	 */
	SystemCoreClockUpdate();

	#ifdef OS_FREE_RTOS
	/* Disable global interrupts */
	taskDISABLE_INTERRUPTS();
	prvSetupTimerInterrupt();
	#endif
#endif
#ifdef OS_UCOS_III
	do {
		extern void OS_CSP_TickInit(void);
		OS_ERR ret;
		
		CPU_IntDis();
		CPU_Init();
		OSInit(&ret);
		if (ret != OS_ERR_NONE) {
			DEBUGSTR("Unable init UCOS-III OS!\r\n");
			while (1) {}
		}
		OS_CSP_TickInit();
		OS_AppIdleTaskHookPtr = APP_Task_Idle_Hook;
	} while(0);
#endif
	/* Initialize the IPC Queue */
	IPCEX_Init();

	#ifdef EXAMPLE_USB_HOST
	USBHOST_Init();
	#endif
	#ifdef EXAMPLE_USB_DEVICE
	USBDEV_Init();
	#endif
	#ifdef EXAMPLE_LWIP
	LWIP_Init();
	#endif
	#ifdef EXAMPLE_EMWIN
	EMWIN_Init();
	#endif
	#ifdef EXAMPLE_BLINKY
	BLINKY_Init();
	#endif
}

/* Main tasks of LPC43xx Dual core examples */
static void main_tasks(void)
{
#if (defined(OS_FREE_RTOS) || defined(OS_UCOS_III))
	const int loop = 0;
#else
	const int loop = 1;
#endif

	do {
		ipcex_tasks();
#ifdef EXAMPLE_BLINKY
		blinky_tasks();
#endif
#ifdef EXAMPLE_USB_HOST
		usb_host_tasks();
#endif
#ifdef EXAMPLE_USB_DEVICE
		usb_device_tasks();
#endif
#ifdef EXAMPLE_LWIP
		lwip_tasks();
#endif
#ifdef EXAMPLE_EMWIN
		emwin_tasks();
#endif
	} while (loop);

#ifdef OS_FREE_RTOS
	/* Start the scheduler */
	vTaskStartScheduler();
#endif

#ifdef OS_UCOS_III
	do {
		OS_ERR ret;
		OSStart(&ret);
		if (ret != OS_ERR_NONE) {
			DEBUGSTR("Unable start UCOS-III OS!\r\n");
			while (1) {}
		}
	} while(0);
#endif

	/* Control should never come here */
	DEBUGSTR("Schedule Failure\r\n");
	while (1) {}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/
/* Milli-second sleep function */
void MSleep(int32_t msecs)
{
	int32_t curr = (int32_t) Chip_RIT_GetCounter(LPC_RITIMER);
	int32_t final = curr + ((SystemCoreClock / 1000) * msecs);

	/* If the value is zero let us not worry about it */
	if (!msecs || (msecs < 0)) {
		return;
	}

	if ((final < 0) && (curr > 0)) {
		while (Chip_RIT_GetCounter(LPC_RITIMER) < (uint32_t) final) {}
	}
	else {
		while ((int32_t) Chip_RIT_GetCounter(LPC_RITIMER) < final) {}
	}
}

/**
 * @brief	Main for dual core examples
 *
 * Entry point for all the dual core examples. All the dual core
 * example execution starts from this function, it is common for
 * code that runs on core-M4 and core-M0.
 *
 * @return  Function should not return.
 */
int main(void)
{
	prvSetupHardware();
#ifdef CORE_M0
	DEBUGSTR("Starting M0 Tasks...\r\n");
#else
	DEBUGSTR("Starting M4 Tasks...\r\n");
#endif
	main_tasks();
	return 0;
}

/**
* @}
*/
