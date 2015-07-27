/*
* @brief	Blinky example
*
* uC/OS-III 'blinky' example. Creates several LED blink threads and
* a UART output thread that ticks at about 1Hz.
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
#include "os.h"

/** @defgroup UCOS_III_18XX43XX_BLINKY LPC18xx/43xx uC/OS-III blinky example
 * @ingroup EXAMPLES_UCOSIII_18XX43XX
 * <b>Example description</b><br>
 * Welcome to the uC/OS-III basic blinky example. This example starts up
 * uC/OS-III and creates 3 tasks. Tasks 1 and 2 blink different LEDs at
 * different rates. Task 3 outputs a tick count to the debug channel (UART)
 * every second.<br>
 *
 * To use the example, connect a serial cable to the board's RS232/UART port
 * and start a terminal program (115.2K8N1) to monitor the port. The LEDs will
 * also toggle based on the task execution.<br>
 *
 * <b>Special connection requirements</b><br>
 * There are no special connection requirements for this example.<br>
 *
 * @ref MICRIUM_UCOSIII_LICENSING_TERMS
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

// FIXME - needs standards formatting

static OS_TCB  vLEDTask1TCB, vLEDTask2TCB, vUARTTaskTCB;
static CPU_STK vLEDTask1Stk[APP_CFG_TASK_START_STK_SIZE];
static CPU_STK vLEDTask2Stk[APP_CFG_TASK_START_STK_SIZE];
static CPU_STK vUARTTaskStk[APP_CFG_TASK_START_STK_SIZE];
extern void OS_CSP_TickInit(void);

/* Sets up system hardware */
static void prvSetupHardware(void)
{
	Board_Init();

	/* Initial LED0 state is off */
	Board_LED_Set(0, false);
}

/* LED2 toggle thread */
static void vLEDTask2 (void *p_arg)
{
	OS_ERR err;
	bool LedState = true;

	while (1) {
		Board_LED_Set(1, LedState);
		LedState = (bool) !LedState;

		/* About a 7Hz on/off toggle rate */
    OSTimeDlyHMSM(0u, 0u, 0u, 71u, OS_OPT_TIME_HMSM_STRICT, &err);
	}
}

/* UART (or output) thread */
static void vUARTTask (void *p_arg)
{
	OS_ERR err;
	int tickCnt = 0;

	while (1) {
		DEBUGOUT("Tick: %d\r\n", tickCnt);
		tickCnt++;

		/* About a 1s delay here */
    OSTimeDlyHMSM(0u, 0u, 1u, 0u, OS_OPT_TIME_HMSM_STRICT, &err);
	}
}

/* LED1 toggle thread */
static void vLEDTask1 (void *p_arg)
{
	OS_ERR os_err;
	bool LedState = true;

	/* Start tick */
	OS_CSP_TickInit();

	/* Other tasks */
	OSTaskCreate(	(OS_TCB      *)&vLEDTask2TCB,
								(CPU_CHAR    *)"vLEDTask2",
								(OS_TASK_PTR  )vLEDTask2, 
								(void        *)0,
								(OS_PRIO      )APP_CFG_TASK_START_PRIO + 1,
								(CPU_STK     *)vLEDTask2Stk,
								(CPU_STK_SIZE )APP_CFG_TASK_START_STK_SIZE_LIMIT,
								(CPU_STK_SIZE )APP_CFG_TASK_START_STK_SIZE,
								(OS_MSG_QTY   )0u,
								(OS_TICK      )0u,
								(void        *)0,
								(OS_OPT       )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
								(OS_ERR      *)&os_err);
	OSTaskCreate(	(OS_TCB      *)&vUARTTaskTCB,
								(CPU_CHAR    *)"vUARTTask",
								(OS_TASK_PTR  )vUARTTask, 
								(void        *)0,
								(OS_PRIO      )APP_CFG_TASK_START_PRIO + 2,
								(CPU_STK     *)vUARTTaskStk,
								(CPU_STK_SIZE )APP_CFG_TASK_START_STK_SIZE_LIMIT,
								(CPU_STK_SIZE )APP_CFG_TASK_START_STK_SIZE,
								(OS_MSG_QTY   )0u,
								(OS_TICK      )0u,
								(void        *)0,
								(OS_OPT       )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
								(OS_ERR      *)&os_err);

	while (1) {
		Board_LED_Set(0, LedState);
		LedState = (bool) !LedState;

		/* About a 3Hz on/off toggle rate */
    OSTimeDlyHMSM(0u, 0u, 0u, 156u, OS_OPT_TIME_HMSM_STRICT, &os_err);
	}
}

/** 
 * @brief  main routine for uC/OS-III blinky example
 * @return Function should not exit.
 */
int main(void)
{
	OS_ERR os_err;

	prvSetupHardware();

	CPU_Init();

	OSInit(&os_err);

	OSTaskCreate(	(OS_TCB      *)&vLEDTask1TCB,
								(CPU_CHAR    *)"vLEDTask1",
								(OS_TASK_PTR  )vLEDTask1, 
								(void        *)0,
								(OS_PRIO      )APP_CFG_TASK_START_PRIO,
								(CPU_STK     *)vLEDTask1Stk,
								(CPU_STK_SIZE )APP_CFG_TASK_START_STK_SIZE_LIMIT,
								(CPU_STK_SIZE )APP_CFG_TASK_START_STK_SIZE,
								(OS_MSG_QTY   )0u,
								(OS_TICK      )0u,
								(void        *)0,
								(OS_OPT       )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
								(OS_ERR      *)&os_err);

	OSStart(&os_err);

	(void) &os_err;

	return 0;
}

/** @} */
