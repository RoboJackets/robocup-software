/*
 * @brief LPC43XX Inter Processor Communication(IPC) functions
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
#include "lpc43xx_dualcore_config.h"
#include "ipc_msg.h"

#if defined(OS_FREE_RTOS)
#include "FreeRTOS.h"
#include "semphr.h"

#elif defined(OS_UCOS_III)
#include "os.h"

#endif

/** @ingroup EXAMPLE_DUALCORE_CMN_IPC
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define SHMEMM0     LOCATE_AT(SHARED_MEM_M0)
#define SHMEMM4     LOCATE_AT(SHARED_MEM_M4)

/*
 * M0 write queue
 * This is the queue which will be written by M0 and be read by M4
 * NOTE:
 * This variable is put in SHMEMM0 section which will
 * be linked to shared memory region by linker script.
 */
SHMEMM0 static struct ipc_queue queue_m0 = {0};

/*
 * M4 write queue
 * This is the queue which will be written by M4 and be read by M0
 * NOTE:
 * This variable is put in SHMEMM4 section which will
 * be linked to shared memory region by linker script.
 */
SHMEMM4 static struct ipc_queue queue_m4 = {0};

/*
 * IRQ priority of IPC interrupt
 * Currently this has the highest priority
 */
#define IPC_IRQ_Priority    IRQ_PRIO_IPC

#ifdef CORE_M4
/*
 * M4 read queue pointer
 * M4 reads from this message queue based on the
 * tail index.
 */
static struct ipc_queue *qrd = &queue_m0;
/*
 * M4 write queue pointer
 * M4 writes from this message queue based on the
 * head index.
 */
static struct ipc_queue *qwr = &queue_m4;

#define IPC_IRQHandler MX_CORE_IRQHandler
#define ClearTXEvent   Chip_CREG_ClearM0Event
#define IPC_IRQn       M0CORE_IRQn

#elif defined(CORE_M0)
static struct ipc_queue *qrd = &queue_m4;
static struct ipc_queue *qwr = &queue_m0;
#define IPC_IRQHandler MX_CORE_IRQHandler
#define ClearTXEvent   Chip_CREG_ClearM4Event
#define IPC_IRQn       M0_M4CORE_IRQn

#else
#error "For LPC43XX, CORE_M0 or CORE_M4 must be defined!"
#endif

/* FreeRTOS functions */
#ifdef OS_FREE_RTOS
/* FreeRTOS semaphores for event handling */
static xSemaphoreHandle event_tx, event_rx;

#elif defined(OS_UCOS_III)
static OS_SEM event_tx, event_rx;
#endif

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

#ifdef OS_FREE_RTOS
/*****************************************************************************
 * FreeRTOS functions
 ****************************************************************************/

/* OS specific event hanlder for FreeRTOS */
static void os_event_handler(void)
{
	portBASE_TYPE wake1 = pdFALSE, wake2 = pdFALSE;

	if (event_rx && !QUEUE_IS_EMPTY(qrd)) {
		xSemaphoreGiveFromISR(event_rx, &wake1);
	}

	if (event_tx && !QUEUE_IS_FULL(qwr)) {
		xSemaphoreGiveFromISR(event_tx, &wake2);
	}

	portEND_SWITCHING_ISR(wake1 || wake2);
}

/* Misc Init function that initializes OS semaphores */
static void ipc_misc_init(void)
{
	vSemaphoreCreateBinary(event_tx);
	vSemaphoreCreateBinary(event_rx);

	if (!event_tx || !event_rx) {
		DEBUGSTR("ERROR: Unable to create FreeRTOS IPC event semaphores.\r\n");
		while (1) {	/* DIE: unable to create semaphores */
		}
	}
}

/* Macro that waits till TX/RX event to happen */
#define ipc_wait_event(evt, sem) while ((evt)) xSemaphoreTake(sem, 100)
/* Macro that waits till TX/RX event to happen before tout (timeout) */
#define ipc_wait_event_tout(evt, tout, sem)	\
	do { \
		while ((evt)) {	\
			if ((evt) && xSemaphoreTake(sem, tout) != pdTRUE) \
			{tout = 0; break; }} \
	} while (0)

#elif defined(OS_UCOS_III)
/*****************************************************************************
 * uCOS-III functions
 ****************************************************************************/

static void ipc_misc_init(void)
{
	OS_ERR ret;
	OSSemCreate(&event_tx, "TX Sema", 0, &ret);
	if (ret != OS_ERR_NONE) {
		while (1) {}
	}
	OSSemCreate(&event_rx, "RX Sema", 0, &ret);
	if (ret != OS_ERR_NONE) {
		while (1) {}
	}
}

static void os_event_handler(void)
{
	OS_ERR ret;
	if (!QUEUE_IS_EMPTY(qrd)) {
		OSSemPost(&event_rx, OS_OPT_POST_ALL, &ret);
	}

	if (!QUEUE_IS_FULL(qwr)) {
		OSSemPost(&event_tx, OS_OPT_POST_ALL, &ret);
	}
}

/* Wait for the event */
#define ipc_wait_event(evt, sem) \
	while ((evt)) {OS_ERR ret; \
				   OSSemPend(&(sem), (OS_TICK) 100, OS_OPT_PEND_BLOCKING, (CPU_TS *) 0, &ret); }
#define ipc_wait_event_tout(evt, tout, sem)	\
	do { \
		while ((evt)) {	\
			OS_ERR ret;	\
			OSSemPend(&(sem), (OS_TICK) tout, OS_OPT_PEND_BLOCKING, (CPU_TS *) 0, &ret); \
			if ((evt) && ret == OS_ERR_TIMEOUT) {tout = 0; break; }	\
		} \
	} while (0)

#else
/*****************************************************************************
 * uCOS-III functions
 ****************************************************************************/

static void os_event_handler(void)
{
	/* Nothing to do here */
}

/* Wait for the event */
#define ipc_wait_event(evt, sem) while ((evt))

#define ipc_wait_event_tout(evt, tout, sem)	\
	do { \
		uint32_t cnt = Chip_RIT_GetCounter(LPC_RITIMER) + (tout * (SystemCoreClock / 1000)); \
		if (cnt + 5000 < cnt) {cnt += 5000; } \
		while ((evt) && Chip_RIT_GetCounter(LPC_RITIMER) < cnt) {}	 \
		if (evt) {tout = 0; } \
	} while (0)

static void ipc_misc_init(void)
{}

#endif

/*
 * Initiate interrupt on other processor
 * Upon calling this function generates and interrupt on the other
 * core. Ex. if called from M0 core it generates interrupt on M4 core
 * and vice versa.
 */
static void ipc_send_signal(void)
{
	__DSB();
	__SEV();
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Function to send notificaton interrupt */
void IPC_msgNotify(void)
{
	ipc_send_signal();
}

/* Function to initialize the IPC message queue */
void IPC_initMsgQueue(void *data, int size, int count)
{
	/* Sanity Check */
	if (!size || !count || !data) {
		DEBUGSTR("ERROR:IPC Queue size invalid parameters\r\n");
		while (1) {}
	}

	/* Check if size is a power of 2 */
	if (count & (count - 1)) {
		DEBUGSTR("ERROR:IPC Queue size not power of 2\r\n");
		while (1) {	/* BUG: Size must always be power of 2 */
		}
	}

	memset(qwr, 0, sizeof(*qwr));
	ipc_misc_init();
	qwr->count = count;
	qwr->size = size;
	qwr->data = data;
	qwr->valid = QUEUE_MAGIC_VALID;
	NVIC_SetPriority(IPC_IRQn, IPC_IRQ_Priority);
	NVIC_EnableIRQ(IPC_IRQn);
}

/* Function to push a message into queue with timeout */
int IPC_pushMsgTout(const void *data, int tout)
{
	/* Check if write queue is initialized */
	if (!QUEUE_IS_VALID(qwr)) {
		return QUEUE_ERROR;
	}

	if (tout == 0) {
		/* Check if queue is full */
		if (QUEUE_IS_FULL(qwr)) {
			return QUEUE_FULL;
		}
	}
	else if (tout < 0) {
		/* Wait for read queue to have some data */
		ipc_wait_event(QUEUE_IS_FULL(qwr), event_tx);
	}
	else {
		/* Wait for read queue to have some data */
		ipc_wait_event_tout(QUEUE_IS_FULL(qwr), tout, event_tx);
		if (tout == 0) {
			return QUEUE_TIMEOUT;
		}
	}

	memcpy(qwr->data + ((qwr->head & (qwr->count - 1)) * qwr->size), data, qwr->size);
	qwr->head++;
	ipc_send_signal();

	return QUEUE_INSERT;
}

/* Function to read a message from queue with timeout */
int IPC_popMsgTout(void *data, int tout)
{
#ifdef EVENT_ON_RX
	int raise_event = QUEUE_IS_FULL(qrd);
#endif

	if (!QUEUE_IS_VALID(qrd)) {
		return QUEUE_ERROR;
	}

	if (tout == 0) {
		/* Check if read queue is empty */
		if (QUEUE_IS_EMPTY(qrd)) {
			return QUEUE_EMPTY;
		}
	}
	else if (tout < 0) {
		/* Wait for read queue to have some data */
		ipc_wait_event(QUEUE_IS_EMPTY(qrd), event_rx);
	}
	else {
		/* Wait for event or timeout */
		ipc_wait_event_tout(QUEUE_IS_EMPTY(qrd), tout, event_rx);
		if (tout == 0) {
			return QUEUE_TIMEOUT;
		}
	}

	/* Pop the queue Item */
	memcpy(data, qrd->data + ((qrd->tail & (qrd->count - 1)) * qrd->size), qrd->size);
	qrd->tail++;

#ifdef EVENT_ON_RX
	if (raise_event) {
		ipc_send_signal();
	}
#endif
	return QUEUE_VALID;
}

/* Get number of pending items in queue */
int IPC_msgPending(int queue_write)
{
	struct ipc_queue *q = queue_write ? qwr : qrd;
	if (!QUEUE_IS_VALID(q))
		return QUEUE_ERROR;

	return QUEUE_DATA_COUNT(q);
}

/**
 * @brief	Call-back function to handle IPC Message receive event
 * @return	None
 * This is a weak function (hence can be overridden by the user's application
 * with a function of same name. This call-back function will be called
 * whenever there is a message received by the core that implements this
 * function.
 */
#ifdef __IAR_SYSTEMS_ICC__
__weak void EVENT_IPC_Receive(void)
#else
void EVENT_IPC_Receive(void) __attribute__ ((weak));

void EVENT_IPC_Receive(void)
#endif
{}

/**
 * @brief	Interrupt handler for IPC interrupts
 * @return	None
 */
void IPC_IRQHandler(void)
{
	/* Clear the interrupt */
	ClearTXEvent();

	/* Invoke OS Specific handler */
	os_event_handler();

	/* Invoke handler */
	EVENT_IPC_Receive();
}

/* Function to convert IPC error number to string */
const char *IPC_strerror(int errnum)
{
	static const char *ipc_errstr[] = {
		"Queue Insert OK",
		"Queue Pop OK/Valid",
		"Queue is Full",
		"Queue is Empty",
		"Queue Error/Not initialized",
		"Queue operation timed out",
	};
	if (errnum < 0) {
		errnum = 1 - errnum;
	}
	return ipc_errstr[errnum];
}

/**
 * @}
 */
