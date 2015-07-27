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

#ifndef __IPC_MSG_H_
#define __IPC_MSG_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup EXAMPLE_DUALCORE_CMN_IPC LPC43xx Inter Processor Communication(IPC) functions
 * @ingroup EXAMPLES_DUALCORE_43XX_COMMON
 * The Inter Processor Communication(IPC) functions provide standalone and RTOS
 * based support functions for sharing data between the M0 and M4 core.
 *
 * Build procedure:
 * <a href="http://www.lpcware.com/content/project/lpcopen-platform-nxp-lpc-microcontrollers/lpcopen-build-procedures/lpc18xx/43xx-lpco">LPCOpen 18xx/43xx build instructions</a>
 *
 * Submit bug reports for LPCOpen code <a href="http://www.lpcware.com/content/bugtrackerproject/lpcopen">here.</a>
 * @{
 */

/* Queue status macro */
/**
 * \def QUEUE_DATA_COUNT(q)
 * This macro will get the number of items pending in \a q
 */
#define QUEUE_DATA_COUNT(q) ((uint32_t) ((q)->head - (q)->tail))
/**
 * \def QUEUE_IS_FULL(q)
 * This macro will evaluate to 1 if queue \a q is full, 0 if it is not
 */
#define QUEUE_IS_FULL(q)    (QUEUE_DATA_COUNT(q) >= (q)->count)
/**
 * \def QUEUE_IS_EMPTY(q)
 * This macro will evaluate to 1 if queue \a q is empty, 0 if it is not
 */
#define QUEUE_IS_EMPTY(q)   ((q)->head == (q)->tail)
/**
 * \def QUEUE_IS_VALID(q)
 * This macro will evaluate to 1 if queue \a q is initialized & valid, 0 if it is not
 */
#define QUEUE_IS_VALID(q)   ((q)->valid == QUEUE_MAGIC_VALID)
/**
 * @brief IPC Queue Structure used for sync between M0 and M4.
 *
 * This structure provides head and tail indexes
 * to the uint32_t array of messages.
 */
struct ipc_queue {
	int32_t size;				/*!< Size of a single item in queue */
	int32_t count;				/*!< Toal number of elements that can be stored in the queue */
	volatile uint32_t head;		/*!< Head index of the queue */
	volatile uint32_t tail;		/*!< Tail index of the queue */
	uint8_t *data;				/*!< Pointer to the data */
	uint32_t valid;             /*!< Queue is valid only if this is #QUEUE_MAGIC_VALID */
	uint32_t reserved[2];		/*!< Reserved entry to keep the structure aligned */
};

/* IPC Function return values */
/**
 * \def QUEUE_VALID
 * Returned by dualcore IPC function when an item is 
 * successfully fetched from Message Queue
 */
#define QUEUE_VALID      1

/**
 * \def QUEUE_INSERT
 * Returned by dualcore IPC function when an item is
 * successfully inserted into the Message Queue
 */
#define QUEUE_INSERT     0

/**
 * \def QUEUE_FULL
 * Returned by dualcore IPC function when an attempt is
 * made to insert item into the Message Queue when the
 * queue is full.
 */
#define QUEUE_FULL      -1

/**
 * \def QUEUE_EMPTY
 * Returned by dualcore IPC function when an attempt is
 * made to fetch/pop an item from the Message Queue when the
 * queue is empty.
 */
#define QUEUE_EMPTY     -2

/**
 * \def QUEUE_ERROR
 * Returned by dualcore IPC function when an push/pop is made
 * to a queue which was not initialized using IPC_initMsgQueue()
 */
#define QUEUE_ERROR     -3

/**
 * \def QUEUE_TIMEOUT
 * Returned by dualcore IPC function when a push/pop operation
 * did not complete within a given time.
 */
#define QUEUE_TIMEOUT   -4

/**
 * \def QUEUE_MAGIC_VALID
 * Macro used to identify if a queue is valid
 */
#define QUEUE_MAGIC_VALID   0xCAB51053

/**
 * @brief	Function to push a message into queue with timeout
 *
 * This function will push an message of size \a size, specified by
 * IPC_initMsgQueue. If this function is called from M4 Core the message
 * will be pushed to M4 Queue and will be popped by M0, and vice-versa.
 *
 * @param	data	: Pointer to data to be pushed
 * @param	tout	: non-zero value - timeout value in milliseconds,
 *                      zero value - no blocking,
 *                      negative value - blocking
 * @return  #QUEUE_INSERT on success,
 * @note	#QUEUE_FULL or #QUEUE_ERROR on failure,
 *          #QUEUE_TIMEOUT when there is a timeout
 */
int IPC_pushMsgTout(const void *data, int tout);

/**
 * @brief	Function to read a message from queue with timeout
 *
 * This function will pop a message of size \a size, specified by
 * IPC_initMsgQueue of the other core. If this function is called from
 * M4 Core the message will be popped from M0 Queue and vice-versa.
 *
 * @param	data	: Pointer to store popped data
 * @param	tout	: non-zero value - timeout value in milliseconds,
 *                      zero value - no blocking,
 *                      negative value - blocking
 * @return	#QUEUE_VALID on success,
 * @note	#QUEUE_EMPTY or #QUEUE_ERROR on failure,
 *          #QUEUE_TIMEOUT when there is a timeout
 */
int IPC_popMsgTout(void *data, int tout);

/**
 * @brief	Function to push the message into queue with no wait
 *
 * The function tries to push the \a data to the IPC queue. If the 
 * queue is full, it will return without blocking. It is equivalent
 * to calling IPC_pushMsgTout(data, 0).
 *
 * @param	data	: Pointer to data to be pushed
 * @return	#QUEUE_INSERT on success, #QUEUE_FULL or #QUEUE_ERROR on failure
 */
static INLINE int IPC_tryPushMsg(const void *data)
{
	return IPC_pushMsgTout(data, 0);
}

/**
 * @brief	Function to pop the message from queue with no wait
 *
 * The function tries to pop the data from the IPC queue. If the queue is empty,
 * it will return without blocking.
 *
 * @param	data	: Pointer to store popped data
 * @return	#QUEUE_VALID on success, #QUEUE_ERROR or #QUEUE_EMPTY on failure
 */
static INLINE int IPC_tryPopMsg(void *data)
{
	return IPC_popMsgTout(data, 0);
}

/**
 * @brief	Function to push the message into queue with wait
 *
 * The function to push the data to the IPC queue. If the queue is full, it will
 * wait till data is pushed (blocking).
 *
 * @param	data	: Pointer to data to be pushed
 * @return	#QUEUE_INSERT on success, #QUEUE_ERROR on failure
 */
static INLINE int IPC_pushMsg(const void *data)
{
	return IPC_pushMsgTout(data, -1);
}

/**
 * @brief	Function to pop the message from queue with wait
 *
 * The function will pop the data from the IPC queue. If the queue is empty,
 * it will wait (blocking) till the data is available.
 *
 * @param	data	: Pointer to store popped data
 * @return	status	: Pop status
 */
static INLINE int IPC_popMsg(void *data)
{
	return IPC_popMsgTout(data, -1);
}

/**
 * @brief	Get number of pending items in queue
 *
 * This function will get the number of pending items in the queue. If
 * this function is called by M4 core with param \a queue_write set to
 * non zero then this function will return the number of messages waiting
 * in M4's Push Queue that is yet to be popped by the M0 core, or vice versa.
 * If this function is called by M4 core with param \a queue_write set to
 * zero then it returns the number of elements that can be popped without
 * waiting (in other words number of messages pending in the M0's Push queue
 * that is yet to be popped by M4 core), or vice versa.
 *
 * @param	queue_write	: 1 - read number of elements in write queue,
 *                        0 - read number of elements in read queue
 * @return	On success - Number of elements in queue (which will be >= 0),
 * @note	On Error   - #QUEUE_ERROR (when queue is not initialized/valid)
 */
int IPC_msgPending(int queue_write);

/**
 * @brief	Function to send notificaton interrupt
 *
 * When called from the M0, it sends the notification interrupt to M4 and vice-versa.
 *
 * @return	None
 */
void IPC_msgNotify(void);

/**
 * @brief	Function to initialize the IPC message queue
 *
 * This function intializes the interprocessor communication message queue.
 * **IMPORTANT NOTE: \a MaxNoOfMsg must always be a power of 2.**
 *
 * @param	data	: Pointer to the array of messages of size \a msgSize
 * @param	msgSize	: Size of the single data element in queue
 * @param	maxNoOfMsg	: Maximum number of items that can be stored in Queue
 * @return	None, will not return if there is error in given arguments
 */
void IPC_initMsgQueue(void *data, int msgSize, int maxNoOfMsg);

/**
 * @brief	Function to convert IPC error number to string
 *
 * This function returns the error string for the error value.
 *
 * @param	errnum : IPC error value
 * @return	pointer to error string
 */
const char *IPC_strerror(int errnum);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ifndef __IPC_MSG_H_ */
