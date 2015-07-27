/*
 * @brief Example implementation of IPC using IPC library
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

#ifndef _IPC_EXAMPLE_H_
#define _IPC_EXAMPLE_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup EXAMPLE_DUALCORE_CMN_IPC_EX LPC43xx Inter Processor Communication(IPC) example
 * @ingroup EXAMPLE_DUALCORE_CMN_IPC
 *
 * Build procedure:
 * <a href="http://www.lpcware.com/content/project/lpcopen-platform-nxp-lpc-microcontrollers/lpcopen-build-procedures/lpc18xx/43xx-lpco">LPCOpen 18xx/43xx build instructions</a>
 *
 * Submit bug reports for LPCOpen code <a href="http://www.lpcware.com/content/bugtrackerproject/lpcopen">here.</a>
 * @{
 */

/**
 * \def IPCEX_QUEUE_SZ
 * Size of the IPC Queue used by this example implimentation, the current
 * implementation uses same queue size for both M0 and M4, application
 * developer can override it.
 */
#define IPCEX_QUEUE_SZ        64

/**
 * \def IPCEX_MAX_IDS
 * Maximum number of message IDs that this IPC examples uses
 */
#define IPCEX_MAX_IDS         20

#define IPCEX_ID_BLINKY        1  /*!< IPC ID used by BLINKY example */
#define IPCEX_ID_USBHOST       2  /*!< IPC ID used by USB HOST example */
#define IPCEX_ID_USBDEVICE     3  /*!< IPC ID used by USB Device example */
#define IPCEX_ID_LWIP          4  /*!< IPC ID used by LWIP example */
#define IPCEX_ID_EMWIN         5  /*!< IPC ID used by EMWIN example */
#define IPCEX_ID_USER1         10 /*!< IPC ID that can be used by other user examples */
#define IPCEX_ID_USER2         11 /*!< IPC ID that can be used by other user examples */

typedef struct __ipcex_msg {
	uint32_t id;
	uint32_t data;
} ipcex_msg_t;

/**
 * @brief	IPC register callback function pointer
 *
 * This function registers a callback function pointer to a
 * message \a id, whenever a message with \a id is received the
 * register call-back function will be invoked.
 *
 * @param	id		: ID of message to which the callback \a func be associated
 * @param	func	: pointer to callback function
 * @return	0 on failure [given \a id is greater than 
 * @note	#IPCEX_MAX_IDS], !0 on success
 */
int ipcex_register_callback(uint32_t id, void (*func)(uint32_t));

/**
 * @brief	Push data on the queue
 * @param	id		: Task ID of the destination task
 * @param	data	: Data containing the message
 * @return	#QUEUE_ERROR or #QUEUE_FULL on error, #QUEUE_INSERT on success
 */
int ipcex_msgPush(uint32_t id, uint32_t data);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* _IPC_EXAMPLE_H_ */
