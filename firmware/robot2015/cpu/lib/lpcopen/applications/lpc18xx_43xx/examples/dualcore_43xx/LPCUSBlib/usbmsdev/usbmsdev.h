/*
 * @brief	LPCUSBlib Mass Storage Device Dual core example
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

/** \file
 *
 *  Header file for usbmsdev.c.
 */

#ifndef __USBMSDEV_H_
#define __USBMSDEV_H_

#define __DATARAM_H_

/* Includes: */
#include "board.h"
#include "lpc43xx_dualcore_config.h"

#include "USB.h"
#include <string.h>
#include "Descriptors.h"
#include "Lib/SCSI.h"

/* OS Specific includes */
#ifdef OS_FREE_RTOS
/* FreeRTOS headers */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#define  LOOP_TASKS    1
#elif defined(OS_UCOS_III)
#include "os.h"
#include "cpu.h"
#include "os_cfg_app.h"
#define  LOOP_TASKS    1
#else
#define  LOOP_TASKS    0
#endif

/** @defgroup EXAMPLE_DUALCORE_MassStorage LPCUSBlib Mass Storage Device Dual core example
 * @ingroup EXAMPLES_DUALCORE_43XX
 * The Mass Storage Device dual core example demonstrates the Mass Storage Device example
 * using LPCUSBlib library. The LPC43XX will be enumerated as as Mass Storage Device when
 * connected to the Host PC. The example can be configured to run on M4/M0 core.
 * @{
 */

/* Macros: */
/** LED mask for the library LED driver, to indicate that the USB interface is not ready. */
	#define LEDMASK_USB_NOTREADY      LEDS_LED1

/** LED mask for the library LED driver, to indicate that the USB interface is enumerating. */
	#define LEDMASK_USB_ENUMERATING  (LEDS_LED2 | LEDS_LED3)

/** LED mask for the library LED driver, to indicate that the USB interface is ready. */
	#define LEDMASK_USB_READY        (LEDS_LED2 | LEDS_LED4)

/** LED mask for the library LED driver, to indicate that an error has occurred in the USB interface. */
	#define LEDMASK_USB_ERROR        (LEDS_LED1 | LEDS_LED3)

/** LED mask for the library LED driver, to indicate that the USB interface is busy. */
	#define LEDMASK_USB_BUSY          LEDS_LED2

/** Total number of logical drives within the device - must be non-zero. */
	#define TOTAL_LUNS                1

/** Blocks in each LUN, calculated from the total capacity divided by the total number of Logical Units in the device. */
	#define LUN_MEDIA_BLOCKS         (VIRTUAL_MEMORY_BLOCKS / TOTAL_LUNS)

/** Indicates if the disk is write protected or not. */
	#define DISK_READ_ONLY            false

/**
 * @brief	USB Device connect event callback
 * @return	None
 * @note	This is the USB Device connect event call back function
 */
void EVENT_USB_Device_Connect(void);

/**
 * @brief	USB Device disconnect event callback
 * @return	None
 * @note	This is the USB Device disconnect event call back function
 */
void EVENT_USB_Device_Disconnect(void);

/**
 * @brief	USB Device configuration change event callback
 * @return	None
 * @note	This is the USB Device configuration change event call back function
 */
void EVENT_USB_Device_ConfigurationChanged(void);

/**
 * @brief	USB Device control request receive event callback
 * @return	None
 * @note	This is the USB Device control request receive event call back function
 */
void EVENT_USB_Device_ControlRequest(void);

/**
 * @brief	Mass Storage class driver callback function
 * @param	MSInterfaceInfo	:	Pointer to the Mass Storage class interface configuration structure
 * @return	true : On success
 * @note	false : On failure
 * Mass Storage class driver callback function for the reception of
 * SCSI commands from the host, which must be processed.
 */
bool CALLBACK_MS_Device_SCSICommandReceived(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo);

/**
 * @}
 */

#endif
