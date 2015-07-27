/*
 * @brief	USB Mass Storage device task implementation module uCOS-III version
 *
 * Main source file for the MassStorage demo. This file contains the main tasks of
 * the demo and is responsible for the initial application hardware configuration.
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

#include "lpc43xx_dualcore_config.h"
#include "MassStorage.h"

/* FreeRTOS headers */
#include "os.h"
#include "cpu.h"
#include "os_cfg_app.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define USBMSDEV_STACK_SZ         (1024)
#define USBMSDEV_STACK_SZ_FULL    90u
#define USBMSDEV_STACK_SZ_LIMIT   (USBMSDEV_STACK_SZ * (100u - USBMSDEV_STACK_SZ_FULL))   / 100u

static OS_TCB    mem_tcb;
static CPU_STK   mem_stack[USBMSDEV_STACK_SZ];

/** nxpUSBlib Mass Storage Class driver interface configuration and state information. This structure is
 *  passed to all Mass Storage Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
static USB_ClassInfo_MS_Device_t Disk_MS_Interface = {
	.Config = {
		.InterfaceNumber           = 0,

		.DataINEndpointNumber      = MASS_STORAGE_IN_EPNUM,
		.DataINEndpointSize        = MASS_STORAGE_IO_EPSIZE,
		.DataINEndpointDoubleBank  = false,

		.DataOUTEndpointNumber     = MASS_STORAGE_OUT_EPNUM,
		.DataOUTEndpointSize       = MASS_STORAGE_IO_EPSIZE,
		.DataOUTEndpointDoubleBank = false,

		.TotalLUNs                 = TOTAL_LUNS,
	},
};

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
static void usb_msdev_func(void)
{
	MS_Device_USBTask(&Disk_MS_Interface);
	USB_USBTask();
}

/* Task that waits for event from USB device, invokes the USBTask
 * whenever it receives one
 */
static void usb_msdev_task(void *arg)
{
	OS_ERR	ucErr;
	CPU_TS  ucos_timeout;
	
	while (1) {
		OSTaskSemPend(100, OS_OPT_PEND_BLOCKING, &ucos_timeout, &ucErr);
		usb_msdev_func();
	}
}

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Initialize USB device stack function */
void USBDEV_Init(void)
{
	USB_Init();
	NVIC_SetPriority(USB0_IRQn, IRQ_PRIO_USBDEV);
}

/* USB Device connect event callback function */
void EVENT_USB_Device_Connect(void)
{}

/* USB Device disconnect event callback function */
void EVENT_USB_Device_Disconnect(void)
{}

/* USB Device configuration change event callback function */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= MS_Device_ConfigureEndpoints(&Disk_MS_Interface);
}

/* USB Device control request receive event callback function */
void EVENT_USB_Device_ControlRequest(void)
{
	MS_Device_ProcessControlRequest(&Disk_MS_Interface);
}

/* Mass Storage class driver callback function */
bool CALLBACK_MS_Device_SCSICommandReceived(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo)
{
	bool CommandSuccess;

	CommandSuccess = SCSI_DecodeSCSICommand(MSInterfaceInfo);
	return CommandSuccess;
}

/**
 * @brief Mass Storage class device stack transfer complete callback
 * @param	logicalEP	: Logical Endpoint number causing the event
 * @param	xfer_in	: 1 if event is IN, 0 when event is OUT
 * @return	None
 * Mass Storage class driver callback function for the reception or
 * completion of any transfers.
 * @param[in]	logicalEP Logical Endpoint number causing the event
 * @param[in]	xfer_in 1 if event is IN, 0 when event is OUT
 * @return      None
 */
void EVENT_USB_Device_TransferComplete(int logicalEP, int xfer_in)
{
	OS_ERR	ucErr;
	OSTaskSemPost(&mem_tcb, OS_OPT_POST_NONE, &ucErr);
	if (ucErr != OS_ERR_NONE) {
		DEBUGSTR("OSTaskSemPost failure!\r\n");
		//while (1);
	}
	
}

/* uCOS-III USB device task */
void usb_device_tasks(void)
{
	OS_ERR ret;
	
	/* Start USB event Task */
	OSTaskCreate   (
		&mem_tcb,
		"USB Task",
		usb_msdev_task,
		(void *) 1,
		TASK_PRIO_USBDEVICE,
		mem_stack,
		//USBMSDEV_STACK_SZ_LIMIT,
		32,
		USBMSDEV_STACK_SZ,
		0,
		0,
		(void *)0,
		(OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
		(OS_ERR *)&ret);
	if (ret != OS_ERR_NONE) {
		DEBUGSTR("Unable to create USB task!\r\n");
		while (1);
	}
}
