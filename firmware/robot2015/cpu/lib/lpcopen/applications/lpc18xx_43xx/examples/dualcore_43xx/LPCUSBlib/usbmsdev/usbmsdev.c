/*
 * @brief	USB Mass Storage Device Dual core example
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

#include "usbmsdev.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/** nxpUSBlib Mass Storage Class driver interface configuration and state information. This structure is
 *  passed to all Mass Storage Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_MS_Device_t Disk_MS_Interface = {
	.Config = {
		.InterfaceNumber           = 0,

		.DataINEndpointNumber      = MASS_STORAGE_IN_EPNUM,
		.DataINEndpointSize        = MASS_STORAGE_IO_EPSIZE,
		.DataINEndpointDoubleBank  = false,

		.DataOUTEndpointNumber     = MASS_STORAGE_OUT_EPNUM,
		.DataOUTEndpointSize       = MASS_STORAGE_IO_EPSIZE,
		.DataOUTEndpointDoubleBank = false,

		.TotalLUNs                 = TOTAL_LUNS,
		.PortNumber = 0,
	},
};


/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/
#ifdef OS_FREE_RTOS

void usb_msdev_func(void);
static xSemaphoreHandle usb_dev_event;

/* Task that waits for event from USB device, invokes the USBTask
 * whenever it receives one
 */
static void usb_msdev_task(void *arg)
{
	vSemaphoreCreateBinary(usb_dev_event);
	if (usb_dev_event == NULL) {
		DEBUGSTR("Unable to create semaphore!\r\n");
		while (1) ;
	}
	while (1) {
		xSemaphoreTake(usb_dev_event, 100);
		usb_msdev_func();
	}
}

/**
 * @brief Mass Storage class device stack transfer complete callback
 * @param	logicalEP	: Logical Endpoint number causing the event
 * @param	xfer_in	: 1 if event is IN, 0 when event is OUT
 * @return	None
 * @note 	Mass Storage class driver callback function for the reception or
 * completion of any transfers.
 * @return      None
 */
void EVENT_USB_Device_TransferComplete(int logicalEP, int xfer_in)
{
	portBASE_TYPE wake = pdFALSE;
	if (usb_dev_event) {
		xSemaphoreGiveFromISR(usb_dev_event, &wake);
	}
	portEND_SWITCHING_ISR(wake);
}

/* FreeRTOS USB device task */
void usb_device_tasks(void)
{
	/* Start Blinky event Task */
	xTaskCreate(usb_msdev_task, (signed char *) "USB Task",
				configMINIMAL_STACK_SIZE + 128, NULL, TASK_PRIO_USBDEVICE,
				(xTaskHandle *) NULL);
}
#elif defined(OS_UCOS_III)

#define USBMSDEV_STACK_SZ         (1024)
#define USBMSDEV_STACK_SZ_FULL    90u
#define USBMSDEV_STACK_SZ_LIMIT   (USBMSDEV_STACK_SZ * (100u - USBMSDEV_STACK_SZ_FULL))   / 100u

void usb_msdev_func(void);
static OS_TCB    mem_tcb;
static CPU_STK   mem_stack[USBMSDEV_STACK_SZ];

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

/**
 * @brief Mass Storage class device stack transfer complete callback
 * @param	logicalEP	: Logical Endpoint number causing the event
 * @param	xfer_in	: 1 if event is IN, 0 when event is OUT
 * @return	None
 * @note    Mass Storage class driver callback function for the reception or
 * completion of any transfers.
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

#else /* No OS */
#define usb_msdev_func usb_device_tasks
#endif

#ifdef CFG_SDCARD
static volatile int32_t sdio_wait_exit = 0;

/* Delay callback for timed SDIF/SDMMC functions */
static void sdmmc_waitms(uint32_t time)
{
	int32_t curr = (int32_t) Chip_RIT_GetCounter(LPC_RITIMER);
	int32_t final = curr + ((SystemCoreClock / 1000) * time);

	/* If the value is zero let us not worry about it */
	if (!time) {
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
 * @brief	Sets up the SD event driven wakeup
 * @param	bits : Status bits to poll for command completion
 * @return	Nothing
 */
static void sdmmc_setup_wakeup(uint32_t bits)
{
	/* Wait for IRQ - for an RTOS, you would pend on an event here with a IRQ based wakeup. */
	NVIC_ClearPendingIRQ(SDIO_IRQn);
	sdio_wait_exit = 0;
	Chip_SDMMC_SetIntMask(LPC_SDMMC, bits);
	NVIC_EnableIRQ(SDIO_IRQn);
}

/**
 * @brief	A better wait callback for SDMMC driven by the IRQ flag
 * @param	bits : Status bits to poll for command completion
 * @return	0 on success, or failure condition (-1)
 */
static uint32_t sdmmc_irq_driven_wait(void)
{
	uint32_t status;

	/* Wait for event, would be nice to have a timeout, but keep it  simple */
	while (sdio_wait_exit == 0) {}

	/* Get status and clear interrupts */
	status = Chip_SDMMC_GetIntStatus(LPC_SDMMC);

	return status;
}

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
#endif

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* USB device initialization */
void USBDEV_Init(void)
{
#ifdef CFG_SDCARD
	memset(&sdcardinfo, 0, sizeof(sdcardinfo));
	sdcardinfo.evsetup_cb = sdmmc_setup_wakeup;
	sdcardinfo.waitfunc_cb = sdmmc_irq_driven_wait;
	sdcardinfo.msdelay_func = sdmmc_waitms;

	/*  SD/MMC initialization */
	Board_SDMMC_Init();

	/* The SDIO driver needs to know the SDIO clock rate */
	Chip_SDMMC_Init(LPC_SDMMC);

	/* Wait for a card to be inserted */
#ifndef BOARD_NGX_XPLORER_18304330
	/* NGX board ignored SD_CD pin */
	/* Wait for a card to be inserted (note CD is not on the SDMMC power rail and can be polled without enabling SD slot power */
	while (Chip_SDMMC_CardNDetect(LPC_SDMMC) == 0) {}
#endif

	/* Acquire the card once ready */
	if (!Chip_SDMMC_Acquire(LPC_SDMMC, &sdcardinfo)) {
		DEBUGOUT("Card Acquire failed...\r\n");
		while(1) {};
	}
#endif

	USB_Init(Disk_MS_Interface.Config.PortNumber, USB_MODE_Device);
#if defined(USB_DEVICE_ROM_DRIVER)
	UsbdMsc_Init();
#endif
	NVIC_SetPriority(USB0_IRQn, IRQ_PRIO_USBDEV);
}

/* USB Device task */
void usb_msdev_func(void)
{
	#if !defined(USB_DEVICE_ROM_DRIVER)
	MS_Device_USBTask(&Disk_MS_Interface);
	USB_USBTask(Disk_MS_Interface.Config.PortNumber,USB_MODE_Device);
	#endif
}
/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= MS_Device_ConfigureEndpoints(&Disk_MS_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	MS_Device_ProcessControlRequest(&Disk_MS_Interface);
}

/** Mass Storage class driver callback function the reception of SCSI commands from the host, which must be processed.
 */
bool CALLBACK_MS_Device_SCSICommandReceived(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo)
{
	bool CommandSuccess;

	CommandSuccess = SCSI_DecodeSCSICommand(MSInterfaceInfo);
	return CommandSuccess;
}
