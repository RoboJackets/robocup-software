/*
 * @brief USB Mass Storage Device Example
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
 * Copyright(C) Dean Camera, 2011, 2012
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

#include "MassStorage.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#ifdef CFG_SDCARD
/* SDMMC card info structure */
static mci_card_struct sdcardinfo;
static volatile int32_t sdio_wait_exit = 0;
#endif

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
		.PortNumber = 0,
	},
};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

#ifdef CFG_SDCARD

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

/*  Sets up the SD event driven wakeup */
static void sdmmc_setup_wakeup(uint32_t bits)
{
	/* Wait for IRQ - for an RTOS, you would pend on an event here with a IRQ based wakeup. */
	NVIC_ClearPendingIRQ(SDIO_IRQn);
	sdio_wait_exit = 0;
	Chip_SDIF_SetIntMask(LPC_SDMMC, bits);
	NVIC_EnableIRQ(SDIO_IRQn);
}

/* A better wait callback for SDMMC driven by the IRQ flag */
static uint32_t sdmmc_irq_driven_wait(void)
{
	uint32_t status;

	/* Wait for event, would be nice to have a timeout, but keep it  simple */
	while (sdio_wait_exit == 0) {}

	/* Get status and clear interrupts */
	status = Chip_SDIF_GetIntStatus(LPC_SDMMC);

	return status;
}

#endif


/* HW set up function */
static void SetupHardware(void)
{
	Board_Init();

#ifdef CFG_SDCARD
	memset(&sdcardinfo, 0, sizeof(sdcardinfo));
	sdcardinfo.evsetup_cb = sdmmc_setup_wakeup;
	sdcardinfo.waitfunc_cb = sdmmc_irq_driven_wait;
	sdcardinfo.msdelay_func = sdmmc_waitms;

	/*  SD/MMC initialization */
	Board_SDMMC_Init();

	/* The SDIO driver needs to know the SDIO clock rate */
	Chip_SDIF_Init(LPC_SDMMC);

	/* Wait for a card to be inserted */
#ifndef BOARD_NGX_XPLORER_18304330
	/* NGX board ignored SD_CD pin */
	/* Wait for a card to be inserted (note CD is not on the SDMMC power rail and can be polled without enabling SD slot power */
	while (Chip_SDIF_CardNDetect(LPC_SDMMC)) {}
#endif

	/* Acquire the card once ready */
	if (!Chip_SDMMC_Acquire(LPC_SDMMC, &sdcardinfo)) {
		DEBUGOUT("Card Acquire failed...\r\n");
		while (1) {}
	}
#endif

	USB_Init(Disk_MS_Interface.Config.PortNumber, USB_MODE_Device);
#if defined(USB_DEVICE_ROM_DRIVER)
	UsbdMsc_Init();
#endif
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

#ifdef CFG_SDCARD

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

/**
 *  @brief  Main program entry point
 *  @return Will never return
 *  @note   This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();

	for (;; ) {
		#if !defined(USB_DEVICE_ROM_DRIVER)
		MS_Device_USBTask(&Disk_MS_Interface);
		USB_USBTask(Disk_MS_Interface.Config.PortNumber, USB_MODE_Device);
		#endif
	}
}

/**
 * @brief  Event handler for the library USB Connection event
 * @return Nothing
 */
void EVENT_USB_Device_Connect(void)
{}

/**
 * @brief Event handler for the library USB Disconnection event
 * @return Nothing
 */
void EVENT_USB_Device_Disconnect(void)
{}

/**
 * @brief Event handler for the library USB Configuration Changed event
 * @return Nothing
 */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= MS_Device_ConfigureEndpoints(&Disk_MS_Interface);
}

/**
 * @brief Event handler for the library USB Control Request reception event
 * @return	Nothing
 */
void EVENT_USB_Device_ControlRequest(void)
{
	MS_Device_ProcessControlRequest(&Disk_MS_Interface);
}

/**
 * @brief Mass Storage class driver callback function
 * @return Nothing
 * @note   The reception of SCSI commands from the host, which must be processed
 */
bool CALLBACK_MS_Device_SCSICommandReceived(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo)
{
	bool CommandSuccess;

	CommandSuccess = SCSI_DecodeSCSICommand(MSInterfaceInfo);
	return CommandSuccess;
}
