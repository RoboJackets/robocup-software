/*
 * @brief M0 Image loader module
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
#include "lpc43xx_dualcore_config.h"

/** @defgroup EXAMPLE_DUALCORE_CMN_M0LOADER LPC43xx M0 Image loader
 * @ingroup EXAMPLES_DUALCORE_43XX_COMMON
 * <b>Example description</b><br>
 * The M0 image loader is a common M0 bootloader that is used with all the
 * dual-core examples. The image loader is used by the M4 core to M0 boot
 * image is valid prior to starting the M0 core.<br>
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

/* Magic numbers to identify M0/M4 images */
#define SIGNATURE_M4_MAGIC     0xF00D4BAD
#define SIGNATURE_M0_MAGIC     0xBEEFF00D

#ifndef ERROR_LED
/* LED to be blinked when there is an error in loading/starting M0 image */
#define ERROR_LED    1
#endif

#define EX_BLINKY          (1 << 0)
#define EX_USBHOST         (1 << 1)
#define EX_USBDEV          (1 << 2)
#define EX_LWIP            (1 << 3)
#define EX_EMWIN           (1 << 4)

#ifdef OS_UCOS_III
#define OS_SIGNATURE        3
#elif defined(OS_FREE_RTOS)
#define OS_SIGNATURE        2
#else
#define OS_SIGNATURE        1
#endif

#ifndef EXAMPLE_BLINKY
#define EXAMPLE_BLINKY      0
#else
#undef  EXAMPLE_BLINKY
#define EXAMPLE_BLINKY      EX_BLINKY
#endif

#ifndef EXAMPLE_USB_HOST
#define EXAMPLE_USB_HOST    0
#else
#undef  EXAMPLE_USB_HOST
#define EXAMPLE_USB_HOST    EX_USBHOST
#endif

#ifndef EXAMPLE_USB_DEVICE
#define EXAMPLE_USB_DEVICE  0
#else
#undef  EXAMPLE_USB_DEVICE
#define EXAMPLE_USB_DEVICE  EX_USBDEV
#endif

#ifndef EXAMPLE_LWIP
#define EXAMPLE_LWIP        0
#else
#undef  EXAMPLE_LWIP
#define EXAMPLE_LWIP        EX_LWIP
#endif

#ifndef EXAMPLE_EMWIN
#define EXAMPLE_EMWIN       0
#else
#undef  EXAMPLE_EMWIN
#define EXAMPLE_EMWIN       EX_EMWIN
#endif

#define EXAMPLES_INCLUDED (EXAMPLE_BLINKY + EXAMPLE_LWIP + EXAMPLE_USB_HOST + \
	EXAMPLE_USB_DEVICE + EXAMPLE_EMWIN)

/* Image signature structure */
/* NOTE: Although the M0 structure created using this type is used in
 * the startup assembly file, the assembly file just needs the symbol and
 * not the type, hence keeping it as private structure.
 */
struct image_sig {
	uint32_t signature;		/* Signature M0/M4 image */
	uint32_t capability;	/* Examples included */
	uint32_t os;			/* OS used */
	char build_date[32];	/* Build Date string */
	char build_time[32];	/* Build Time string */
};

#ifdef CORE_M4
/* M4 Image Structure */
static const struct image_sig __M4Signature = {
	SIGNATURE_M4_MAGIC,	/* M4 Image magic signature */
	EXAMPLES_INCLUDED,
	OS_SIGNATURE,
	__DATE__,
	__TIME__,
};
#endif

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
#ifdef CORE_M0
/* Structure object that identifies the image as M0 image */
/* NOTE: This symbol is used in startup file (only) */
const struct image_sig __M0Signature = {
	SIGNATURE_M0_MAGIC,	/* M0 Image magic signature */
	EXAMPLES_INCLUDED,
	OS_SIGNATURE,
	__DATE__,
	__TIME__,
};
#endif

/*****************************************************************************
 * Private functions
 ****************************************************************************/

#ifdef CORE_M4
/* Function to blink the LED to show the error code
 * caused by M0 image boot failure
 */
static void booting_m0_failure(uint32_t msec)
{
	int32_t cnt = 60000 / (msec * 2);
	DEBUGSTR("ERROR: Boot failure!!\r\n");
	while (cnt--) {
		Board_LED_Set(ERROR_LED, 1);
		MSleep(msec);
		Board_LED_Set(ERROR_LED, 0);
		MSleep(msec);
	}
}

/* Prints the information of the M0/M4 image to screen */
static void print_image_info(const char *pre, const struct image_sig *img)
{
	DEBUGSTR("***************************************\r\n");
	DEBUGOUT("%s: Header found at %p\r\n", pre, img);
	DEBUGOUT("%s: Included Examples: %s%s%s%s%s\r\n", pre,
			 img->capability & EX_BLINKY  ? "BLINKY" : "",
			 img->capability & EX_USBHOST ? ", USB_Host" : "",
			 img->capability & EX_USBDEV  ? ", USB_Device" : "",
			 img->capability & EX_LWIP    ? ", lwIP" : "",
			 img->capability & EX_EMWIN   ? ", emWin" : "");
	DEBUGOUT("%s: OS Used: %s\r\n", pre,
			 img->os == 1 ? "NONE (STANDALONE)" :
			 (img->os == 2 ? "FreeRTOS" : "UCOS-III"));
	DEBUGOUT("%s: Built on %s %s\r\n", pre,
			 img->build_date,
			 img->build_time);
	DEBUGSTR("***************************************\r\n");
}

/* Validates the M0/M4 image at address image_addr */
static int CheckImages(uint32_t image_addr, const struct image_sig *m4)
{
	const uint32_t *addr = (uint32_t *) image_addr + 8;
	const struct image_sig *m0;

	uint32_t val, usbprob;

	/* Check for validity of M0 Image */
	if (*addr != 0xAA55DEAD) {
		DEBUGOUT("ERROR: Unable to find signature1 of M0 Image at %p\r\n",
				 ((uint32_t *) image_addr + 4));
		booting_m0_failure(20);
		return -1;
	}

	/* Do sanity check */
	addr++;
	if ((image_addr & 0xFFF00000UL) != (*addr & 0xFFF00000UL)) {
		DEBUGOUT("ERROR: M0 Image at 0x%08X, Infostruct at "
				 "0x%08X not is same region\r\n", image_addr, *addr);
		booting_m0_failure(20);
		return -1;
	}

	/* Valid structure is found */
	m0 = (const struct image_sig *) *addr;
	if (m0->signature != SIGNATURE_M0_MAGIC) {
		DEBUGSTR("M0_IMAGE: ERROR: M0 image signature 2 not found!\r\n");
		booting_m0_failure(20);
		return -1;
	}

	/* This should never happen, kept only for completeness */
	if (m4->signature != SIGNATURE_M4_MAGIC) {
		DEBUGSTR("M0_IMAGE: ERROR: M4 image signature 2 not found!\r\n");
		booting_m0_failure(10000);
		return -1;
	}
	print_image_info("M0_IMAGE", m0);
	print_image_info("M4_IMAGE", m4);
	val = m0->capability & m4->capability;
	usbprob = (m0->capability & (EX_USBHOST | EX_USBDEV)) &&
			  (m4->capability & (EX_USBHOST | EX_USBDEV));

	/* TODO: Check on possibility of running device on one core
	 * and host stack on another
	 **/
	if (usbprob) {
		DEBUGSTR("ERROR: Running USB Host/Device stack on both"
				 " cores is not supported yet!\r\n");
		booting_m0_failure(2000);
		return -1;
	}

	if (val & EX_LWIP) {
		DEBUGSTR("ERROR: Running lwIP on both core is not supported!\r\n");
		booting_m0_failure(2000);
		return -1;
	}

	if (val & EX_EMWIN) {
		DEBUGSTR("ERROR: Running emWIN on both core is not supported!\r\n");
		booting_m0_failure(2000);
		return -1;
	}
	return 0;
}

#endif

/*****************************************************************************
 * Public functions
 ****************************************************************************/

#ifdef CORE_M4
/* M0 Boot loader */
int M0Image_Boot(uint32_t m0_image_addr)
{
	/* Make sure the alignment is OK */
	if (m0_image_addr & 0xFFF) {
		return -1;
	}

	/* Check the validity of images */
	if (CheckImages(m0_image_addr, &__M4Signature) != 0) {
		return -1;
	}

	/* Make sure the M0 core is being held in reset via the RGU */
	Chip_RGU_TriggerReset(RGU_M0APP_RST);

	Chip_Clock_Enable(CLK_M4_M0APP);

	/* Keep in mind the M0 image must be aligned on a 4K boundary */
	Chip_CREG_SetM0AppMemMap(m0_image_addr);

	Chip_RGU_ClearReset(RGU_M0APP_RST);

	return 0;
}

#endif

/**
 * @}
 */
