/*
 * @brief emWin with touch example
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

#include "GUI.h"
#include "board.h"

/** @defgroup EXAMPLES_EMWIN_18XX43XX_WTOUCH LPC18xx/43xx emWin with touch example
 * @ingroup EXAMPLES_EMWIN_18XX43XX
 * <b>Example description</b><br>
 * This example shows how to setup emWin with a simple GUI and use of the
 * touch screen. Prior to building this example, the emWin libraries need to
 * be built.<br>
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
 * @ref LPCOPEN_18XX_BOARD_KEIL1857<br>
 * @ref LPCOPEN_43XX_BOARD_KEIL4357<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* LCD width and height (from board LCD setup descriptor) */
#define LCD_WIDTH       BOARD_LCD.PPL
#define LCD_HEIGHT      BOARD_LCD.LPP

#if defined(__IAR_SYSTEMS_ICC__)
	#define LOCATE_ATX(x)      _Pragma(#x)
	#define LOCATE_ATXX(x)     LOCATE_ATX(location=x)
	#define LOCATE_AT(x)       LOCATE_ATXX(x)
#elif defined(__ARMCC_VERSION)
	#define LOCATE_AT(x)     __attribute__ ((at(x)))
#elif (defined(__CODE_RED))
	#define LOCATE_ATX(x)     __attribute__((section(".shmem_"#x ",\"aw\",%nobits@")))
	#define LOCATE_AT(x) LOCATE_ATX(x)
#elif (defined(__CODE_RED))
	#define LOCATE_AT(x) LOCATE_ATX(x)
#endif

static int16_t old_tmp_x = -1, old_tmp_y = -1;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/**
 * @brief GUI buffers required for emwin library
 */
#define GUI_BUF       LOCATE_AT(GUI_BUF_ADDR)
#define GUI_BUF_ADDR  0x28050000
#define GUI_NUMBYTES  ((1024 * 1024) * 2)
U32 GUI_Memory_Size = GUI_NUMBYTES;
U32 GUI_Block_Size = 0x128;
GUI_BUF U32 GUI_Memory[GUI_NUMBYTES / sizeof(U32)];

/**
 * Systick 1mS clock required for emWin time functions
 */
volatile uint32_t systick_timems;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* mSec delay */
static void lcdDelay(uint32_t delay)
{
	delay += systick_timems;
	while (systick_timems < delay) {}
}

/* Initialize the LCD for the current board */
static void lcdInit(void)
{
	/* Board specific LCD pre-setup */
	Board_LCD_Init();

	/* Setup for current board */
	Chip_LCD_Init(LPC_LCD, (LCD_Config_T *) &BOARD_LCD);
	Board_InitTouchController();
	Chip_LCD_SetUPFrameBuffer(LPC_LCD, (void *) FRAMEBUFFER_ADDR);
	Chip_LCD_PowerOn(LPC_LCD);
	lcdDelay(100);

	/* Turn on backlight */
	Board_SetLCDBacklight(1);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief		SysTick handler sub-routine (1ms)
 * @return      Nothing
 */
void SysTick_Handler(void)						// SysTick Interrupt Handler @ 1000Hz
{
	static uint8_t ttick = 0;			// Touch Screen time tick
	int16_t tmp_x = -1, tmp_y = -1;
	int16_t tmp_x1 = -1, tmp_y1 = -1;
	systick_timems++;						// Increment 1ms time tick

	if (ttick++ == 20) {			// Touch Screen update period is 20ms
		ttick = 0;
		Board_GetTouchPos((int16_t *) &tmp_x, (int16_t *) &tmp_y);
		if ((tmp_x >= 0) && (tmp_y > 0) && ((tmp_x != old_tmp_x) || (tmp_y != old_tmp_y))) {
			tmp_x1 = tmp_y;
			tmp_y1 = tmp_x;
			GUI_TOUCH_StoreState(320 - tmp_x1, tmp_y1);
			old_tmp_x = tmp_x;
			old_tmp_y = tmp_y;
		}
		else {
			GUI_TOUCH_StoreState(-1, -1);
		}
	}
}

extern void MainTask(void);

/**
 * @brief		Main function
 * @return      Nothing
 */
int main(void)
{
	Board_Init();

	/* sysTick will handle touch events at 1KHz */
	SysTick_Config(Chip_Clock_GetRate(CLK_MX_MXCORE) / 1000);

	/* Setup LCD */
	lcdInit();

	/* Main Task */
	MainTask();

	return 0;
}

/**
 * @}
 */
