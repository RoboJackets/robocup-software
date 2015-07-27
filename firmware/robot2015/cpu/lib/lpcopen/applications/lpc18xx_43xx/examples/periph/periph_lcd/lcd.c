/*
 * @brief Simple LCD colorbar example with image and cursor
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
#include "Cursor.h"

/** @defgroup EXAMPLES_PERIPH_18XX43XX_LCD LPC18xx/43xx LCD (colorbars and cursor) example
 * @ingroup EXAMPLES_PERIPH_18XX43XX
 * <b>Example description</b><br>
 * This example shows how to configure the LCD. It renders colorbars, shows
 * an image, and allows control of a pointer.<br>
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
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define LCD_WIDTH       BOARD_LCD.PPL
#define LCD_HEIGHT      BOARD_LCD.LPP
#define LOGO_WIDTH      110
#define LOGO_HEIGHT     42

/* pointer to frame buffer */
static uint16_t *framebuffer = (uint16_t *) FRAMEBUFFER_ADDR;
static volatile uint32_t msec;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/* Image */
extern const unsigned short image[];

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Put a pixel at the x, y coordinate */
static void putpixel(uint32_t x, uint32_t y, uint16_t val) {
	framebuffer[x + y * LCD_WIDTH] = val;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	SysTick Interrupt Handler
 * @return	Nothing
 */
void SysTick_Handler(void)
{
	if (msec) {
		msec--;
	}
}

/**
 * @brief	Main entry point
 * @return	Nothing
 */
int main(void)
{
	uint32_t i, j;
	int cursor_x = 100, cursor_y = 150;
	int16_t tmp_x = -1, tmp_y = -1;

	Board_Init();
	Board_LCD_Init();

	SysTick_Config(SystemCoreClock / 1000);
	msec = 5;
	while (msec) {}

	/* Fill Colorbar only*/
	for (i = 0; i < LCD_WIDTH * LCD_HEIGHT / 4; i++)
		framebuffer[i] = 0x1F;
	for (i = LCD_WIDTH * LCD_HEIGHT / 4; i < LCD_WIDTH * LCD_HEIGHT * 2 / 4; i++)
		framebuffer[i] = 0x3F << 5;
	for (i = LCD_WIDTH * LCD_HEIGHT * 2 / 4; i < LCD_WIDTH * LCD_HEIGHT * 3 / 4; i++)
		framebuffer[i] = 0x1F << 11;
	for (i = LCD_WIDTH * LCD_HEIGHT * 3 / 4; i < LCD_WIDTH * LCD_HEIGHT; i++)
		framebuffer[i] = 0xFFFF;
	/* Fill NXP logo */
	for (j = 0; j < LOGO_HEIGHT; j++)
		for (i = 0; i < LOGO_WIDTH; i++)
			putpixel(i, j, image[(i + j * LOGO_WIDTH)]);

	Chip_LCD_Init(LPC_LCD, (LCD_Config_T *) &BOARD_LCD);

	Board_InitTouchController();
	Chip_LCD_SetUPFrameBuffer(LPC_LCD,  (void *) framebuffer);
	Chip_LCD_PowerOn(LPC_LCD);
	msec = 100;
	while (msec) {}

	Chip_LCD_Cursor_Disable(LPC_LCD, 0);
	Chip_LCD_Cursor_Config(LPC_LCD, LCD_CURSOR_32x32, true);
	Chip_LCD_Cursor_WriteImage(LPC_LCD, 0, (void *) Cursor);
	Chip_LCD_Cursor_SetClip(LPC_LCD, CURSOR_H_OFS, CURSOR_V_OFS);
	Chip_LCD_Cursor_SetPos(LPC_LCD, cursor_x, cursor_y);
	Chip_LCD_Cursor_Enable(LPC_LCD, 0);

	/* Turn on backlight */
	Board_SetLCDBacklight(1);

	msec = 20;
	while (msec) {}

	while (1) {
		Board_GetTouchPos((int16_t *) &tmp_x, (int16_t *) &tmp_y);
		if ((tmp_x >= 0) && (tmp_y >= 0)) {
			cursor_x = tmp_x;
			cursor_y = tmp_y;
		}

		if (LCD_WIDTH < cursor_x) {
			cursor_x = LCD_WIDTH - CURSOR_H_OFS;
		}

		if (LCD_HEIGHT < cursor_y) {
			cursor_y = (LCD_HEIGHT - CURSOR_V_OFS);
		}
		Chip_LCD_Cursor_SetPos(LPC_LCD, cursor_x, cursor_y);
	}
}

/**
 * @}
 */
