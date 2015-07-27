/*
 * @brief Color bar examples for LCDs using SWIM
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
#include "lpc_swim.h"
#include "lpc_swim_font.h"
#include "lpc_rom8x16.h"
#include "lpc_winfreesystem14x16.h"
#include "lpc_x6x13.h"

/** @defgroup EXAMPLES_SWIM_18XX43XX_COLORBARS LPC18xx/43xx SWIM color bars
 * @ingroup EXAMPLES_SWIM_18XX43XX
 * <b>Example description</b><br>
 * This SWIM example draws some color bars on the display, animates a
 * simple ball, and shows the LCD VCOMP IRQ interrupt rate (number of
 * LCD refresh cycles).<br>
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

/* pointer to frame buffer */
static uint16_t *framebuffer = (uint16_t *) FRAMEBUFFER_ADDR;

/* Counts refresh cycles */
static volatile uint32_t frame_rate_counter;

#define FONT  font_x6x13
// #define FONT  font_rom8x16
// #define FONT  font_winfreesys14x16
#define DISPLAY_WIDTH  BOARD_LCD.PPL
#define DISPLAY_HEIGHT BOARD_LCD.LPP

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* @brief Draw color bars and animate ball */
static void lcd_colorbars(void)
{
	SWIM_WINDOW_T win1;
	COLOR_T clr, *fblog;
	CHAR str[32];
	short int idx;
	UNS_16 xgs, ygs = 0, curx, cury = 0, curym, xidx;
	int last = frame_rate_counter;
	int oldballx, oldbally, ballx, bally, balldx, balldy;

	/* Set LCD frame buffer address */
	fblog = (COLOR_T *) framebuffer;

	/* Create a SWIM window */
	swim_window_open(&win1, DISPLAY_WIDTH,
					 DISPLAY_HEIGHT, fblog, 0, 0,
					 (DISPLAY_WIDTH - 1),
					 (DISPLAY_HEIGHT - 1), 1, WHITE, BLACK, BLACK);

	/* Compute vertical size for 3 color bars */
	ygs = DISPLAY_HEIGHT / 3;

	/* Draw red bars */
	cury = 0;
	curx = 0;							/* Start cursor at X=0 cursor postion */
	curym = (ygs - 1);					/* End Cursor postion of Y */
	xgs = DISPLAY_WIDTH / RED_COLORS;	/* Divide pixels/line by # of red colors possible in this mode (32 shades red in RGB565) */
	clr = BLACK;						/* start with black then increase to full color across the line */
	for (xidx = 0; xidx < RED_COLORS; xidx++) {
		swim_set_pen_color(&win1, clr);
		for (idx = 0; idx <= xgs; idx++) {
			swim_put_line(&win1, curx, cury, curx, curym);	/* Draw line */
			curx++;
		}
		clr = clr + MINRED;				/* increment color value for a gradient,(RGB1:5:6:5) */
	}

	/* Draw green bars */
	cury = cury + ygs;					/* Start cursor postion of Y */
	curx = 0;							/* Start cursor postion of X */
	curym = cury + (ygs - 1);			/* End Cursor postion of Y  at 1/3 of panel */
	for (xidx = 0; xidx < GREEN_COLORS; xidx++) {
		swim_set_pen_color(&win1, clr);
		for (idx = 0; idx <= xgs; idx++) {

			swim_set_pen_color(&win1, clr);
			swim_put_line(&win1, curx, cury, curx, curym);
			curx++;
		}
		clr = clr + MINGREEN;
	}

	/* Draw blue bars */
	cury = cury + ygs;
	curx = 0;
	curym = cury + (ygs - 1);
	xgs = DISPLAY_WIDTH / BLUE_COLORS;	//
	clr = BLACK;
	for (xidx = 0; xidx < BLUE_COLORS; xidx++) {
		swim_set_pen_color(&win1, clr);
		for (idx = 0; idx <= xgs; idx++) {
			swim_put_line(&win1, curx, cury, curx, curym);
			curx++;
		}
		clr = clr + MINBLUE;		/* incement blue color value for a gradient, Blue=bits[4:0] in memory	(RGB1:5:6:5 mode) */
	}

	/* select the font to use */
	swim_set_font(&win1, (FONT_T *) &FONT);

	/* set the pen color to use */
	swim_set_pen_color(&win1, WHITE);

	/* Add a title bar */
	swim_set_title(&win1, "NXP SWIM Graphics Library", BLACK);

	/* set the location to write text */
	swim_set_xy(&win1, 60, 160);

	/* set the pen color to use */
	swim_set_pen_color(&win1, LIGHTGRAY);

	/* put the timer tick on the panel */
	oldballx = ballx = 0;
	oldbally = bally = 0;
	balldx = balldy = 9;

	while (1) {
		if (frame_rate_counter > last) {
			ballx += balldx;
			if (ballx >= win1.xvsize) {
				balldx *= -1, ballx += balldx;
			}
			if (ballx < 0) {
				balldx *= -1, ballx += balldx;
			}

			bally += balldy;
			if (bally >= win1.yvsize) {
				balldy *= -1, bally += balldy;
			}
			if (bally < 0) {
				balldy *= -1, bally += balldy;
			}

			swim_set_pen_color(&win1, BLACK);
			swim_set_fill_color(&win1, BLACK);
			swim_put_diamond(&win1, oldballx, oldbally, 7, 7);
			swim_set_pen_color(&win1, WHITE);
			swim_set_fill_color(&win1, RED);
			swim_put_diamond(&win1, ballx, bally, 7, 7);

			oldballx = ballx;
			oldbally = bally;

			swim_set_xy(&win1, 0, 0);
			swim_put_text(&win1, "Tick #");
			sprintf(str, "%d", frame_rate_counter);
			swim_put_text(&win1, str);
			swim_put_text(&win1, "\n");

			last = frame_rate_counter + 10;
		}
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	LCD VCOMP Interrupt Handler
 * @return	Nothing
 */
void LCD_IRQHandler(void)
{
	frame_rate_counter++;
	Chip_LCD_ClearInts(LPC_LCD, LCD_INTMSK_VCOMPIM);
}

/**
 * @brief	Main entry point
 * @return	Nothing
 */
int main(void)
{
	Board_Init();
	Board_LCD_Init();

	/* Initialize LCD, setup frame buffer, and enable vertical comp interrupt */
	Chip_LCD_Init(LPC_LCD, (LCD_Config_T *) &BOARD_LCD);
	Chip_LCD_SetUPFrameBuffer(LPC_LCD, (void *) framebuffer);
	Chip_LCD_PowerOn(LPC_LCD);
	Chip_LCD_EnableInts(LPC_LCD, LCD_INTMSK_VCOMPIM);

	NVIC_EnableIRQ(LCD_IRQn);

	/* Turn on backlight */
	Board_SetLCDBacklight(1);
	lcd_colorbars();

	while (1) {}
}

/**
 * @}
 */
