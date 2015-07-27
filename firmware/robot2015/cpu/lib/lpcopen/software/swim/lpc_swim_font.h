/*
 * @brief SWIM font management
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

#ifndef __LPC_SWIM_FONT_H_
#define __LPC_SWIM_FONT_H_

#include "lpc_types.h"
#include "lpc_swim.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/** @defgroup GUI_SWIM_FONT SWIM font manager
 * @ingroup GUI_SWIM
 * This package provides basic SWIM font management capabilities such as
 * font selection, text positioning, newline and window scrolling, and
 * text display with multiple, selectable fonts.
 * @{
 */

/**
 * @brief	Put text at x, y (char) position on screen
 * @param	win		: Pointer to window data structure
 * @param	text	: Text string to output in window
 * @param	x		: Virtual X position of start of text
 * @param	y		: Virtual Y position of start of text
 * @return	Nothing
 * @note	Sets the virtual (upper left) text position in the window and
 * render the text string at this position.
 */
void swim_put_text_xy(SWIM_WINDOW_T *win,
					  const CHAR *text,
					  int32_t x,
					  int32_t y);

/**
 * @brief	Sets the X, Y pixel coordinates for the next text operation
 * @param	win		: Pointer to window data structure
 * @param	x		: Virtual X position of start of text
 * @param	y		: Virtual Y position of start of text
 * @return	Nothing
 */
void swim_set_xy(SWIM_WINDOW_T *win,
				 int32_t x,
				 int32_t y);

/**
 * @brief	Returns the X, Y pixel coordinates for the next text operation
 * @param	win		: Pointer to window data structure
 * @param	x		: Address of where to return virtual X value
 * @param	y		: Address of where to return virtual X value
 * @return	Nothing
 * @note	The logical X and Y positions are computed by subtracting the
 * physical text position values by the physical minimum window
 * limits.
 */
void swim_get_xy(SWIM_WINDOW_T *win,
				 int32_t *x,
				 int32_t *y);

/**
 * @brief	Puts a string of text in a window
 * @param	win		: Pointer to window data structure
 * @param	text	: Text string to output in window
 * @return	Nothing
 * @note	Each character will be routed to the swim_put_char function until
 * a string terminator is reached. For newline characters, a newline
 * will occur instead of a character output.
 */
void swim_put_text(SWIM_WINDOW_T *win,
				   const CHAR *text);

/**
 * @brief	Puts a string of text in a window with breaks
 * @param	win		: Pointer to window data structure
 * @param	text	: Text string to output in window
 * @return	Nothing
 * @note	Puts a string of text in a window, but will adjust the position of
 * a word if the word length exceeds the edge of the display.
 */
void swim_put_ltext(SWIM_WINDOW_T *win,
					const CHAR *text);

/**
 * @brief	Scrolls the window up one line
 * @param	win		: Pointer to window data structure
 * @param	lines	: Number of lines to scroll up
 * @return	Nothing
 */
void swim_window_scroll(SWIM_WINDOW_T *win,
						int32_t lines);

/**
 * @brief	Puts a single character in the window
 * @param	win			: Pointer to window data structure
 * @param	textchar	: Character to print
 * @return	Nothing
 * @note	The character is placed at the end of the last text operation
 * or the current text X, Y position.
 */
void swim_put_char(SWIM_WINDOW_T *win,
				   const CHAR textchar);

/**
 * @brief	Puts a newline in the window
 * @param	win	: Pointer to window data structure
 * @return	Nothing
 */
void swim_put_newline(SWIM_WINDOW_T *win);

/**
 * @brief	Sets the active font
 * @param	win		: Pointer to window data structure
 * @param	font	: Pointer to font data structure to use for font
 * @return	Nothing
 */
void swim_set_font(SWIM_WINDOW_T *win,
				   FONT_T *font);

/**
 * @brief	Returns the active font's height in pixels
 * @param	win		: Pointer to window data structure
 * @return	The height of the current font in pixels
 */
int16_t swim_get_font_height(SWIM_WINDOW_T *win);

/**
 * @brief	Creates a title bar for the window
 * @param	win			: Pointer to window data structure
 * @param	title		: Text for title bard
 * @param	ttlbkcolor	: Totle bar backgorund color
 * @return	Nothing
 * @note	Creates a title bar in the window and adjusts the client
 * area to be outside the title bar area.
 */
void swim_set_title(SWIM_WINDOW_T *win,
					const CHAR *title,
					COLOR_T ttlbkcolor);

/**
 * @brief	Enables and disables font backgrounds
 * @param	win		: Pointer to window data structure
 * @param	trans	: 1 for transparent backgrounds, 0 for solid color
 * @return	Nothing
 * @note	Enables and disables font backgrounds. When set, the font background
 * will not be drawn in the background color (useful for painting text
 * over pictures).
 */
void swim_set_font_trasparency(SWIM_WINDOW_T *win, int32_t trans);

#if defined(__cplusplus)
}
#endif

/**
 * @}
 */

#endif /* __LPC_SWIM_FONT_H_ */
