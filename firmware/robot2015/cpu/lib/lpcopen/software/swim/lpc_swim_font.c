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

#include "lpc_swim_font.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Determines the length of the word (in pixels) up to the first
   whitespace character */
static int16_t swim_get_word_len(SWIM_WINDOW_T *win,
								const CHAR *text)
{
	int32_t i;
	int16_t wlen = 0;

	/* Find the length in pixels of the next word (separated by
	   whitespace) */
	i = 0;
	while (((uint8_t) text[i] > ' ') && ((uint8_t) text[i] <= 0x7E)) {
		wlen = wlen + win->font->font_width_table
			   [(uint8_t) text[i] - win->font->first_char];
		i++;
	}

	return wlen;
}

/* Puts a word in the window, but adds a newline to keep the
   word contiguous (without an edge break) if necessary */
static int32_t swim_put_word(SWIM_WINDOW_T *win,
							const CHAR *text)
{
	int32_t i;

	/* Will the length of the next word exceed the window margin? */
	if ((swim_get_word_len(win, text) + win->xvpos) > win->xpvmax) {
		/* Do a newline */
		swim_put_newline(win);
	}

	/* Put just the word characters on the display up to the next
	   non-whitespace character or the end of the string */
	i = 0;
	while (((uint8_t) text[i] > ' ') && ((uint8_t) text[i] <= 0x7E)) {
		swim_put_char(win, text[i]);
		i++;
	}

	return i;
}

/***********************************************************************
 * Public functions
 **********************************************************************/

/* Put text at x, y (char) position on screen */
void swim_put_text_xy(SWIM_WINDOW_T *win,
					  const CHAR *text,
					  int32_t x,
					  int32_t y)
{
	/* Convert virtual x, y positon to physical position */
	swim_set_xy(win, x, y);

	/* Display text string */
	swim_put_text(win, text);
}

/* Sets the X, Y pixel coordinates for the next text operation */
void swim_set_xy(SWIM_WINDOW_T *win,
				 int32_t x,
				 int32_t y)
{
	win->xvpos = x + win->xpvmin;
	win->yvpos = y + win->ypvmin;

	/* Limit to window dimensions */
	if (win->xvpos < win->xpvmin) {
		win->xvpos = win->xpvmin;
	}
	else if (win->xvpos > win->xpvmax) {
		win->xvpos = win->xpvmax;
	}

	if (win->yvpos < win->ypvmin) {
		win->yvpos = win->ypvmin;
	}
	else if (win->yvpos > win->ypvmax) {
		win->yvpos = win->ypvmax;
	}
}

/* Returns the X, Y pixel coordinates for the next text operation */
void swim_get_xy(SWIM_WINDOW_T *win,
				 int32_t *x,
				 int32_t *y)
{
	*x = win->xvpos - win->xpvmin;
	*y = win->yvpos - win->ypvmin;
}

/* Puts a string of text in a window */
void swim_put_text(SWIM_WINDOW_T *win,
				   const CHAR *text)
{
	int32_t i = 0;

	/* Continue until the entire string is output */
	while (text[i] != '\0') {
		if (text[i] == '\n') {
			swim_put_newline(win);
		}
		else if (((uint8_t) text[i] >= win->font->first_char)
				 && ((uint8_t) text[i] <= win->font->last_char)) {
			/* Put character on screen */
			swim_put_char(win, text[i]);
		}

		i++;
	}
}

/* Puts a string of text in a window with breaks */
void swim_put_ltext(SWIM_WINDOW_T *win,
					const CHAR *text)
{
	int32_t i = 0;

	/* Continue until the entire string is output */
	while (text[i] != '\0') {
		if (text[i] == '\n') {
			swim_put_newline(win);
			i++;
		}
		else if (((uint8_t) text[i] >= win->font->first_char)
				 && ((uint8_t) text[i] <= win->font->last_char)) {
			/* Check for entire words first */
			if (((uint8_t) text[i] > ' ') && ((uint8_t) text[i] <= 0x7E)) {
				/* Put entire word on screen */
				i = i + swim_put_word(win, &text[i]);
			}
			else {
				swim_put_char(win, text[i]);
				i++;
			}
		}
		else {
			/* Put a space out */
			swim_put_char(win, ' ');
			i++;
		}
	}
}

/* xx */
void swim_window_scroll(SWIM_WINDOW_T *win,
						int32_t lines)
{
	COLOR_T *fb1, *fb2;
	int32_t yref1 = win->ypvmin;
	int32_t yref2 = yref1 + lines;
	int32_t ref;

	while (yref2 <= win->ypvmax) {
		/* Line move addresses */
		fb1 = win->fb + win->xpvmin + (yref1 * win->xpsize);
		fb2 = win->fb + win->xpvmin + (yref2 * win->xpsize);

		/* Move a single line at a time */
		ref = win->xpvmax - win->xpvmin + 1;
		while (ref > 0) {
			*fb1++ = *fb2++;
			ref--;
		}

		/* Next lines */
		yref1++;
		yref2++;
	}

	/* Clear out bottom lines */
	yref1 = win->yvpos;
	while (yref1 <= win->ypvmax) {
		/* Line clear address */
		fb1 = win->fb + win->xpvmin + (yref1 * win->xpsize);

		/* Clear a single line at a time */
		ref = win->xpvmax - win->xpvmin + 1;
		while (ref > 0) {
			*fb1++ = win->bkg;
			ref--;
		}

		yref1++;
	}
}

/* Puts a single character in the window */
void swim_put_char(SWIM_WINDOW_T *win,
				   const CHAR textchar)
{
	int32_t i, j;
	int32_t charindex;
	uint16_t *charfields, chardata;
	COLOR_T *fb;

	/* If this is a carriage return, do a newline */
	if (textchar == '\n') {
		swim_put_newline(win);
	}
	else {
		/* Determine index to character data */
		charindex = (int32_t) textchar - (int32_t) win->font->first_char;

		/* Will the character fit on the display? */
		if ((win->xvpos +
			 (int32_t) win->font->font_width_table[charindex]) >
			win->xpvmax) {
			/* Will not fit, do a newline */
			swim_put_newline(win);
		}

		/* Determine the start of the bitfields for the character */
		charfields = win->font->font_table + (charindex *
											  win->font->font_height);

		/* Map character to the window */
		for (i = 0; i < (int32_t) win->font->font_height; i++) {
			/* Get starting pixel location for font mapping in window */
			fb = win->fb + win->xvpos + ((win->yvpos + i) * win->xpsize);

			/* Get character line mapping data */
			chardata = charfields[i];

			/* Convert character line bit data to a pixel line in
			   window */
			for (j =
					 (int32_t) win->font->font_width_table[charindex];
				 j > 0; j--) {
				if ((chardata & 0x8000) != 0) {
					*fb++ = win->pen;
				}
				else if (win->tfont != 0) {
					*fb++ = win->bkg;
				}
				else {
					fb++;
				}

				/* Next bit in character line */
				chardata = chardata << 1;
			}
		}

		/* Increment to next text location */
		win->xvpos = win->xvpos +
					 (int32_t) win->font->font_width_table[charindex];
	}
}

/* Puts a newline in the window */
void swim_put_newline(SWIM_WINDOW_T *win)
{
	int32_t diff;

	/* Set text pointer to start of next line */
	win->xvpos = win->xpvmin;
	win->yvpos = win->yvpos + (int32_t) win->font->font_height;

	/* Next character is below bottom of window, scroll the window
	   up */
	while ((win->yvpos +
			(int32_t) win->font->font_height) > win->ypvmax) {
		/* Scroll just enough for the next line */
		diff = (int32_t) win->font->font_height -
			   (win->ypvmax - win->yvpos);
		win->yvpos = win->yvpos - diff;
		swim_window_scroll(win, diff);
	}
}

/* Sets the active font */
void swim_set_font(SWIM_WINDOW_T *win,
				   FONT_T *font)
{
	int32_t diff;

	win->font = font;

	/* After changing to the new font, determine if there is enough
	   room for the font height on the existing line in the window */
	if ((win->yvpos + win->font->font_height) > win->ypvmax) {
		diff = (int32_t) win->font->font_height -
			   (win->ypvmax - win->yvpos);
		win->yvpos = win->yvpos - diff;
		swim_window_scroll(win, diff);
	}
}

/* Returns the active font's height in pixels */
int16_t swim_get_font_height(SWIM_WINDOW_T *win)
{
	return win->font->font_height;
}

/* Creates a title bar for the window */
void swim_set_title(SWIM_WINDOW_T *win,
					const CHAR *title,
					COLOR_T ttlbkcolor)
{
	COLOR_T savedf, savedp, savedb;
	int32_t savedt;

	/* Is present font height larger than window client height? */
	if ((swim_get_font_height(win) < (4 + win->yvsize)) &&
		(title != (CHAR *) 0)) {
		/* There is enough room for title bar, so continue */

		/* Save original colors and font transparentcy flag */
		savedf = win->fill;
		savedp = win->pen;
		savedb = win->bkg;
		savedt = win->tfont;

		/* Set fill color to background color (temporarily)
		   used with box function */
		win->fill = ttlbkcolor;
		win->bkg = ttlbkcolor;
		win->pen = win->bkg;

		/* Draw the background for the title bar */
		swim_put_box(win, 0, 0, win->xvsize,
					 (4 + swim_get_font_height(win) - 2));

		/* Reset text starting position for title string */
		win->xvpos = win->xpvmin + 2;
		win->yvpos = win->ypvmin + 1;

		/* Restore original pen color (used for text color) */
		win->pen = savedp;

		/* Restore the original colors */
		win->fill = savedf;
		win->bkg = savedb;

		/* Put string in title bar area (with transparent background) */
		win->tfont = 0;
		swim_put_text(win, title);
		win->tfont = savedt;

		/* Draw a line under the title bar, but before the
		   (new) client area */
		swim_put_line(win, 0,
					  (4 + swim_get_font_height(win) - 1),
					  win->xpvmax, (4 + swim_get_font_height(win) - 1));

		/* Adjust client height of window (virtual and physcal) */
		win->ypmin = win->ypmin + swim_get_font_height(win) + 4;
		win->ypvmin = win->ypvmin + swim_get_font_height(win) + 4;

		/* Resize y dimension */
		win->yvsize = win->yvsize - swim_get_font_height(win) + 4;

		/* Reset text starting position to new client area */
		win->xvpos = win->xpvmin;
		win->yvpos = win->ypvmin;
	}
}

/* Enables and disables font backgrounds */
void swim_set_font_trasparency(SWIM_WINDOW_T *win,
							   int32_t trans)
{
	win->tfont = trans;
}
