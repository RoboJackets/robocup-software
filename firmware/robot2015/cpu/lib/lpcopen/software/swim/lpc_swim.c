/*
 * @brief Simple Windowing Interface Manager (SWIM)
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

#include "lpc_swim.h"
#include "lpc_fonts.h"
#include "lpc_helvr10.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Absolute value function */
static int32_t swim_abs(int32_t v1)
{
	if (v1 > 0) {
		return v1;
	}

	return -v1;
}

/* Draw a line on the physical display */
static void swim_put_line_raw(SWIM_WINDOW_T *win,
							  int32_t x1,
							  int32_t y1,
							  int32_t x2,
							  int32_t y2)
{
	int32_t e2, sx, sy, dx, dy, err;

	/* calculate delta_x and delta_y */
	dx = swim_abs(x2 - x1);
	dy = swim_abs(y2 - y1);

	/* set the direction for the step for both x and y, and
	   initialize the error */
	if (x1 < x2) {
		sx = 1;
	}
	else {
		sx = -1;
	}

	if (y1 < y2) {
		sy = 1;
	}
	else {
		sy = -1;
	}

	err = dx - dy;

	while (1) {
		if ((x1 >= 0) && (x1 <= win->xpsize) &&
			(y1 >= 0) && (y1 <= win->ypsize)) {
			*(win->fb + x1 + (y1 * win->xpsize)) = win->pen;
		}

		if ((x1 == x2) && (y1 == y2)) {
			return;
		}

		e2 = 2 * err;
		if (e2 > -dy) {
			err -= dy;
			x1 += sx;
		}
		if (e2 < dx) {
			err += dx;
			y1 += sy;
		}
	}
}

/* Initializes a window and the default values for the window */
static BOOL_32 swim_window_open_p(SWIM_WINDOW_T *win,
								  int32_t xsize,
								  int32_t ysize,
								  COLOR_T *fbaddr,
								  int32_t xwin_min,
								  int32_t ywin_min,
								  int32_t xwin_max,
								  int32_t ywin_max,
								  int32_t border_width,
								  COLOR_T pcolor,
								  COLOR_T bkcolor,
								  COLOR_T fcolor,
								  BOOL_32 clear)
{
	int32_t i;
	BOOL_32 init = false;

	/* Before continuing, check to see that the window size is
	   in the physical dimensions of the display */
	if ((xwin_min >= 0) && (ywin_min >= 0) &&
		(xwin_max < xsize) && (ywin_max < ysize)) {
		init = true;
	}
	else {
		/* Window size is out of the physical display size, so it
		   should be invalidated */
		win->winused = 0x0;
	}

	if (init == true) {
		/* Save physical display dimensions */
		win->xpsize = xsize;
		win->ypsize = ysize;

		/* Save frame buffer address */
		win->fb = fbaddr;

		/* Save physical window dimensions and default colors */
		win->xpmin = xwin_min;
		win->ypmin = ywin_min;
		win->xpmax = xwin_max;
		win->ypmax = ywin_max;
		win->pen = pcolor;
		win->bkg = bkcolor;
		win->fill = fcolor;

		/* Compute physical window dimensions of draw area only */
		win->xpvmin = xwin_min + border_width;
		win->ypvmin = ywin_min + border_width;
		win->xpvmax = xwin_max - border_width;
		win->ypvmax = ywin_max - border_width;

		/* Compute virtual window size of draw area */
		win->xvsize = xwin_max - xwin_min - 2 * border_width;
		win->yvsize = ywin_max - ywin_min - 2 * border_width;

		/* Fill in any unused border padding between draw area and border
		   will fill color */
		for (i = 0; i < border_width; i++) {
			swim_put_line_raw(win, (xwin_min + i),
							  (ywin_min + i), (xwin_max - i), (ywin_min + i));
			swim_put_line_raw(win, (xwin_max - i),
							  (ywin_min + i), (xwin_max - i), (ywin_max - i));
			swim_put_line_raw(win, (xwin_max - i),
							  (ywin_max - i), (xwin_min + i), (ywin_max - i));
			swim_put_line_raw(win, (xwin_min + i),
							  (ywin_max - i), (xwin_min + i), (ywin_min + i));
		}

		/* Clear draw area with background color */
		if (clear == true) {
			swim_clear_screen(win, bkcolor);
		}

		/* Use the default font and make background transparent */
		win->font = (FONT_T *) &font_helvr10;

		/* Set starting text position in upper left of window */
		win->xvpos = win->xpvmin;
		win->yvpos = win->ypvmin;
	}

	return init;
}

/* Circle support function */
static void swim_plot4points(SWIM_WINDOW_T *win, int32_t cx, int32_t cy, int32_t x, int32_t y, int32_t Filled)
{
	int16_t x0, x1, y0, y1;

	y0 = cy + y;
	y1 = cy - y;
	if ( Filled ) {
		for ( x0 = cx - x; x0 <= cx + x; x0++ ) {
			*(win->fb + x0 + (y0 * win->xpsize)) = win->pen;
			*(win->fb + x0 + (y1 * win->xpsize)) = win->pen;
		}
	}
	else {
		x0 = cx + x;
		x1 = cx - x;
		*(win->fb + x0 + (y0 * win->xpsize)) = win->pen;
		if (x != 0) {
			*(win->fb + x1 + (y0 * win->xpsize)) = win->pen;
		}
		if (y != 0) {
			*(win->fb + x0 + (y1 * win->xpsize)) = win->pen;
		}
		if (( x != 0) && ( y != 0) ) {
			*(win->fb + x1 + (y1 * win->xpsize)) = win->pen;
		}
	}
}

/* Circle support function */
static void swim_plot8points(SWIM_WINDOW_T *win, int32_t cx, int32_t cy, int32_t x, int32_t y, int32_t Filled)
{
	swim_plot4points(win, cx, cy, x, y, Filled);
	if (x != y) {
		swim_plot4points(win, cx, cy, y, x, Filled);
	}
}

/***********************************************************************
 * Public functions
 **********************************************************************/

/* Puts a pixel at the virtual X, Y coordinate in the window */
void swim_put_pixel(SWIM_WINDOW_T *win,
					int32_t x1,
					int32_t y1)
{
	int16_t realx, realy;

	/* Convert virtual coordinate to physical coordinate taking into
	   consideration the border size of the window */
	realx = win->xpvmin + x1;
	realy = win->ypvmin + y1;

	/* Only put the pixel in the window if it fits in the window */
	if ((realx <= win->xpvmax) &&
		(realy <= win->ypvmax)) {
		*(win->fb + realx + (realy * win->xpsize)) = win->pen;
	}
}

/* Draw a line in the virtual window with clipping */
void swim_put_line(SWIM_WINDOW_T *win,
				   int32_t x1,
				   int32_t y1,
				   int32_t x2,
				   int32_t y2)
{
	int32_t e2, sx, sy, dx, dy, err;

	/* Convert virtual coordinates to physical coordinates */
	x1 = x1 + win->xpvmin;
	x2 = x2 + win->xpvmin;
	y1 = y1 + win->ypvmin;
	y2 = y2 + win->ypvmin;

	/* calculate delta_x and delta_y */
	dx = swim_abs(x2 - x1);
	dy = swim_abs(y2 - y1);

	/* set the direction for the step for both x and y, and
	   initialize the error */
	if (x1 < x2) {
		sx = 1;
	}
	else {
		sx = -1;
	}

	if (y1 < y2) {
		sy = 1;
	}
	else {
		sy = -1;
	}

	err = dx - dy;

	while (1) {
		if ((x1 >= win->xpvmin) && (x1 <= win->xpvmax) &&
			(y1 >= win->ypvmin) && (y1 <= win->ypvmax)) {
			*(win->fb + x1 + (y1 * win->xpsize)) = win->pen;
		}

		if ((x1 == x2) && (y1 == y2)) {
			return;
		}

		e2 = 2 * err;
		if (e2 > -dy) {
			err -= dy;
			x1 += sx;
		}
		if (e2 < dx) {
			err += dx;
			y1 += sy;
		}
	}
}

/* Draw a diamond in the virtual window */
void swim_put_diamond(SWIM_WINDOW_T *win,
					  int32_t x,
					  int32_t y,
					  int32_t rx,
					  int32_t ry)
{
	int32_t xleft, xright, xleft1, xleft2, xright1, idy, ypmid;
	int32_t ypmin, ypmax, dlta, err, e2;

	/* Use line draw functions to draw border in pen color in virtual
	   coordinates */
	swim_put_line(win, x - rx, y, x, y - ry);
	swim_put_line(win, x + rx, y, x, y - ry);
	swim_put_line(win, x - rx, y, x, y + ry);
	swim_put_line(win, x + rx, y, x, y + ry);

	/* Adjust rx and rx for interior fill region minus border */
	rx--;
	ry--;
	if ((rx <= 0) || (ry <= 0)) {
		return;
	}

	/* Y limits in physical coordinates minus border line */
	ypmin = y - ry + win->ypvmin;
	ypmid = y + win->ypvmin;
	ypmax = y + ry + win->ypvmin;

	/* X starts draw from center line */
	xleft = xright = x + win->xpvmin;

	err = rx - ry;
	dlta = 1 + rx / ry;

	for (idy = ypmin; idy <= ypmid; idy++) {
		xleft1 = xleft2 = xleft;
		xright1 = xright;

		/* Clip left and right to virtual window size */
		if (xleft1 < win->xpvmin) {
			xleft2 = xleft1 = win->xpvmin;
		}
		if (xright1 > win->xpvmax) {
			xright1 = win->xpvmax;
		}

		/* Is top half visible? */
		if ((idy >= win->ypvmin) && (idy <= win->ypvmax)) {
			while (xleft1 <= xright1) {
				*(win->fb + xleft1 + (idy * win->xpsize)) = win->fill;
				xleft1++;
			}
		}

		/* Draw bottom half if visible */
		if ((ypmax >= ypmid) && (ypmax <= win->ypvmax)) {
			/* Mirror bottom */
			while (xleft2 <= xright1) {
				*(win->fb + xleft2 + (ypmax * win->xpsize)) = win->fill;
				xleft2++;
			}
		}
		ypmax--;

		e2 = 2 * err;
		if (e2 > -ry) {
			err -= ry;
			xleft -= dlta;
			xright += dlta;
		}
		if (e2 < rx) {
			err += rx;
		}
	}
}

/* Draws a circle in the virtual window */
void swim_put_circle(SWIM_WINDOW_T *win, int32_t cx, int32_t cy, int32_t radius, int32_t Filled)
{
	int32_t Error = -radius;
	int16_t x = radius;
	int16_t y = 0;

	while ( x >= y ) {
		swim_plot8points(win, cx, cy, x, y, Filled);

		Error += y;
		++y;
		Error += y;

		if ( Error >= 0 ) {
			--x;
			Error -= x;
			Error -= x;
		}
	}
}

/* Fills the draw area of the display with the selected color */
void swim_clear_screen(SWIM_WINDOW_T *win,
					   COLOR_T colr)
{
	int32_t x, y;

	for (y = win->ypvmin; y <= win->ypvmax; y++) {
		for (x = win->xpvmin; x <= win->xpvmax; x++) {
			*(win->fb + x + (y * win->xpsize)) = colr;
		}
	}
}

/* Place a box with corners (X1, Y1) and (X2, Y2) */
void swim_put_box(SWIM_WINDOW_T *win,
				  int32_t x1,
				  int32_t y1,
				  int32_t x2,
				  int32_t y2)
{
	int32_t xinc, yinc;
	int32_t ysave;

	if (x1 > x2) {
		xinc = x1;
		x1 = x2;
		x2 = xinc;
	}

	/* Swap y1 and y2 if y1 is larger than y2 */
	if (y1 > y2) {
		yinc = y1;
		y1 = y2;
		y2 = yinc;
	}

	/* Convert virtual coordinates to physical coordinates */
	x1 = x1 + win->xpvmin;
	x2 = x2 + win->xpvmin;
	y1 = y1 + win->ypvmin;
	y2 = y2 + win->ypvmin;

	/* Clip boxes to window sizes */
	if (x1 < win->xpvmin) {
		x1 = win->xpvmin;
	}
	if (y1 < win->ypvmin) {
		y1 = win->ypvmin;
	}
	if (x2 > win->xpvmax) {
		x2 = win->xpvmax;
	}
	if (y2 > win->ypvmax) {
		y2 = win->ypvmax;
	}

	/* Get X and Y differences */
	xinc = x2 - x1;
	yinc = y2 - y1;

	/* Make outer edge of box in pen color */
	swim_put_line_raw(win, x1, y1, x2, y1);
	swim_put_line_raw(win, x2, y1, x2, y2);
	swim_put_line_raw(win, x2, y2, x1, y2);
	swim_put_line_raw(win, x1, y2, x1, y1);

	/* Increment X, Y values so they won't overwrite the edge */
	x1++;
	y1++;

	/* Draw the box inside with the fill color */
	ysave = y1;
	while (x1 < x2) {
		y1 = ysave;
		while (y1 < y2) {
			*(win->fb + x1 + (y1 * win->xpsize)) = win->fill;
			y1++;
		}

		x1++;
	}
}

/* Initializes a window and the default values for the window */
BOOL_32 swim_window_open(SWIM_WINDOW_T *win,
						 int32_t xsize,
						 int32_t ysize,
						 COLOR_T *fbaddr,
						 int32_t xwin_min,
						 int32_t ywin_min,
						 int32_t xwin_max,
						 int32_t ywin_max,
						 int32_t border_width,
						 COLOR_T pcolor,
						 COLOR_T bkcolor,
						 COLOR_T fcolor)
{
	BOOL_32 init;

	init = swim_window_open_p(win, xsize, ysize, fbaddr, xwin_min,
							  ywin_min, xwin_max, ywin_max, border_width, pcolor, bkcolor,
							  fcolor, true);

	/* Default font background is not transparent */
	win->tfont = 1;

	return init;
}

/* Initializes a window without clearing it */
BOOL_32 swim_window_open_noclear(SWIM_WINDOW_T *win,
								 int32_t xsize,
								 int32_t ysize,
								 COLOR_T *fbaddr,
								 int32_t xwin_min,
								 int32_t ywin_min,
								 int32_t xwin_max,
								 int32_t ywin_max,
								 int32_t border_width,
								 COLOR_T pcolor,
								 COLOR_T bkcolor,
								 COLOR_T fcolor)
{
	BOOL_32 init;

	init = swim_window_open_p(win, xsize, ysize, fbaddr, xwin_min,
							  ywin_min, xwin_max, ywin_max, border_width, pcolor, bkcolor,
							  fcolor, false);

	/* Default font background is transparent */
	win->tfont = 0;

	return init;
}

/* Deallocates a window */
void swim_window_close(SWIM_WINDOW_T *win)
{
	win->winused = 0x0;
}

/* Sets the pen color */
void swim_set_pen_color(SWIM_WINDOW_T *win,
						COLOR_T pen_color)
{
	win->pen = pen_color;
}

/* Sets the fill color */
void swim_set_fill_color(SWIM_WINDOW_T *win,
						 COLOR_T fill_color)
{
	win->fill = fill_color;
}

/* Sets the color used for backgrounds */
void swim_set_bkg_color(SWIM_WINDOW_T *win,
						COLOR_T bkg_color)
{
	win->bkg = bkg_color;
}

/* Get the virtual window horizontal size */
int32_t swim_get_horizontal_size(SWIM_WINDOW_T *win)
{
	return win->xvsize;
}

/* Get the virtual window vertical size */
int32_t swim_get_vertical_size(SWIM_WINDOW_T *win)
{
	return win->yvsize;
}
