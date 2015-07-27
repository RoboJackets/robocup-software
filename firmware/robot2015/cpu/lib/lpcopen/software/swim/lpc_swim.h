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

#ifndef __LPC_SWIM_H_
#define __LPC_SWIM_H_

#include "lpc_types.h"
#include "lpc_fonts.h"
#include "lpc_colors.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/** @defgroup GUI_SWIM_SWIM Simple Windowing Interface Manager (SWIM)
 * @ingroup GUI_SWIM
 * This package provides the core SWIM capabilities such as
 * Windows initialization and validity checks, Color support for
 * background/primary/fill pens, graphics primatives, and Window
 * deallocation.
 * @{
 */

/**
 * @brief Structure used to store information about a specific window
 */
typedef struct {
	int32_t xpsize;	/* Physical (absolute) horizontal screen size */
	int32_t ypsize;	/* Physical (absolute) vertical screen size */
	int32_t xpmin;	/* Physical left edge of window */
	int32_t ypmin;	/* Physical top edge of window */
	int32_t xpmax;	/* Physical right edge of window */
	int32_t ypmax;	/* Physical bottom edge of window */
	int32_t bdsize;	/* Size of window frame in pixels */
	int32_t xvsize;	/* Virtual horizontal window size */
	int32_t yvsize;	/* Virtual vertical window size */
	int32_t xpvmin;	/* Physical left edge of draw window */
	int32_t ypvmin;	/* Physical top edge of draw window */
	int32_t xpvmax;	/* Physical right edge of draw window */
	int32_t ypvmax;	/* Physical bottom edge of draw window */
	int32_t xvpos;	/* Next virtual 'x' position of output */
	int32_t yvpos;	/* Next virtual 'y' position of output */
	COLOR_T pen;	/* Pen/text color */
	COLOR_T bkg;	/* Window/text background color */
	COLOR_T fill;	/* Fill/border color */
	FONT_T *font;	/* Selected font structure */
	int32_t tfont;	/* Transparent font background flag when true */
	COLOR_T *fb;	/* Frame buffer address for the physical display */
	int32_t winused;	/* Window used flag */
	BOOL_32 tfonts;	/* Transparent font background flag */
} SWIM_WINDOW_T;

/**
 * @brief	Puts a pixel at the virtual X, Y coordinate in the window
 * @param	win	: Pointer to window data structure
 * @param	x1	: Virtual X position of pixel
 * @param	y1	: Virtual Y position of pixel
 * @return	Nothing
 * @note	The pixel will not be displayed if the pixel exceeds the window
 * virtual size. Pixel positions below 0 should not be used with
 * this function.
 */
void swim_put_pixel(SWIM_WINDOW_T *win,
					int32_t x1,
					int32_t y1);

/**
 * @brief	Draw a line in the virtual window with clipping
 * @param	win	: Pointer to window data structure
 * @param	x1	: Virtual X position of X line start
 * @param	y1	: Virtual Y position of Y line start
 * @param	x2	: Virtual X position of X line end
 * @param	y2	: Virtual Y position of Y line end
 * @return	Nothing
 */
void swim_put_line(SWIM_WINDOW_T *win,
				   int32_t x1,
				   int32_t y1,
				   int32_t x2,
				   int32_t y2);

/**
 * @brief	Draw a diamond in the virtual window
 * @param	win	: Pointer to window data structure
 * @param	x	: Virtual X position of the diamond
 * @param	y	: Virtual Y position of the diamond
 * @param	rx	: Radius for horizontal
 * @param	ry	: Radius for vertical
 * @return	Nothing
 */
void swim_put_diamond(SWIM_WINDOW_T *win,
					  int32_t x,
					  int32_t y,
					  int32_t rx,
					  int32_t ry);

/**
 * @brief	Draws a circle in the virtual window
 * @param	win		: Pointer to window data structure
 * @param	cx		: Virtual center X position of the circle
 * @param	cy		: Virtual center Y position of the circle
 * @param	radius	: Radius of the circle
 * @param	Filled	: Flag to indicate whether the circle should be filled
 * @return	Nothing
 */
void swim_put_circle(SWIM_WINDOW_T *win,
					 int32_t cx,
					 int32_t cy,
					 int32_t radius,
					 int32_t Filled);

/**
 * @brief	Fills the draw area of the display with the selected color
 * @param	win		: Pointer to window data structure
 * @param	colr	: Color to place in the window
 * @return	Nothing
 */
void swim_clear_screen(SWIM_WINDOW_T *win,
					   COLOR_T colr);

/**
 * @brief	Place a box with corners (X1, Y1) and (X2, Y2)
 * @param	win	: Pointer to window data structure
 * @param	x1	: Virtual left position of box
 * @param	y1	: Virtual upper position of box
 * @param	x2	: Virtual right position of box
 * @param	y2	: Virtual lower position of box
 * @return	Nothing
 * @note	Use pen color for edges and fill color for center.
 */
void swim_put_box(SWIM_WINDOW_T *win,
				  int32_t x1,
				  int32_t y1,
				  int32_t x2,
				  int32_t y2);

/**
 * @brief	Initializes a window and the default values for the window
 * @param	win				s: Pointer to window data structure
 * @param	xsize			: Physical horizontal dimension of the display
 * @param	ysize			: Physical vertical dimension of the display
 * @param	fbaddr			: Address of the display's frame buffer
 * @param	xwin_min		: Physical window left coordinate
 * @param	ywin_min		: Physical window top coordinate
 * @param	xwin_max		: Physical window right coordinate
 * @param	ywin_max		: Physical window bottom coordinate
 * @param	border_width	: Width of the window border in pixels
 * @param	pcolor			: Pen color
 * @param	bkcolor			: Background color
 * @param	fcolor			: Fill color
 * @return	true if the window was initialized correctly, otherwise false
 * @note	This function must be called prior to any other window function. The
 * window will be drawn in the background color.
 */
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
						 COLOR_T fcolor);

/**
 * @brief	Initializes a window without clearing it
 * @param	win				s: Pointer to window data structure
 * @param	xsize			: Physical horizontal dimension of the display
 * @param	ysize			: Physical vertical dimension of the display
 * @param	fbaddr			: Address of the display's frame buffer
 * @param	xwin_min		: Physical window left coordinate
 * @param	ywin_min		: Physical window top coordinate
 * @param	xwin_max		: Physical window right coordinate
 * @param	ywin_max		: Physical window bottom coordinate
 * @param	border_width	: Width of the window border in pixels
 * @param	pcolor			: Pen color
 * @param	bkcolor			: Background color
 * @param	fcolor			: Fill color
 * @return	true if the window was initialized correctly, otherwise false
 * @note	This function must be called prior to any other window function.
 */
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
								 COLOR_T fcolor);

/**
 * @brief	Deallocates a window
 * @param	win	: Pointer to window data structure
 * @return	Nothing
 * @note	This function does nothing.
 */
void swim_window_close(SWIM_WINDOW_T *win);

/**
 * @brief	Sets the pen color
 * @param	win			: Pointer to window data structure
 * @param	pen_color	: New pen color
 * @return	Nothing
 */
void swim_set_pen_color(SWIM_WINDOW_T *win,
						COLOR_T pen_color);

/**
 * @brief	Sets the fill color
 * @param	win			: Pointer to window data structure
 * @param	fill_color	: New fill color
 * @return	Nothing
 */
void swim_set_fill_color(SWIM_WINDOW_T *win,
						 COLOR_T fill_color);

/**
 * @brief	Sets the color used for backgrounds
 * @param	win			: Pointer to window data structure
 * @param	bkg_color	: New background color
 * @return	Nothing
 */
void swim_set_bkg_color(SWIM_WINDOW_T *win,
						COLOR_T bkg_color);

/**
 * @brief	Get the virtual window horizontal size
 * @param	win	: Pointer to window data structure
 * @return	The virtual window horizontal size
 */
int32_t swim_get_horizontal_size(SWIM_WINDOW_T *win);

/**
 * @brief	Get the virtual window vertical size
 * @param	win	: Pointer to window data structure
 * @return	The virtual window vertical size
 */
int32_t swim_get_vertical_size(SWIM_WINDOW_T *win);

#if defined(__cplusplus)
}
#endif

/**
 * @}
 */

#endif /* __LPC_SWIM_H_ */
