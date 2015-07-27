/*
 * @brief SWIM image management
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

#include "lpc_types.h"
#include "lpc_swim_image.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Puts a raw image into a window */
void swim_put_image(SWIM_WINDOW_T *win,
					const COLOR_T *image,
					int32_t xsize,
					int32_t ysize)
{
	COLOR_T *fb;
	int32_t x, y;

	/* Unknown values of rtype will do no rotation */
	y = win->ypvmin;

	xsize = xsize + win->xpvmin;
	ysize = ysize + win->ypvmin;

	/* Move image to window pixel by pixel */
	while ((y <= win->ypvmax) && (y < ysize)) {
		/* Set physical frame buffer address */
		x = win->xpvmin;
		fb = win->fb + x + (y * win->xpsize);

		/* Render a single line */
		while ((x <= win->xpvmax) && (x < xsize)) {
			*fb = *image;
			fb++;
			image++;
			x++;
		}

		/* Adjust to end of line if the image was clipped */
		image = image + (xsize - x);

		y++;
	}
}

/* Puts a raw image into a window inverted */
void swim_put_invert_image(SWIM_WINDOW_T *win,
						   const COLOR_T *image,
						   int32_t xsize,
						   int32_t ysize)
{
	COLOR_T *fb;
	int32_t x, y, xr, yr;

	y = win->ypvmin;
	yr = ysize - 1;

	/* Move image to window pixel by pixel */
	while ((y <= win->ypvmax) && (yr >= 0)) {
		/* Set physical frame buffer address */
		x = win->xpvmin;
		xr = xsize - 1;
		fb = win->fb + x + (y * win->xpsize);

		/* Render a single line */
		while ((x <= win->xpvmax) && (xr >= 0)) {
			*fb = image[xr + yr * xsize];
			fb++;
			x++;
			xr--;
		}

		y++;
		yr--;
	}
}

/* Puts a raw image into a window rotated left */
void swim_put_left_image(SWIM_WINDOW_T *win,
						 const COLOR_T *image,
						 int32_t xsize,
						 int32_t ysize)
{
	COLOR_T *fb;
	int32_t x, y, xr, yr;

	x = win->xpvmin;
	yr = ysize - 1;

	/* Move image to window pixel by pixel */
	while ((x <= win->xpvmax) && (yr >= 0)) {
		/* Set physical frame buffer address to start drawing at
		   bottom */
		y = win->ypvmin;
		fb = win->fb + x + (y * win->xpsize);
		xr = 0;

		/* Render a single line */
		while ((y <= win->ypvmax) && (xr < xsize)) {
			/* Go to next line (y) */
			*fb = image[(xsize - xr - 1) + (ysize - yr - 1) * xsize];
			fb = fb + win->xpsize;

			/* Update picture to next x coordinate */
			y++;
			xr++;
		}

		x++;
		yr--;
	}
}

/* Puts a raw image into a window rotated right */
void swim_put_right_image(SWIM_WINDOW_T *win,
						  const COLOR_T *image,
						  int32_t xsize,
						  int32_t ysize)
{
	COLOR_T *fb;
	int32_t x, y, xr, yr;

	x = win->xpvmin;
	yr = ysize - 1;

	/* Move image to window pixel by pixel */
	while ((x <= win->xpvmax) && (yr >= 0)) {
		/* Set physical frame buffer address to start drawing at bottom */
		y = win->ypvmin;
		fb = win->fb + x + (y * win->xpsize);
		xr = 0;

		/* Render a single line */
		while ((y <= win->ypvmax) && (xr < xsize)) {
			/* Go to next line (y) */
			*fb = image[xr + yr * xsize];
			fb = fb + win->xpsize;

			/* Update picture to next x coordinate */
			y++;
			xr++;
		}

		x++;
		yr--;
	}
}

/* Puts and scales a raw image into a window */
void swim_put_scale_image(SWIM_WINDOW_T *win,
						  const COLOR_T *image,
						  int32_t xsize,
						  int32_t ysize)
{
	int32_t xsc, ysc;
	int32_t x, y;
	COLOR_T *fb;

	/* Top of window */
	y = win->ypvmin;

	/* Rescale image into window */
	while (y <= win->ypvmax) {
		x = win->xpvmin;

		/* Scale he display size to the image size */
		ysc = ((ysize - 1) * (y - win->ypvmin)) / win->yvsize;
		fb = win->fb + x + (y * win->xpsize);

		/* Render a single line */
		while (x <= win->xpvmax) {
			/* Get x pixel in image */
			xsc = ((xsize - 1) * (x - win->xpvmin)) / win->xvsize;
			*fb = image[xsc + ysc * xsize];
			fb++;
			x++;
		}

		y++;
	}
}

/* Puts and scales a raw image into a window inverted */
void swim_put_scale_invert_image(SWIM_WINDOW_T *win,
								 const COLOR_T *image,
								 int32_t xsize,
								 int32_t ysize)
{
	int32_t xsc, ysc;
	int32_t x, y;
	COLOR_T *fb;

	/* Top of window */
	y = win->ypvmin;

	/* Rescale image into window */
	while (y <= win->ypvmax) {
		x = win->xpvmin;

		/* Scale he display size to the image size */
		ysc = ((ysize - 1) * (y - win->ypvmin)) / win->yvsize;
		fb = win->fb + x + (y * win->xpsize);

		/* Render a single line */
		while (x <= win->xpvmax) {
			/* Get x pixel in image */
			xsc = ((xsize - 1) * (x - win->xpvmin)) / win->xvsize;
			*fb = image[(xsize - 1 - xsc) + (ysize - 1 - ysc) * xsize];
			fb++;
			x++;
		}

		y++;
	}
}

/* Puts and scales a raw image into a window rotated left */
void swim_put_scale_left_image(SWIM_WINDOW_T *win,
							   const COLOR_T *image,
							   int32_t xsize,
							   int32_t ysize)
{
	int32_t xsc, ysc;
	int32_t x, y;
	COLOR_T *fb;

	/* Top of window */
	y = win->ypvmin;

	/* Rescale image into window */
	while (y <= win->ypvmax) {
		x = win->xpvmin;

		/* Scale y coords of picture into x axis */
		ysc = ((xsize - 1) * (win->ypvmax - y)) / win->yvsize;
		fb = win->fb + x + (y * win->xpsize);

		/* Render a single horizontal line with 'y' data */
		while (x <= win->xpvmax) {
			/* Get x pixel in image */
			xsc = ((ysize - 1) * (x - win->xpvmin)) / win->xvsize;
			*fb = image[ysc + xsc * xsize];
			fb++;
			x++;
		}

		y++;
	}
}

/* Puts and scales a raw image into a window rotated right */
void swim_put_scale_right_image(SWIM_WINDOW_T *win,
								const COLOR_T *image,
								int32_t xsize,
								int32_t ysize)
{
	int32_t xsc, ysc;
	int32_t x, y;
	COLOR_T *fb;

	/* Top of window */
	y = win->ypvmin;

	/* Rescale image into window */
	while (y <= win->ypvmax) {
		x = win->xpvmin;

		/* Scale y coords of picture into x axis */
		ysc = ((xsize - 1) * (y - win->ypvmin)) / win->yvsize;
		fb = win->fb + x + (y * win->xpsize);

		/* Render a single horizontal line with 'y' data */
		while (x <= win->xpvmax) {
			/* Get x pixel in image */
			xsc = ((ysize - 1) * (win->xpvmax - x)) / win->xvsize;
			*fb = image[ysc + xsc * xsize];
			fb++;
			x++;
		}

		y++;
	}
}

/* SWIM image draw composite function */
void swim_put_win_image(SWIM_WINDOW_T *win,
						const COLOR_T *image,
						int32_t xsize,
						int32_t ysize,
						int32_t scale,
						SWIM_ROTATION_T rtype)
{
	switch (rtype) {
	case INVERT:
		if (scale != 0) {
			swim_put_scale_invert_image(win, image, xsize, ysize);
		}
		else {
			swim_put_invert_image(win, image, xsize, ysize);
		}
		break;

	case LEFT:
		if (scale != 0) {
			swim_put_scale_left_image(win, image, xsize, ysize);
		}
		else {
			swim_put_left_image(win, image, xsize, ysize);
		}
		break;

	case RIGHT:
		if (scale != 0) {
			swim_put_scale_right_image(win, image, xsize, ysize);
		}
		else {
			swim_put_right_image(win, image, xsize, ysize);
		}
		break;

	case NOROTATION:
	default:
		if (scale != 0) {
			swim_put_scale_image(win, image, xsize, ysize);
		}
		else {
			swim_put_image(win, image, xsize, ysize);
		}
		break;
	}
}
