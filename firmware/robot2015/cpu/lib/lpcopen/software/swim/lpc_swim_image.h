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

#ifndef __LPC_SWIM_IMAGE_H_
#define __LPC_SWIM_IMAGE_H_

#include "lpc_types.h"
#include "lpc_swim.h"
#include "lpc_colors.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/** @defgroup GUI_SWIM_IMAGE SWIM image manager
 * @ingroup GUI_SWIM
 * This package provides basic SWIM image management capabilities such as
 * image scaling, rotation, and clipping. All image data passed to SWIM
 * must be raw image data (stored left to right, top to bottom) in the
 * same color format as COLOR_T.
 * @{
 */

/**
 * Image rotation tags
 */
typedef enum {NOROTATION, RIGHT, INVERT, LEFT} SWIM_ROTATION_T;

/**
 * @brief	Puts a raw image into a window
 * @param	win		: Pointer to window data structure
 * @param	image	: Pointer to raw image data
 * @param	xsize	: Horizontal size of image data
 * @param	ysize	: Vertical size of image data
 * @return	Nothing
 * @note	Places an image in the upper left corner of the window.
 * Image is cropped to the window size.
 */
void swim_put_image(SWIM_WINDOW_T *win,
					const COLOR_T *image,
					int32_t xsize,
					int32_t ysize);

/**
 * @brief	Puts a raw image into a window inverted
 * @param	win		: Pointer to window data structure
 * @param	image	: Pointer to raw image data
 * @param	xsize	: Horizontal size of image data
 * @param	ysize	: Vertical size of image data
 * @return	Nothing
 * @note	Places an image in the upper left corner of the window,
 * but inverts it. Image is cropped to the window size.
 */
void swim_put_invert_image(SWIM_WINDOW_T *win,
						   const COLOR_T *image,
						   int32_t xsize,
						   int32_t ysize);

/**
 * @brief	Puts a raw image into a window rotated left
 * @param	win		: Pointer to window data structure
 * @param	image	: Pointer to raw image data
 * @param	xsize	: Horizontal size of image data
 * @param	ysize	: Vertical size of image data
 * @return	Nothing
 * @note	Places an image in the upper left corner of the window,
 * but rotates it 90 degrees left. Image is cropped to the
 * window size.
 */
void swim_put_left_image(SWIM_WINDOW_T *win,
						 const COLOR_T *image,
						 int32_t xsize,
						 int32_t ysize);

/**
 * @brief	Puts a raw image into a window rotated right
 * @param	win		: Pointer to window data structure
 * @param	image	: Pointer to raw image data
 * @param	xsize	: Horizontal size of image data
 * @param	ysize	: Vertical size of image data
 * @return	Nothing
 * @note	Places an image in the upper left corner of the window,
 * but rotates it 90 degrees right. Image is cropped to the
 * window size.
 */
void swim_put_right_image(SWIM_WINDOW_T *win,
						  const COLOR_T *image,
						  int32_t xsize,
						  int32_t ysize);

/**
 * @brief	Puts and scales a raw image into a window
 * @param	win		: Pointer to window data structure
 * @param	image	: Pointer to raw image data
 * @param	xsize	: Horizontal size of image data
 * @param	ysize	: Vertical size of image data
 * @return	Nothing
 * @note	Scales the image to the current window. Image will be
 * increased or decreased in size to fit completely in
 * the window.
 */
void swim_put_scale_image(SWIM_WINDOW_T *win,
						  const COLOR_T *image,
						  int32_t xsize,
						  int32_t ysize);

/**
 * @brief	Puts and scales a raw image into a window inverted
 * @param	win		: Pointer to window data structure
 * @param	image	: Pointer to raw image data
 * @param	xsize	: Horizontal size of image data
 * @param	ysize	: Vertical size of image data
 * @return	Nothing
 * @note	Scales and inverts the image to the current window. Image
 * will be increased or decreased in size to fit in the window.
 */
void swim_put_scale_invert_image(SWIM_WINDOW_T *win,
								 const COLOR_T *image,
								 int32_t xsize,
								 int32_t ysize);

/**
 * @brief	Puts and scales a raw image into a window rotated left
 * @param	win		: Pointer to window data structure
 * @param	image	: Pointer to raw image data
 * @param	xsize	: Horizontal size of image data
 * @param	ysize	: Vertical size of image data
 * @return	Nothing
 * @note	Scales and rotates 90 degrees left the image to the current
 * window. Image will be increased or decreased in size to fit
 * in the window.
 */
void swim_put_scale_left_image(SWIM_WINDOW_T *win,
							   const COLOR_T *image,
							   int32_t xsize,
							   int32_t ysize);

/**
 * @brief	Puts and scales a raw image into a window rotated right
 * @param	win		: Pointer to window data structure
 * @param	image	: Pointer to raw image data
 * @param	xsize	: Horizontal size of image data
 * @param	ysize	: Vertical size of image data
 * @return	Nothing
 * @note	Scales and rotates 90 degrees right the image to the current
 * window. Image will be increased or decreased in size to fit
 * in the window.
 */
void swim_put_scale_right_image(SWIM_WINDOW_T *win,
								const COLOR_T *image,
								int32_t xsize,
								int32_t ysize);

/**
 * @brief	SWIM image draw composite function
 * @param	win		: Pointer to window data structure
 * @param	image	: Pointer to raw image data
 * @param	xsize	: Horizontal size of image data
 * @param	ysize	: Vertical size of image data
 * @param	scale	: Set to 1 to scale, or 0 for cropping
 * @param	rtype	: Image rotation type
 * @return	Nothing
 * @note	This function provides a simple call that supports all SWIM
 * image functions.
 */
void swim_put_win_image(SWIM_WINDOW_T *win,
						const COLOR_T *image,
						int32_t xsize,
						int32_t ysize,
						int32_t scale,
						SWIM_ROTATION_T rtype);

#if defined(__cplusplus)
}
#endif

/**
 * @}
 */

#endif /* __LPC_SWIM_IMAGE_H_ */
