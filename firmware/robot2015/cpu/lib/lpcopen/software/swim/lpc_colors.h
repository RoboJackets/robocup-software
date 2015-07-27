/*
 * @brief SWIM color definitions and palette table setup
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

#ifndef __LPC_COLOR_H_
#define __LPC_COLOR_H_

#include "lpc_types.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/** @defgroup GUI_SWIM_COLORS SWIM color definitions
 * @ingroup GUI_SWIM
 * The Simple Windows Interface manager (SWIM) supports 8-bit RGB323,
 * 12-bit RGB 444 (in a 16-bit field), 15-bit RGB555, 16-bit RGB565,
 * and 24-bit RGB888 (in a 32-bit field) color.
 *
 * SWIM is configured for the color type at build-time based on the
 * the COLORS_DEF definitions. Select one of the following values to
 * configure SWIM.
 *  8-bit RGB323 : COLORS_DEF = 8
 *  12-bit RGB323: COLORS_DEF = 12
 *  15-bit RGB323: COLORS_DEF = 15
 *  16-bit RGB323: COLORS_DEF = 16
 *  24-bit RGB323: COLORS_DEF = 24
 * @{
 */

/**
 * Default configuration values
 */

#ifndef COLORS_DEF
//#define COLORS_DEF 24		/* 24-bit 888 color mode */
#define COLORS_DEF 16		/* 16-bit 565 color mode */
// #define COLORS_DEF 15	/* 15-bit 555 color mode */
// #define COLORS_DEF 12	/* 12-bit 444 color mode */
// #define COLORS_DEF 8		/* 8-bit color mode */
#endif

#if defined(COLORS_DEF)
#ifndef COLORS_8_565_MODE
/** Setup the palette table for RGB565 mode if COLORS_8_565_MODE
    is defined, other use RGB555 mode if not defined. */
#define COLORS_8_565_MODE
#endif
#endif

#if COLORS_DEF == 24
/* Black color, 888 mode */
#define BLACK         0x000000
/* Light gray color, 888 mode */
#define LIGHTGRAY     0x545454
/* Dark gray color, 888 mode */
#define DARKGRAY      0xA8A8A8
/* White color, 888 mode */
#define WHITE         0xFFFFFF
/* Red color, 888 mode */
#define RED           0xFF0000
/* Green color, 888 mode */
#define GREEN         0x00FF00
/* Blue color, 888 mode */
#define BLUE          0x0000FF

/* Light red color, 888 mode */
#define LIGHTRED      0x3F0000
/* Light green color, 888 mode */
#define LIGHTGREEN    0x003F00
/* Light blue color, 888 mode */
#define LIGHTBLUE     0x00003F

/* Minimum red color, 888 mode */
#define MINRED      0x010000
/* Light green color, 888 mode */
#define MINGREEN    0x000100
/* Light blue color, 888 mode */
#define MINBLUE     0x000001

/* Red color mask, 888 mode */
#define REDMASK       0xFF0000
/* Red shift value, 888 mode */
#define REDSHIFT      16
/* Green color mask, 888 mode */
#define GREENMASK     0x00FF00
/* Green shift value, 888 mode */
#define GREENSHIFT    8
/* Blue color mask, 888 mode */
#define BLUEMASK      0x0000FF
/* Blue shift value, 888 mode */
#define BLUESHIFT     0

/* Number of colors in 888 mode */
#define NUM_COLORS    16777216
/* Number of red colors in 888 mode */
#define RED_COLORS    0x100
/* Number of green colors in 888 mode */
#define GREEN_COLORS  0x100
/* Number of blue colors in 888 mode */
#define BLUE_COLORS   0x100

/* Color type is a 16-bit value */
typedef uint32_t COLOR_T;
#endif

#if COLORS_DEF == 16
/* Black color, 565 mode */
#define BLACK         0x0000
/* Light gray color, 565 mode */
#define LIGHTGRAY     0X7BEF
/* Dark gray color, 565 mode */
#define DARKGRAY      0x39E7
/* White color, 565 mode */
#define WHITE         0xffff
/* Red color, 565 mode */
#define RED           0xF800
/* Green color, 565 mode */
#define GREEN         0x07E0
/* Blue color, 565 mode */
#define BLUE          0x001F

/* Light red color, 565 mode */
#define LIGHTRED      0x7800
/* Light green color, 565 mode */
#define LIGHTGREEN    0x03E0
/* Light blue color, 565 mode */
#define LIGHTBLUE     0x000F

/* Minimum red color, 565 mode */
#define MINRED      0x0800
/* Light green color, 565 mode */
#define MINGREEN    0x0020
/* Light blue color, 565 mode */
#define MINBLUE     0x0001

/* Red color mask, 565 mode */
#define REDMASK       0xF800
/* Red shift value, 565 mode */
#define REDSHIFT      11
/* Green color mask, 565 mode */
#define GREENMASK     0x07E0
/* Green shift value, 565 mode */
#define GREENSHIFT    5
/* Blue color mask, 565 mode */
#define BLUEMASK      0x001F
/* Blue shift value, 565 mode */
#define BLUESHIFT     0

/* Number of colors in 565 mode */
#define NUM_COLORS    65536
/* Number of red colors in 565 mode */
#define RED_COLORS    0x20
/* Number of green colors in 565 mode */
#define GREEN_COLORS  0x40
/* Number of blue colors in 565 mode */
#define BLUE_COLORS   0x20

/* Color type is a 16-bit value */
typedef uint16_t COLOR_T;
#endif

#if COLORS_DEF == 15
/* Black color, 555 mode */
#define BLACK         0x0000
/* Llight gray color, 555 mode */
#define LIGHTGRAY     0x3DEF
/* Drak gray color, 555 mode */
#define DARKGRAY      0x1CE7
/* White color, 555 mode */
#define WHITE         0xffff
/* Red color, 555 mode */
#define RED           0x7C00
/* Green color, 555 mode */
#define GREEN         0x03E0
/* Blue color, 555 mode */
#define BLUE          0x001F
/* Magenta color, 555 mode */
#define MAGENTA       (RED | BLUE)
/* Cyan color, 555 mode */
#define CYAN          (GREEN | BLUE)
/* Yellow color, 555 mode */
#define YELLOW        (RED | GREEN)
/* Light red color, 555 mode */
#define LIGHTRED      0x3C00
/* Light green color, 555 mode */
#define LIGHTGREEN    0x01E0
/* Light blue color, 555 mode */
#define LIGHTBLUE     0x000F
/* Light magenta color, 555 mode */
#define LIGHTMAGENTA  (LIGHTRED | LIGHTBLUE)
/* Light cyan color, 555 mode */
#define LIGHTCYAN     (LIGHTGREEN | LIGHTBLUE)
/* Light yellow color, 555 mode */
#define LIGHTYELLOW   (LIGHTRED | LIGHTGREEN)

/* Red color mask, 555 mode */
#define REDMASK       0x7C00
/* Red shift value, 555 mode */
#define REDSHIFT      10
/* Green color mask, 555 mode */
#define GREENMASK     0x03E0
/* Green shift value, 555 mode */
#define GREENSHIFT    5
/* Blue color mask, 555 mode */
#define BLUEMASK      0x001F
/* Blue shift value, 555 mode */
#define BLUESHIFT     0

/* Number of colors in 555 mode */
#define NUM_COLORS    32768
/* Number of red colors in 555 mode */
#define RED_COLORS    0x20
/* Number of green colors in 555 mode */
#define GREEN_COLORS  0x20
/* Number of blue colors in 555 mode */
#define BLUE_COLORS   0x20

/* Color type is a 16-bit value */
typedef uint16_t COLOR_T;
#endif

#if COLORS_DEF == 12
/* Black color, 444 mode */
#define BLACK         0x0000
/* Llight gray color, 444 mode */
#define LIGHTGRAY     0x3DEF
/* Drak gray color, 444 mode */
#define DARKGRAY      0x1CE7
/* White color, 444 mode */
#define WHITE         0x7fff
/* Red color, 444 mode */
#define RED           0x3C00
/* Green color, 444 mode */
#define GREEN         0x01E0
/* Blue color, 444 mode */
#define BLUE          0x000F
/* Magenta color, 444 mode */
#define MAGENTA       (RED | BLUE)
/* Cyan color, 444 mode */
#define CYAN          (GREEN | BLUE)
/* Yellow color, 444 mode */
#define YELLOW        (RED | GREEN)
/* Light red color, 444 mode */
#define LIGHTRED      0x3C00
/* Light green color, 444 mode */
#define LIGHTGREEN    0x01E0
/* Light blue color, 444 mode */
#define LIGHTBLUE     0x000F
/* Light magenta color, 444 mode */
#define LIGHTMAGENTA  (LIGHTRED | LIGHTBLUE)
/* Light cyan color, 444 mode */
#define LIGHTCYAN     (LIGHTGREEN | LIGHTBLUE)
/* Light yellow color, 444 mode */
#define LIGHTYELLOW   (LIGHTRED | LIGHTGREEN)

/* Red color mask, 444 mode */
#define REDMASK       0x3C00
/* Red shift value, 444 mode */
#define REDSHIFT      10
/* Green color mask, 444 mode */
#define GREENMASK     0x01E0
/* Green shift value, 444 mode */
#define GREENSHIFT    5
/* Blue color mask, 444 mode */
#define BLUEMASK      0x000F
/* Blue shift value, 444 mode */
#define BLUESHIFT     0

/* Number of colors in 444 mode */
#define NUM_COLORS    4096
/* Number of red colors in 444 mode */
#define RED_COLORS    0x10
/* Number of green colors in 444 mode */
#define GREEN_COLORS  0x10
/* Number of blue colors in 444 mode */
#define BLUE_COLORS   0x10

/* Color type is a 16-bit value */
typedef uint16_t COLOR_T;
#endif

#if COLORS_DEF == 8
/* Black color, 323 mode */
#define BLACK         0x00
/* Light gray color, 323 mode */
#define LIGHTGRAY     0x6E
/* Dark gray color, 323 mode */
#define DARKGRAY      0x25
/* White color, 323 mode */
#define WHITE         0xFF
/* Red color, 323 mode */
#define RED           0xE0
/* Green color, 323 mode */
#define GREEN         0x1C
/* Blue color, 323 mode */
#define BLUE          0x03
/* Magenta color, 323 mode */
#define MAGENTA       (RED | BLUE)
/* Cyan color, 323 mode */
#define CYAN          (GREEN | BLUE)
/* Yellow color, 323 mode */
#define YELLOW        (RED | GREEN)
/* Light red color, 323 mode */
#define LIGHTRED      0x60
/* Light green color, 323 mode */
#define LIGHTGREEN    0x0C
/* Light blue color, 323 mode */
#define LIGHTBLUE     0x01
/* Light magenta color, 323 mode */
#define LIGHTMAGENTA  (LIGHTRED | LIGHTBLUE)
/* Light cyan color, 323 mode */
#define LIGHTCYAN     (LIGHTGREEN | LIGHTBLUE)
/* Light yellow color, 323 mode */
#define LIGHTYELLOW   (LIGHTRED | LIGHTGREEN)

/* Red color mask, 323 mode */
#define REDMASK       0xE0
/* Red shift value, 323 mode */
#define REDSHIFT      5
/* Green color mask, 323 mode */
#define GREENMASK     0x1C
/* Green shift value, 323 mode */
#define GREENSHIFT    2
/* Blue color mask, 323 mode */
#define BLUEMASK      0x3
/* Blue shift value, 323 mode */
#define BLUESHIFT     0

/* Number of colors in 332 mode */
#define NUM_COLORS    256
/* Number of red colors in 332 mode */
#define RED_COLORS    0x08
/* Number of green colors in 332 mode */
#define GREEN_COLORS  0x08
/* Number of blue colors in 332 mode */
#define BLUE_COLORS   0x08

/* Color type is a 8-bit value */
typedef uint8_t COLOR_T;
#endif

/**
 * @brief	Generate a palette table (only in 8-bit mode)
 * @param	palette_table	: Pointer to palette table (256 entries)
 * @return	Nothing
 * @note	Depending on the target LCD color mapping (either 555 or 565), a
 * palette table will be generated to convert colors stored in 233
 * format to either 555 or 565 format through a lookup table.
 * If compiled in 16-bit color mode, this will be a NULL function.
 * Select the appropriate define in this function for 555 or 565
 * color mode displays when using an 256 color frame buffer.
 */
void lpc_colors_set_palette(uint16_t *palette_table);

/**
 * @}
 */

#if defined(__cplusplus)
}
#endif /*__cplusplus */

#endif /* __LPC_COLOR_H_ */
