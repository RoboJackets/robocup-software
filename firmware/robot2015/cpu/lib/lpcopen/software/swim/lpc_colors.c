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

#include "lpc_colors.h"

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

/* Generate a palette table (only in 8-bit mode) */
void lpc_colors_set_palette(uint16_t *palette_table)
{
#if COLORS_DEF == 8
	int32_t idx;
	uint16_t entry, r, g, b;

	/* 256 entries */
	for (idx = 0; idx < NUM_COLORS; idx++) {
		r = ((uint16_t) idx & REDMASK) >> REDSHIFT;
		g = ((uint16_t) idx & GREENMASK) >> GREENSHIFT;
		b = ((uint16_t) idx & BLUEMASK) >> BLUESHIFT;

#ifdef COLORS_8_565_MODE
		/* Strip out and scale colors */
		r = r * 0x1F / ((REDMASK >> REDSHIFT) + 1);
		g = g * 0x3F / ((GREENMASK >> GREENSHIFT) + 1);
		b = b * 0x1F / ((BLUEMASK >> BLUESHIFT) + 1);
		entry = b + (g << 5) + (r << 11);

#else
		/* Strip out and scale colors */
		r = r * 0x1F / ((REDMASK >> REDSHIFT) + 1);
		g = g * 0x1F / ((GREENMASK >> GREENSHIFT) + 1);
		b = b * 0x1F / ((BLUEMASK >> BLUESHIFT) + 1);
		entry = b + (g << 5) + (r << 10);
#endif

		/* Save palette entry */
		palette_table[idx] = entry;
	}
#endif /* COLORS_DEF == 8 */
}
