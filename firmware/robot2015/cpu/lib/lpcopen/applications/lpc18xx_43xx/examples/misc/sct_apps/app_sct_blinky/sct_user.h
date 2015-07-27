/*
 * @brief Interface to auto generated State Configurable Timer Blinky 
 * example code
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
 
#ifndef __SCT_USER_H__
#define __SCT_USER_H__

#include "chip.h"


// This is the timer setting for the match register of the L timer
// The other relevant factors for the final blinky speed is the processor clock 
// and the prescaler value and the number of LED's in the 
// For LPC43xx based boards, f(blinky) =  204MHz / (prescaler * speed * nb_of_LEDs) = 204MHz/(256*65535*4) = 3.0Hz
// For LPC18xx based boards, f(blinky) =  180MHz / (prescaler * speed * nb_of_LEDs) = 180MHz/(256*65535*4) = 2.7Hz

#define speed (65535)      // max value for the 16-bit L counter


#endif
