#ifndef _PINS_H_
#define _PINS_H_

#define bit_is_set(x, y)    ((x) & (1 << (y)))

/*  This file contains the IO pin descriptions for the 2008 robots
    Rev:2.1 (Will not work with Rev:2.0!)
*/
#define IO_LED_RED      (1 << 5)
#define IO_LED_ORANGE   (1 << 4)

#define IO_BUZZER       (1 << 3)

#endif
