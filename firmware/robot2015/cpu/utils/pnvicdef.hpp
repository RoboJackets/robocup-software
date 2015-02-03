#pragma once
/**
 * peripherial NVIC ISR defs
 */

//pnvic interrupt controller type register
#define NVIC_ICTR (volatile unsigned int*) 0xE000E004

//pnvic vector table offset register
#define NVIC_VTOR (volatile unsigned int*) 0xE000ED04

//pnvic set-enable registers
#define NVIC_ISER0 (volatile unsigned int*) 0xE000E100
#define NVIC_ISER1 (volatile unsigned int*) 0xE000E104
#define NVIC_ISER2 (volatile unsigned int*) 0xE000E108
#define NVIC_ISER3 (volatile unsigned int*) 0xE000E10C
#define NVIC_ISER4 (volatile unsigned int*) 0xE000E110
#define NVIC_ISER5 (volatile unsigned int*) 0xE000E114
#define NVIC_ISER6 (volatile unsigned int*) 0xE000E118
#define NVIC_ISER7 (volatile unsigned int*) 0xE000E11C

//pnivc clear-enable registers
#define NVIC_ICER0 (volatile unsigned int*) 0xE000E180
#define NVIC_ICER1 (volatile unsigned int*) 0xE000E184
#define NVIC_ICER2 (volatile unsigned int*) 0xE000E188
#define NVIC_ICER3 (volatile unsigned int*) 0xE000E18C
#define NVIC_ICER4 (volatile unsigned int*) 0xE000E190
#define NVIC_ICER5 (volatile unsigned int*) 0xE000E194
#define NVIC_ICER6 (volatile unsigned int*) 0xE000E198
#define NVIC_ICER7 (volatile unsigned int*) 0xE000E19C

//pnivc set-pending registers
#define NVIC_ISPR0 (volatile unsigned int*) 0xE000E200
#define NVIC_ISPR1 (volatile unsigned int*) 0xE000E204
#define NVIC_ISPR2 (volatile unsigned int*) 0xE000E208
#define NVIC_ISPR3 (volatile unsigned int*) 0xE000E20C
#define NVIC_ISPR4 (volatile unsigned int*) 0xE000E210
#define NVIC_ISPR5 (volatile unsigned int*) 0xE000E214
#define NVIC_ISPR6 (volatile unsigned int*) 0xE000E218
#define NVIC_ISPR7 (volatile unsigned int*) 0xE000E21C

//pnvic clear-pending registers
#define NVIC_ICPR0 (volatile unsigned int*) 0xE000E280
#define NVIC_ICPR1 (volatile unsigned int*) 0xE000E284
#define NVIC_ICPR2 (volatile unsigned int*) 0xE000E288
#define NVIC_ICPR3 (volatile unsigned int*) 0xE000E28C
#define NVIC_ICPR4 (volatile unsigned int*) 0xE000E290
#define NVIC_ICPR5 (volatile unsigned int*) 0xE000E294
#define NVIC_ICPR6 (volatile unsigned int*) 0xE000E298
#define NVIC_ICPR7 (volatile unsigned int*) 0xE000E29C

//pnvic active bit registers
#define NVIC_IABR0 (volatile unsigned int*) 0xE000E300
#define NVIC_IABR1 (volatile unsigned int*) 0xE000E304
#define NVIC_IABR2 (volatile unsigned int*) 0xE000E308
#define NVIC_IABR3 (volatile unsigned int*) 0xE000E30C
#define NVIC_IABR4 (volatile unsigned int*) 0xE000E310
#define NVIC_IABR5 (volatile unsigned int*) 0xE000E314
#define NVIC_IABR6 (volatile unsigned int*) 0xE000E318
#define NVIC_IABR7 (volatile unsigned int*) 0xE000E31C

//pnvic interrupt priority register range
#define NVIC_IPR_RANGE_START (volatile unsigned int*) 0xE000E400
#define NVIC_IPR_RANGE_END   (volatile unsigned int*) 0xE000E4EF

//pnvic reset value
#define NVIC_RESET 0x00000000

//pnvic status flags
#define NVIC_GOOD 0x600DC0DE
#define NVIC_DEAD 0xDEADC0DE
#define NVIC_BAD  0xBAADC0DE
