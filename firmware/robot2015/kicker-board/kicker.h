#pragma once

#include <avr/io.h>

/*
 * init() currently forces freq to 9.6MHz, so we need to define it before delay
 * 	0.104us/cycle
 */
#define F_CPU 9600000UL
#include <util/delay.h>
