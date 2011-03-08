#include <board.h>

#include "main.h"
#include "status.h"
#include "timer.h"
#include "ball_sense.h"
#include "adc.h"

int have_ball;
int ball_sense_light, ball_sense_dark;

// Time the ball status LED was last toggled while flashing
unsigned int last_ball_flash_time;

void update_ball_sensor()
{
	// Update the ball sensor and toggle its LED.
	// Invert the data so increasing values indicate increasing light.
	if (LED_IS_ON(BALL_LED))
	{
		// LED was on
		ball_sense_light = 0x3ff - adc[4];
		
		// Check for emitter failure
		//
		// While the output is low, switch it to input.  If the LED is connected, it will
		// quickly pull the pin high.  If the LED is open, the pin will stay low.
		//
		// I haven't found a way to also detect a shorted emitter without using another ADC
		// channel, which we can't spare (to drive the LED directly it must be on PA0-3,
		// none of which can be ADC inputs, and we don't have enough I/O to connect the LED to
		// two pins).
		AT91C_BASE_PIOA->PIO_CODR = BALL_LED;
		AT91C_BASE_PIOA->PIO_ODR = BALL_LED;
		if (!(AT91C_BASE_PIOA->PIO_PDSR & BALL_LED))
		{
			failures |= Fail_Ball_LED_Open;
		} else {
			failures &= ~Fail_Ball_LED_Open;
		}
		
		// Drive the LED off
		LED_OFF(BALL_LED);
		
		// Make the pin output again
		AT91C_BASE_PIOA->PIO_OER = BALL_LED;
	} else {
		// LED was off
		ball_sense_dark = 0x3ff - adc[4];
		LED_ON(BALL_LED);
	}
	
	// Check for detector failures and excessive ambient light
	failures &= ~(Fail_Ball_Det_Open | Fail_Ball_Det_Short | Fail_Ball_Dazzled);
	if (ball_sense_dark == 0 && ball_sense_light == 0)
	{
		// Detector is open and the pullup resistor pulled the ADC input to 3.3V
		failures |= Fail_Ball_Det_Open;
	} else if (ball_sense_dark == 0x3ff && ball_sense_light == 0x3ff)
	{
		// Detector is shorted so the ADC input is fixed at GND
		failures |= Fail_Ball_Det_Short;
	} else if (ball_sense_dark > 0x200)
	{
		// Too much outside light
		//FIXME - The above number is arbitrary
		failures |= Fail_Ball_Dazzled;
	}
	
	// Update have_ball and the ball status LED
	if (failures & Fail_Ball)
	{
		// The ball sensor is broken
		have_ball = 0;
		
		// Flash the ball status LED
		if ((current_time - last_ball_flash_time) >= 250)
		{
			last_ball_flash_time = current_time;
			LED_TOGGLE(LED_LY);
		}
	} else {
		// The ball sensor works, so determine if we have the ball
		//FIXME - This number is arbitrary
		have_ball = (ball_sense_light - ball_sense_dark) < 0x050;
		
		if (have_ball)
		{
			LED_ON(LED_LY);
		} else {
			LED_OFF(LED_LY);
		}
	}
}
