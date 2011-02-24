#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

#include "main.h"
#include "LCD_driver.h"
#include "button.h"

// Distance between sensor centers in 1/10 mm
#define SENSOR_DISTANCE 265

// Additional 8 bits to augment timer 0
volatile uint16_t tcnt0_high;

// Nonzero when t0tr_high wraps
volatile uint8_t tcnt0_overflow;

volatile uint8_t beep_time;
volatile uint16_t power_off_time;

enum
{
	SPEED_IDLE,			// Waiting for any beam to be broken

	// Object moving from first to second beam
	SPEED_FWD_FIRST,	// First beam is broken, forward direction
	SPEED_FWD_SECOND,	// Second beam is broken, forward direction
	SPEED_FWD_DONE,		// Cleared second beam

	// Object moving from second to first beam
	SPEED_REV_SECOND,	// Second beam broken, reverse direction
	SPEED_REV_FIRST,	// First beam broken, reverse direction
	SPEED_REV_DONE		// Cleared first beam
};

volatile uint8_t speed_state = SPEED_IDLE;

void beep(uint16_t period)
{
	cli();
	TCCR1A = 0x40;
	TCCR1B = 0x09;
	OCR1A = period;
	beep_time = 15;
	sei();
}

void beep_off()
{
	TCCR1A = 0;
	TCCR1B = 0;
}

void clear_display()
{
	lcd_puts_r(PSTR(" -----"));
}

void reset_auto_power_off()
{
	// 5 minutes
	power_off_time = 36621;
}

void reset()
{
	// Stop the beeper timer
	beep_off();
	
	// Stop the break-beam timer
	TCCR0A = 0;

	// Reset the speed measurement state
	speed_state = SPEED_IDLE;

	reset_auto_power_off();
}

void power_off()
{
	reset();

	// Turn off LEDs
	PORTE &= ~(1 << 6);

    // Disable LCD
    LCDCRA &= ~(1 << 7);

	// Wait for all buttons to be released
    while ((PINB & 0xd0) != 0xd0 || (PINE & 0x0c) != 0x0c)
    {
		Delay(100);
	}

	// Sleep until a button is pressed
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    while ((PINB & 0xd0) == 0xd0 && (PINE & 0x0c) == 0x0c)
    {
        sleep_mode();
    }
    
	// Clear the LCD screen before enabling
    uint8_t i;
    for (i = 0; i < 20; i++)
    {
        *(pLCDREG + i) = 0x00;
    }
    
    // Enable LCD
    LCDCRA |= 1 << 7;

	// Turn on LEDs
	PORTE |= 1 << 6;

	// Do this again because the LEDs just turned on
	reset();
	clear_display();
}

ISR(TIMER0_OVF_vect)
{
    tcnt0_high++;
    if (tcnt0_high == 0)
    {
        tcnt0_overflow = 1;
    }
}

void timer_start()
{
	tcnt0_overflow = 0;
    tcnt0_high = 0;
    TCNT0 = 0;
    TCCR0A = 1;
}

void speed_interrupt()
{
	// Nonzero if first beam is broken
	uint8_t first = PINE & (1 << 4);

	// Nonzero if second beam is broken
	uint8_t second = PINE & (1 << 5);

	switch (speed_state)
	{
		case SPEED_IDLE:
			if (second)
			{
				timer_start();
				speed_state = SPEED_REV_SECOND;
			} else if (first)
			{
				timer_start();
				speed_state = SPEED_FWD_FIRST;
			}
			break;

		case SPEED_FWD_FIRST:
			if (second)
			{
				TCCR0A = 0;
				speed_state = SPEED_FWD_SECOND;
			}
			break;

		case SPEED_FWD_SECOND:
			if (!second && !first)
			{
				speed_state = SPEED_FWD_DONE;
			}
			break;

		case SPEED_REV_SECOND:
			if (first)
			{
				TCCR0A = 0;
				speed_state = SPEED_REV_FIRST;
			}
			break;

		case SPEED_REV_FIRST:
			if (!second && !first)
			{
				speed_state = SPEED_REV_DONE;
			}
			break;
	}
}

ISR(TIMER2_OVF_vect)
{
	if (power_off_time)
	{
		--power_off_time;
	}

	if (beep_time)
	{
		--beep_time;
		if (!beep_time)
		{
			beep_off();
		}
	}
}

int main()
{
	uint32_t time_us;

    Initialization();

	// Beeper
	PORTB |= 0x20;
	DDRB |= 0x20;
	// LED enable
	DDRE |= 0x40;
	// Pullups on phototransistors
	PORTE |= 0x30;

    // Timer 0 is used for break-beam timing
    TCCR0A = 0;
    TIMSK0 = 1;
    
	// Timer 2 is used for beep and power-off timing
	ASSR = 0;
	TCCR2A = 3;
	TIMSK2 = 1;

	power_off();

    while (1)
    {
		set_sleep_mode(SLEEP_MODE_IDLE);
		sleep_mode();

		// Auto power off
		if (power_off_time == 0)
		{
			power_off();
		}

		switch (getkey())
		{
			case KEY_DOWN:
				// Power off
				power_off();
				break;

			case KEY_UP:
				// Clear last measurement
				reset();
				clear_display();
				break;
			
			case KEY_RIGHT:
			{
				// Display raw time in microseconds
				uint32_t v = time_us;
                lcd_clear();
                for (uint8_t i = 0; i < 6; i++)
                {
                    uint8_t digit = v % 10;
                    v /= 10;
                    
                    lcd_putc(5 - i, '0' + digit);
                }
				lcd_update();
				break;
			}
		}

        if (speed_state == SPEED_FWD_DONE || speed_state == SPEED_REV_DONE)
        {
			// Display last speed
            if (tcnt0_overflow)
            {
                tcnt0_overflow = 0;
                lcd_puts_r(PSTR("SLOW"));
            } else {
                // Time in microseconds
                time_us = (uint32_t)TCNT0 + ((uint32_t)tcnt0_high << 8);
                
                // Velocity in mm/s
                // v(mm/s) = d(mm) / (t(us) / 1e6)
                uint32_t v = SENSOR_DISTANCE * 100000 / time_us;
                if (v > 99999)
                {
                    lcd_puts_r(PSTR("FAST"));
                } else {
                    lcd_clear();
                    for (uint8_t i = 0; i < 5; i++)
                    {
                        uint8_t digit = v % 10;
                        v /= 10;
                        
                        lcd_putc(5 - i, '0' + digit);
                    }

					if (speed_state == SPEED_FWD_DONE)
					{
						lcd_putc(0, '>');
					} else {
						lcd_putc(0, '<');
					}

                    lcd_update();
                }
            }

			if (speed_state == SPEED_FWD_DONE)
			{
				beep(1400);
			} else {
				beep(600);
			}
			speed_state = SPEED_IDLE;
			reset_auto_power_off();
		} else if (speed_state != SPEED_IDLE)
		{
			// Measurement in progress
			clear_display();
		}
    }
}

/*****************************************************************************
*
*   Function name : Initialization
*
*   Returns :       None
*
*   Parameters :    None
*
*   Purpose :       Initializate the different modules
*
*****************************************************************************/
void Initialization(void)
{
    OSCCAL_calibration();       // calibrate the OSCCAL byte
        
    CLKPR = (1<<CLKPCE);        // set Clock Prescaler Change Enable

    // set prescaler = 8, Inter RC 8Mhz / 8 = 1Mhz
    CLKPR = (1<<CLKPS1) | (1<<CLKPS0);

    // Disable Analog Comparator (power save)
    ACSR = (1<<ACD);

    // Disable Digital input on PF0-2 (power save)
    DIDR0 = (7<<ADC0D);

    Button_Init();              // Initialize pin change interrupt on joystick
    lcd_init();                 // initialize the LCD
}

/*****************************************************************************
*
*   Function name : Delay
*
*   Returns :       None
*
*   Parameters :    unsigned int millisec
*
*   Purpose :       Delay-loop
*
*****************************************************************************/
void Delay(unsigned int millisec)
{
	// mt, int i did not work in the simulator:  int i; 
	uint8_t i;

	while (millisec--) {
		for (i=0; i<125; i++) {
			asm volatile ("nop"::);
		}
	}
}

/*****************************************************************************
*
*   Function name : OSCCAL_calibration
*
*   Returns :       None
*
*   Parameters :    None
*
*   Purpose :       Calibrate the internal OSCCAL byte, using the external 
*                   32,768 kHz crystal as reference
*
*****************************************************************************/
void OSCCAL_calibration(void)
{
    unsigned char calibrate = FALSE;
    int temp;
    unsigned char tempL;

    CLKPR = (1<<CLKPCE);        // set Clock Prescaler Change Enable
    // set prescaler = 8, Inter RC 8Mhz / 8 = 1Mhz
    CLKPR = (1<<CLKPS1) | (1<<CLKPS0);
    
    TIMSK2 = 0;             //disable OCIE2A and TOIE2

    ASSR = (1<<AS2);        //select asynchronous operation of timer2 (32,768kHz)
    
    OCR2A = 200;            // set timer2 compare value 

    TIMSK0 = 0;             // delete any interrupt sources
        
    TCCR1B = (1<<CS10);     // start timer1 with no prescaling
    TCCR2A = (1<<CS20);     // start timer2 with no prescaling

    while((ASSR & 0x01) | (ASSR & 0x04));       //wait for TCN2UB and TCR2UB to be cleared

    Delay(100);             // wait for external crystal to stabilise
    
    while(!calibrate)
    {
        cli();              // disable global interrupt
        
        TIFR1 = 0xFF;       // clear TIFR1 flags
        TIFR2 = 0xFF;       // clear TIFR2 flags
        
        TCNT1H = 0;         // clear timer1 counter
        TCNT1L = 0;
        TCNT2 = 0;          // clear timer2 counter
           
        while ( !(TIFR2 & (1<<OCF2A)) );   // wait for timer2 compareflag

        TCCR1B = 0;         // stop timer1

        sei();              // enable global interrupt
    
        if ( (TIFR1 & (1<<TOV1)) )
        {
            temp = 0xFFFF;  // if timer1 overflows, set the temp to 0xFFFF
        }
        else
        {                   // read out the timer1 counter value
            tempL = TCNT1L;
            temp = TCNT1H;
            temp = (temp << 8);
            temp += tempL;
        }
    
        if (temp > 6250)
        {
            OSCCAL--;   // the internRC oscillator runs to fast, decrease the OSCCAL
        } else if (temp < 6120)
        {
            OSCCAL++;   // the internRC oscillator runs to slow, increase the OSCCAL
        } else {
            calibrate = TRUE;   // the interRC is correct
        }

        TCCR1B = (1<<CS10);     // start timer1
    }
    TCCR1B = 0;
}
