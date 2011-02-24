#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

#include "main.h"
#include "LCD_driver.h"
#include "button.h"

enum
{
    Menu_Trigger,
    Menu_Period,
    Menu_Time_On,
    Menu_Burst_Mode,
    Menu_Burst_Count,
    Menu_Velocity,
    Menu_DTime,
    Menu_Distance,
    Menu_Off,
    
    Num_Menu_Items
};

uint8_t input_max[] = {6, 5, 5, 3, 5};
uint8_t input_buf[5];
uint8_t input_pos;

uint16_t period = 10000;
uint16_t on_time = 1000;
    
// Additional 8 bits to augment timer 0
uint16_t tcnt0_high;

// Nonzero when t0tr_high wraps
uint8_t tcnt0_overflow;

// Distance between sensor centers in mm
uint16_t speed_distance = 25;

// Set after both sensors have been triggered
#define SPEED_DONE  1
// Prevents speed_dtime from being changed
#define SPEED_HOLD  2
volatile uint8_t speed_flags = 0;       // Combination of SPEED_* above

// If nonzero, use burst_count
volatile char burst = 1;
volatile uint16_t burst_remaining = 0;
uint16_t burst_count = 1;

void input_move(uint8_t new_pos)
{
    lcd_putc(input_pos, input_buf[input_pos] + '0');
    input_pos = new_pos;
    lcd_putc(input_pos, input_buf[input_pos] + '0' + 0x80);
    lcd_update();
    lcd_flash_off();
}

uint16_t input_value()
{
    uint8_t i;
    uint16_t value = 0;
    
    for (i = 0; i < 5; ++i)
    {
        value = value * 10 + input_buf[i];
    }
    
    return value;
}

char input_set(uint8_t new_value, uint16_t min, uint16_t max)
{
    char ret = 1;
    uint8_t old = input_buf[input_pos];
    input_buf[input_pos] = new_value;
    
    // Revert if the new value is greater than the maximum value
    uint8_t i;
    for (i = 0; i < 5; ++i)
    {
        if (input_buf[i] < input_max[i])
        {
            break;
        } else if (input_buf[i] > input_max[i])
        {
            input_buf[input_pos] = old;
            ret = 0;
        }
    }
    
    uint16_t value = input_value();
    if (value < min || value > max)
    {
        input_buf[input_pos] = old;
        ret = 0;
    }
    
    lcd_putc(input_pos, input_buf[input_pos] + '0' + 0x80);
    lcd_update();
    
    return ret;
}

uint16_t input_number(uint16_t value, uint16_t min, uint16_t max, void (*update)(uint16_t))
{
    char i;
    
    lcd_clear();
    
    for (i = 4; i >= 0; --i)
    {
        input_buf[(uint8_t)i] = value % 10;
        value /= 10;
    }
    
    for (i = 0; i < 5; ++i)
    {
        lcd_putc(i, input_buf[(uint8_t)i] + '0');
    }
    
    lcd_update();
    
    input_move(0);
    
    while (1)
    {
        char ch = getch();
        uint8_t cur = input_buf[input_pos];
        switch (ch)
        {
            case KEY_LEFT:
                if (input_pos)
                {
                    input_move(input_pos - 1);
                }
                break;
            
            case KEY_RIGHT:
                if (input_pos < 4)
                {
                    input_move(input_pos + 1);
                }
                break;
            
            case KEY_UP:
                if (cur < 9)
                {
                    if (input_set(cur + 1, min, max) && update)
                    {
                        update(input_value());
                    }
                }
                break;
            
            case KEY_DOWN:
                if (cur > 0)
                {
                    if (input_set(cur - 1, min, max) && update)
                    {
                        update(input_value());
                    }
                }
                break;
            
            case KEY_CENTER:
                return input_value();
        }
    }
}

void sleep()
{
    // Disable LCD
    LCDCRA &= ~(1 << 7);

    set_sleep_mode(SLEEP_MODE_PWR_SAVE);

    while (PINB & 0x40)
    {
        sleep_mode();
    }
    
    // UP pressed
    uint8_t i;
    for (i = 0; i < 20; i++)
    {
        *(pLCDREG + i) = 0x00;
    }
    
    // Enable LCD
    LCDCRA |= 1 << 7;
}

void set_period(uint16_t value)
{
    TCCR1B &= ~7;
    
    // Set TOP
    ICR1H = value >> 8;
    ICR1L = value;
    
    // Make the timer overflow on the next count
    // so the output will turn on.
    --value;
    TCNT1H = value >> 8;
    TCNT1L = value;
    
    TCCR1B |= 1;
}

void set_on_time(uint16_t value)
{
    OCR1AH = value >> 8;
    OCR1AL = value;
}

ISR(TIMER0_OVF_vect)
{
    tcnt0_high++;
    if (tcnt0_high == 0)
    {
        tcnt0_overflow = 1;
    }
}

ISR(TIMER1_COMPA_vect)
{
    if (burst)
    {
        burst_remaining--;
        if (burst_remaining == 0)
        {
            // Stop timer 1
            TCCR1B &= ~7;
        }
    }
}

void burst_trigger()
{
    if (burst_count)
    {
        burst_remaining = burst_count;
        
        set_period(period);
        /*
        // Make the counter overflow so the output will go high
        // (the output only goes high when the timer resets to zero
        // while counting).
        TCNT1H = 0xff;
        TCNT1L = 0xff;
        
        TCCR1B |= 1;*/
    } else {
        burst_remaining = 0;
    }
}

void speed_interrupt()
{
    if (!(PINE & (1 << 4)))
    {
        // Start
        //
        // Reset and start the timer if we haven't finished a reading
        // or we are replacing one.
        if (TCCR0A == 0 && (!(speed_flags & SPEED_DONE) || !(speed_flags & SPEED_HOLD)))
        {
            speed_flags &= ~SPEED_DONE;
            tcnt0_overflow = 0;
            tcnt0_high = 0;
            TCNT0 = 0;
            TCCR0A = 1;
        }
    }
    
    if (!(PINE & (1 << 5)))
    {
        // Stop
        if (TCCR0A)
        {
            TCCR0A = 0;
            speed_flags |= SPEED_DONE;
        }
    }
}

void show_velocity(uint8_t time_only)
{
    // Keep speed readings until they are displayed
    speed_flags |= SPEED_HOLD;
    
    lcd_puts_r(PSTR("-- ---"));

    while (1)
    {
        if (speed_flags & SPEED_DONE)
        {
            if (tcnt0_overflow)
            {
                tcnt0_overflow = 0;
                lcd_puts_r(PSTR("SLOW"));
            } else {
                // Time in microseconds
                uint32_t dtime = (uint32_t)TCNT0 + ((uint32_t)tcnt0_high << 8);
                
                // Velocity in mm/s
                // v(mm/s) = d(mm) / (t(us) / 1e6)
                uint32_t v;
                
                if (time_only)
                {
                    v = dtime / 1000;
                } else {
                    v = speed_distance * 1000000 / dtime;
                }
                
                if (v > 99999)
                {
                    lcd_puts_r(PSTR("FAST"));
                } else {
                    uint8_t i;
                    
                    lcd_clear();
                    for (i = 0; i < 5; i++)
                    {
                        uint8_t digit = v % 10;
                        v /= 10;
                        
                        if (i < 3)
                        {
                            lcd_putc(5 - i, '0' + digit);
                        } else {
                            lcd_putc(4 - i, '0' + digit);
                        }
                    }
                    lcd_update();
                }
            }
            
            // Allow another measurement
            speed_flags &= ~SPEED_DONE;
        }
        
        char ch = getkey();
        
        switch (ch)
        {
            case KEY_CENTER:
            case KEY_LEFT:
                // Exit
                speed_flags &= ~SPEED_HOLD;
                return;
            
            case KEY_RIGHT:
                // Trigger
                burst = 1;
                burst_trigger();
                break;
            
            case KEY_UP:
            case KEY_DOWN:
                // Reset
                TCCR0A = 0;
                TCNT0 = 0;
                tcnt0_overflow = 0;
                lcd_puts_r(PSTR("-- ---"));
                break;
        }
    }
}

int main()
{
    Initialization();

    cli();
    
    set_period(period);
    set_on_time(on_time);
    
    // WGM mode 14
    TCCR1A = 0x82;
    TCCR1B = 0x18;
    TCCR1C = 0x00;
    TIMSK1 = 0x02;
    sei();
    
    // Timer 0 is used for break-beam timing
    TCCR0A = 0;
    TIMSK0 = 1;
    
    uint8_t cur = 0;
    
    while (1)
    {
        switch (cur)
        {
            case Menu_Period:
                lcd_puts_r(PSTR("PERIOD"));
                break;
            
            case Menu_Time_On:
                lcd_puts_r(PSTR("T-ON"));
                break;
            
            case Menu_Trigger:
                lcd_puts_r(PSTR("TRIG"));
                break;
                
            case Menu_Burst_Mode:
                if (burst)
                {
                    lcd_puts_r(PSTR("BURST"));
                } else {
                    lcd_puts_r(PSTR("CONT"));
                }
                break;
                
            case Menu_Burst_Count:
                lcd_puts_r(PSTR("COUNT"));
                break;
            
            case Menu_Velocity:
                lcd_puts_r(PSTR("V M/S"));
                break;
                
            case Menu_DTime:
                lcd_puts_r(PSTR("^TIME"));
                break;
                
            case Menu_Distance:
                lcd_puts_r(PSTR("D MM"));
                break;
            
            case Menu_Off:
                lcd_puts_r(PSTR("Off"));
                break;
        }
        
        char k = getch();
        switch (k)
        {
            case KEY_UP:
                if (cur)
                {
                    cur--;
                } else {
                    cur = Num_Menu_Items - 1;
                }
                break;
                
            case KEY_DOWN:
                if (cur < (Num_Menu_Items - 1))
                {
                    cur++;
                } else {
                    cur = 0;
                }
                break;
                
            case KEY_CENTER:
            case KEY_RIGHT:
                switch (cur)
                {
                    case Menu_Period:
                        period = input_number(period, 1, 65535, set_period);
                        break;
                
                    case Menu_Time_On:
                        on_time = input_number(on_time, 1, 65535, set_on_time);
                        break;
                    
                    case Menu_Burst_Mode:
                        burst ^= 1;
                        burst_trigger();
                        break;
                    
                    case Menu_Burst_Count:
                        burst_count = input_number(burst_count, 1, 65535, 0);
                        break;
                        
                    case Menu_Trigger:
                        burst = 1;
                        burst_trigger();
                        break;
                        
                    case Menu_Velocity:
                        show_velocity(0);
                        break;
                    
                    case Menu_DTime:
                        show_velocity(1);
                        break;
                    
                    case Menu_Distance:
                        speed_distance = input_number(speed_distance, 1, 1000, 0);
                        break;
                        
                    case Menu_Off:
                        sleep();
                        cur = 0;
                        break;
                }
                break;
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

    PORTB = (15<<PB0);       // Enable pullup on 
    PORTE = (15<<PE4);

    DDRB |= 1 << 5;               // set OC1A as output
    PORTB = 0;

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
