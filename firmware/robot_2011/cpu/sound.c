#include <board.h>

#include "sound.h"
#include "timer.h"

volatile int music_playing;

// http://www.phy.mtu.edu/~suits/notefreqs.html

const note_t song_startup[] =
{
	{PERIOD( 523), 100},
	{PERIOD( 880), 100},
	{PERIOD(1047), 100},
	{PERIOD(1397), 100},
	{0, 0}
};

const note_t song_failure[] =
{
	{PERIOD(1397), 200},
	{PERIOD( 523), 500},
	{PERIOD(1397), 200},
	{PERIOD( 523), 500},
	{PERIOD(1397), 200},
	{PERIOD( 523), 500},
	{0, 0}
};

const note_t song_overvoltage[] =
{
	{PERIOD(4699), 100},
	{0,            50},
	{PERIOD(4699), 100},
	{0,            100},
	{PERIOD(3951), 100},
	{0,            50},
	{PERIOD(3951), 100},
	{0,            100},
	{0, 0}
};

const note_t song_undervoltage[] =
{
	{PERIOD(4699), 100},
	{0,            50},
	{PERIOD(4699), 100},
	{0,            1500},
	{0, 0}
};

const note_t song_fuse_blown[] =
{
	{PERIOD(4699), 100},
	{0,            50},
	{PERIOD(4699), 100},
	{0,            50},
	{PERIOD(4699), 100},
	{0,            1500},
	{0, 0}
};

void music_handler(void *arg);
timer_t music_timer = {0, 0, music_handler};

void music_handler(void *arg)
{
	const note_t *note = (note_t *)arg;
	
	AT91C_BASE_PWMC->PWMC_DIS = 1 << 3;
	if (note->duration)
	{
		if (note->period)
		{
			// Set the period for the new note
			AT91C_BASE_PWMC->PWMC_CH[3].PWMC_CUPDR = note->period;
			AT91C_BASE_PWMC->PWMC_CH[3].PWMC_CDTYR = note->period / 2;
			AT91C_BASE_PWMC->PWMC_ENA = 1 << 3;
		}
		
		// Set the timer to the next note
		music_timer.arg = (void *)(note + 1);
	} else {
		// End of song
		music_playing = 0;
	}
	
	// If this is zero, the timer code will remove the timer.
	music_timer.period = note->duration;
}

void music_start(const note_t *song)
{
	if (music_playing)
	{
		music_stop();
	}
	
	music_playing = 1;
	
	// Turn on PWM in PMC
	AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PWMC;
	// Set period and duty cycle
	AT91C_BASE_PWMC->PWMC_MR = 0;
	AT91C_BASE_PWMC->PWMC_CH[3].PWMC_CMR = 0x607;
	AT91C_BASE_PWMC->PWMC_CH[3].PWMC_CPRDR = song[0].period;
	AT91C_BASE_PWMC->PWMC_CH[3].PWMC_CDTYR = song[0].period / 2;
	AT91C_BASE_PWMC->PWMC_ENA = 1 << 3;
	// Buzzer is driven by PWM3
	AT91C_BASE_PIOA->PIO_BSR = BUZZ;
	AT91C_BASE_PIOA->PIO_PDR = BUZZ;
	
	// Start a timer to play the next note
	music_timer.arg = (void *)(song + 1);
	music_timer.time = song[0].duration;
	timer_start(&music_timer);
}

void music_stop()
{
	if (music_playing)
	{
		timer_stop(&music_timer);
		music_playing = 0;
	}
	AT91C_BASE_PWMC->PWMC_DIS = 1 << 3;
}
