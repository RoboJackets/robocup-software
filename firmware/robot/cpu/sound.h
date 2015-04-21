#pragma once

#define MCK_FREQ	48000000
#define PWM3_CLK	(MCK_FREQ / 128)
#define PERIOD(hz)	(PWM3_CLK / hz)

// A song is an array of notes.
typedef struct
{
	// Use the PERIOD(hz) macro to convert frequency to period.
	// A period of zero is silence.
	// The first note in a song should not be silent (see the various errata for the PWM module).
	int period;
	
	// Time to play this note in milliseconds.
	// There must be a zero-duration note at the end of each song.
	int duration;
} note_t;

////////////////
// Songs

// Normal startup
extern const note_t song_startup[];

//Celebration 
extern const note_t song_victory[];

// still_alive
extern const note_t song_still_alive[];

//Anthem
extern const note_t song_national_anthem[];

// Critical hardware failed at startup
extern const note_t song_failure[];

// Power supply failure
extern const note_t song_overvoltage[];
extern const note_t song_undervoltage[];
extern const note_t song_fuse_blown[];

// Nonzero while music is playing.
extern volatile int music_playing;

void music_start(const note_t *song);
void music_stop(void);
