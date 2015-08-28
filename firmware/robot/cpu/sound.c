#include <board.h>

#include "sound.h"
#include "timer.h"

#include "songs/NationalAnthem.h"
#include "songs/music_notes.h"

volatile int music_playing;

// http://www.phy.mtu.edu/~suits/notefreqs.html

const note_t song_victory[] = {{PERIOD(1318), 150},
                               {PERIOD(1108), 150},
                               {PERIOD(987), 150},
                               {PERIOD(830), 150},
                               {PERIOD(987), 150},
                               {PERIOD(1108), 150},
                               {PERIOD(987), 150},
                               {PERIOD(880), 150},
                               {PERIOD(830), 150},
                               {PERIOD(739), 150},
                               {PERIOD(659), 150},
                               {0, 150},
                               {PERIOD(830), 150},
                               {PERIOD(739), 150},
                               {PERIOD(659), 150},
                               {PERIOD(659), 150},
                               {PERIOD(659), 150},
                               {PERIOD(739), 150},
                               {PERIOD(830), 150},
                               {PERIOD(830), 150},
                               {PERIOD(830), 150},
                               {PERIOD(739), 150},
                               {PERIOD(659), 150},
                               {PERIOD(739), 150},
                               {PERIOD(830), 150},
                               {PERIOD(739), 150},
                               {PERIOD(659), 150},
                               {PERIOD(622), 150},
                               {PERIOD(659), 150},
                               {PERIOD(739), 150},
                               {PERIOD(830), 150},
                               {PERIOD(830), 150},
                               {PERIOD(830), 150},
                               {PERIOD(830), 150},
                               {PERIOD(880), 150},
                               {PERIOD(987), 150},
                               {PERIOD(987), 150},
                               {PERIOD(987), 150},
                               {PERIOD(987), 150},
                               {PERIOD(987), 150},
                               {PERIOD(987), 150},
                               {PERIOD(739), 150},
                               {PERIOD(739), 150},
                               {PERIOD(739), 150},
                               {PERIOD(830), 150},
                               {PERIOD(739), 150},
                               {PERIOD(987), 150},
                               {PERIOD(1108), 150},
                               {PERIOD(880), 150},
                               {PERIOD(1108), 150},
                               {PERIOD(880), 150},
                               {PERIOD(1108), 150},
                               {PERIOD(1318), 150},
                               {PERIOD(1244), 150},
                               {PERIOD(1108), 150},
                               {PERIOD(987), 150},
                               {PERIOD(830), 150},
                               {PERIOD(987), 150},
                               {PERIOD(830), 150},
                               {PERIOD(987), 150},
                               {0, 150},
                               {PERIOD(830), 150},
                               {PERIOD(739), 150},
                               {PERIOD(659), 150},
                               {PERIOD(659), 150},
                               {PERIOD(659), 150},
                               {PERIOD(739), 150},
                               {PERIOD(830), 150},
                               {PERIOD(830), 150},
                               {PERIOD(830), 150},
                               {PERIOD(739), 150},
                               {PERIOD(659), 150},
                               {PERIOD(739), 150},
                               {PERIOD(830), 150},
                               {PERIOD(739), 150},
                               {PERIOD(659), 150},
                               {PERIOD(622), 150},
                               {PERIOD(659), 150},
                               {0, 150},
                               {PERIOD(830), 150},
                               {PERIOD(739), 150},
                               {PERIOD(659), 150},
                               {PERIOD(659), 150},
                               {PERIOD(659), 150},
                               {PERIOD(739), 150},
                               {PERIOD(830), 150},
                               {PERIOD(830), 150},
                               {PERIOD(830), 150},
                               {PERIOD(739), 150},
                               {PERIOD(659), 150},
                               {PERIOD(739), 150},
                               {PERIOD(830), 150},
                               {PERIOD(739), 150},
                               {PERIOD(659), 150},
                               {PERIOD(622), 150},
                               {PERIOD(659), 150},
                               {PERIOD(739), 150},
                               {PERIOD(830), 150},
                               {PERIOD(830), 150},
                               {PERIOD(830), 150},
                               {PERIOD(830), 150},
                               {PERIOD(880), 150},
                               {PERIOD(987), 150},
                               {PERIOD(987), 150},
                               {PERIOD(987), 150},
                               {PERIOD(987), 150},
                               {PERIOD(987), 150},
                               {PERIOD(987), 150},
                               {PERIOD(739), 150},
                               {PERIOD(739), 150},
                               {PERIOD(739), 150},
                               {PERIOD(830), 150},
                               {PERIOD(739), 150},
                               {PERIOD(987), 150},
                               {PERIOD(1108), 150},
                               {PERIOD(880), 150},
                               {PERIOD(1108), 150},
                               {PERIOD(880), 150},
                               {PERIOD(1108), 150},
                               {PERIOD(1318), 150},
                               {PERIOD(1244), 150},
                               {PERIOD(1108), 150},
                               {PERIOD(987), 150},
                               {PERIOD(830), 150},
                               {PERIOD(987), 150},
                               {PERIOD(830), 150},
                               {PERIOD(987), 150},
                               {0, 150},
                               {PERIOD(830), 150},
                               {PERIOD(739), 150},
                               {PERIOD(659), 150},
                               {PERIOD(659), 150},
                               {PERIOD(659), 150},
                               {PERIOD(739), 150},
                               {PERIOD(830), 150},
                               {PERIOD(830), 150},
                               {PERIOD(830), 150},
                               {PERIOD(739), 150},
                               {PERIOD(659), 150},
                               {PERIOD(739), 150},
                               {PERIOD(830), 150},
                               {PERIOD(739), 150},
                               {PERIOD(659), 150},
                               {PERIOD(622), 150},
                               {PERIOD(659), 150},
                               {0, 150},
                               {PERIOD(830), 150},
                               {PERIOD(739), 150},
                               {PERIOD(659), 150},
                               {PERIOD(659), 150},
                               {PERIOD(659), 150},
                               {PERIOD(739), 150},
                               {PERIOD(830), 150},
                               {PERIOD(830), 150},
                               {PERIOD(830), 150},
                               {PERIOD(739), 150},
                               {PERIOD(659), 150},
                               {PERIOD(739), 150},
                               {PERIOD(830), 150},
                               {PERIOD(739), 150},
                               {PERIOD(659), 150},
                               {PERIOD(622), 150},
                               {PERIOD(659), 150},
                               {PERIOD(739), 150},
                               {PERIOD(830), 150},
                               {PERIOD(830), 150},
                               {PERIOD(830), 150},
                               {PERIOD(830), 150},
                               {PERIOD(880), 150},
                               {PERIOD(987), 150},
                               {PERIOD(987), 150},
                               {PERIOD(987), 150},
                               {PERIOD(987), 150},
                               {PERIOD(987), 150},
                               {PERIOD(987), 150},
                               {PERIOD(739), 150},
                               {PERIOD(739), 150},
                               {PERIOD(739), 150},
                               {PERIOD(830), 150},
                               {PERIOD(739), 150},
                               {PERIOD(987), 150},
                               {PERIOD(1108), 150},
                               {PERIOD(880), 150},
                               {PERIOD(1108), 150},
                               {PERIOD(880), 150},
                               {PERIOD(1108), 150},
                               {PERIOD(1318), 150},
                               {PERIOD(1244), 150},
                               {PERIOD(1108), 150},
                               {PERIOD(987), 150},
                               {PERIOD(830), 150},
                               {PERIOD(987), 150},
                               {PERIOD(830), 150},
                               {PERIOD(987), 150},
                               {0, 150},
                               {PERIOD(1318), 150},
                               {PERIOD(1108), 150},
                               {PERIOD(987), 150},
                               {PERIOD(830), 150},
                               {PERIOD(987), 150},
                               {PERIOD(1108), 150},
                               {PERIOD(987), 150},
                               {PERIOD(880), 150},
                               {PERIOD(830), 150},
                               {PERIOD(739), 150},
                               {PERIOD(659), 150},
                               {0, 150},
                               {PERIOD(1318), 150},
                               {0, 150},
                               {0, 0}};

const note_t song_startup[] = {{PERIOD(523), 100},
                               {PERIOD(880), 100},
                               {PERIOD(1047), 100},
                               {PERIOD(1397), 100},
                               {0, 0}};

const note_t song_failure[] = {{PERIOD(1397), 200},
                               {PERIOD(523), 500},
                               {PERIOD(1397), 200},
                               {PERIOD(523), 500},
                               {PERIOD(1397), 200},
                               {PERIOD(523), 500},
                               {0, 0}};

const note_t song_overvoltage[] = {{PERIOD(4699), 100},
                                   {0, 50},
                                   {PERIOD(4699), 100},
                                   {0, 100},
                                   {PERIOD(3951), 100},
                                   {0, 50},
                                   {PERIOD(3951), 100},
                                   {0, 100},
                                   {0, 0}};

const note_t song_undervoltage[] = {
    {PERIOD(4699), 100}, {0, 50}, {PERIOD(4699), 100}, {0, 1500}, {0, 0}};

const note_t song_fuse_blown[] = {{PERIOD(4699), 100},
                                  {0, 50},
                                  {PERIOD(4699), 100},
                                  {0, 50},
                                  {PERIOD(4699), 100},
                                  {0, 1500},
                                  {0, 0}};

const note_t song_still_alive[] = {{0, 100},
                                   {PERIOD(784), 200},
                                   {PERIOD(740), 200},
                                   {PERIOD(659), 200},
                                   {0, 10},
                                   {PERIOD(659), 200},
                                   {PERIOD(740), 800},
                                   {0, 1400},

                                   // clears garble
                                   {PERIOD(CLEAR), 1},
                                   {PERIOD(440), 200},
                                   {PERIOD(784), 200},
                                   {PERIOD(740), 200},
                                   {PERIOD(659), 200},
                                   {0, 10},
                                   {PERIOD(659), 400},
                                   {PERIOD(740), 200},
                                   {PERIOD(587), 400},
                                   {PERIOD(659), 200},
                                   {PERIOD(440), 1600},
                                   {0, 10},
                                   {PERIOD(440), 200},
                                   {PERIOD(659), 400},
                                   {PERIOD(740), 200},
                                   {PERIOD(784), 600},
                                   {PERIOD(659), 200},
                                   {PERIOD(554), 400},
                                   {PERIOD(587), 600},
                                   {PERIOD(659), 200},
                                   {PERIOD(440), 200},
                                   {0, 10},
                                   {PERIOD(440), 400},
                                   {PERIOD(740), 1400},
                                   {0, 800},

                                   {PERIOD(CLEAR), 1},
                                   {PERIOD(784), 200},
                                   {PERIOD(740), 200},
                                   {PERIOD(659), 200},
                                   {0, 10},
                                   {PERIOD(659), 200},
                                   {PERIOD(740), 800},
                                   {0, 1400},

                                   {PERIOD(CLEAR), 1},
                                   {PERIOD(440), 200},
                                   {PERIOD(784), 200},
                                   {PERIOD(740), 200},
                                   {PERIOD(659), 200},
                                   {0, 10},
                                   {PERIOD(659), 600},
                                   {PERIOD(740), 200},
                                   {PERIOD(587), 600},
                                   {PERIOD(659), 200},
                                   {PERIOD(440), 1000},

                                   {0, 800},
                                   {PERIOD(CLEAR), 1},
                                   {PERIOD(659), 400},
                                   {PERIOD(740), 200},
                                   {PERIOD(784), 600},
                                   {PERIOD(659), 200},
                                   {PERIOD(554), 400},
                                   {PERIOD(587), 200},
                                   {PERIOD(659), 400},
                                   {PERIOD(440), 200},
                                   {PERIOD(587), 200},
                                   {PERIOD(659), 200},
                                   {PERIOD(698), 200},
                                   {PERIOD(659), 200},
                                   {PERIOD(587), 200},
                                   {PERIOD(523), 200},
                                   {0, 400},

                                   {PERIOD(CLEAR), 1},
                                   {PERIOD(440), 200},
                                   {PERIOD(466), 200},
                                   {PERIOD(523), 400},
                                   {PERIOD(698), 400},
                                   {PERIOD(659), 200},
                                   {PERIOD(587), 200},
                                   {0, 10},
                                   {PERIOD(587), 200},
                                   {PERIOD(523), 200},
                                   {PERIOD(587), 200},
                                   {PERIOD(523), 200},
                                   {0, 10},
                                   {PERIOD(523), 400},
                                   {0, 10},
                                   {PERIOD(523), 400},
                                   {PERIOD(440), 200},
                                   {PERIOD(494), 200},
                                   {PERIOD(523), 400},
                                   {PERIOD(698), 400},
                                   {PERIOD(784), 200},
                                   {PERIOD(698), 200},
                                   {PERIOD(659), 200},
                                   {PERIOD(587), 200},
                                   {0, 10},
                                   {PERIOD(587), 200},
                                   {PERIOD(659), 200},
                                   {PERIOD(698), 400},
                                   {0, 10},
                                   {PERIOD(698), 400},
                                   {PERIOD(784), 200},
                                   {PERIOD(880), 200},
                                   {PERIOD(932), 200},
                                   {0, 10},
                                   {PERIOD(932), 200},
                                   {PERIOD(880), 400},
                                   {PERIOD(784), 400},
                                   {PERIOD(698), 200},
                                   {PERIOD(784), 200},
                                   {PERIOD(880), 200},
                                   {0, 10},
                                   {PERIOD(880), 200},
                                   {PERIOD(780), 400},
                                   {PERIOD(698), 400},
                                   {PERIOD(587), 200},
                                   {PERIOD(523), 200},
                                   {PERIOD(587), 200},
                                   {PERIOD(698), 200},
                                   {0, 10},
                                   {PERIOD(698), 200},
                                   {PERIOD(659), 400},
                                   {0, 10},
                                   {PERIOD(659), 200},
                                   {PERIOD(740), 200},
                                   {0, 10},
                                   {PERIOD(740), 1000}

};

void music_handler(void* arg);
Timer music_timer = {0, 0, music_handler};

void music_handler(void* arg) {
    const note_t* note = (note_t*)arg;

    AT91C_BASE_PWMC->PWMC_DIS = 1 << 3;
    if (note->duration) {
        if (note->period) {
            // Set the period for the new note
            AT91C_BASE_PWMC->PWMC_CH[3].PWMC_CUPDR = note->period;
            AT91C_BASE_PWMC->PWMC_CH[3].PWMC_CDTYR = note->period / 2;
            AT91C_BASE_PWMC->PWMC_ENA = 1 << 3;
        }

        // Set the timer to the next note
        music_timer.arg = (void*)(note + 1);
    } else {
        // End of song
        music_playing = 0;
    }

    // If this is zero, the timer code will remove the timer.
    music_timer.period = note->duration;
}

void music_start(const note_t* song) {
    if (music_playing) {
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
    music_timer.arg = (void*)(song + 1);
    music_timer.time = song[0].duration;
    timer_start(&music_timer);
}

void music_stop() {
    if (music_playing) {
        timer_stop(&music_timer);
        music_playing = 0;
    }
    AT91C_BASE_PWMC->PWMC_DIS = 1 << 3;
}
