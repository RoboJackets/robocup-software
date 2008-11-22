#ifndef _GAME_INFO_HPP
#define _GAME_INFO_HPP

#include <sstream>
#include <cstdarg>
#include <time.h>
#include <sys/time.h>
#include <math.h>

#define MAX(a, b) 	(((a) > (b)) ? (a) : (b))
#define MIN2SEC(t) 	((t) * 60.0)
#define SEC2MIN(t) 	((t) / 60.0)
#define DISP_MIN(t)	(((t) <= 0) ? 0 : (int) floor((t) / 60.0))
#define DISP_SEC(t)	(((t) <= 0) ? 0 : fmod((t), 60.0))
#define NUM_TEAMS 2
#define NR_GAME_STAGES (((int) PENALTY_SHOOTOUT) + 1)

/* game stage and status enums */
enum GameState {HALTED = 0, STOPPED, TIMEOUT, PRESTART, RUNNING};	
enum GameStage {PREGAME = 0, FIRST_HALF, HALF_TIME, PRESECONDHALF, SECOND_HALF, 
                PREOVERTIME1, OVER_TIME1, PREOVERTIME2, OVER_TIME2, PENALTY_SHOOTOUT};	
enum GameRestart {NEUTRAL = 0, DIRECT, INDIRECT, PENALTY, KICKOFF};
enum Team {Blue = 0, Yellow};

// time function
//static double getTime() {
//
//    struct timeval curr;
//    gettimeofday(&curr, NULL);
//    return (curr.tv_sec + curr.tv_usec / 1000000.0);
//}


#endif /*_GAME_INFO_HPP_*/
