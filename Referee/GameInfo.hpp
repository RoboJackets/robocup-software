#pragma once

#include <sstream>
#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <time.h>
#include <sys/time.h>
#include <math.h>

#define MAX(a, b) 	(((a) > (b)) ? (a) : (b))
#define MIN2SEC(t) 	((t) * 60.0)
#define SEC2MIN(t) 	((t) / 60.0)
#define DISP_MIN(t)	(((t) <= 0) ? 0 : (int) floor((t) / 60.0))
#define DISP_SEC(t)	(((t) <= 0) ? 0 : fmod((t), 60.0))
#define NR_GAME_STAGES (((int) PENALTY_SHOOTOUT) + 1)
#define NUM_TEAMS 2

/** Game state enum
 *
 *	0 - Halted
 *	1 - Stopped
 *	2 - Timeout
 *	3 - Prestart
 *	4 - Running */	
typedef enum
{
	HALTED,
	STOPPED,
	TIMEOUT,
	PRESTART,
	RUNNING
} GameState;

/** Game stage enum
 * 
 * 0 - Pre Game
 * 1 - First Half
 * 2 - Half Time
 * 3 - Pre Second Half
 * 4 - Second Half
 * 5 - Pre Overtime 1
 * 6 - Overtime 1
 * 7 - Pre Overtime 2
 * 8 - Overtime 2
 * 9 - Penalty Shootout */
typedef enum
{
	PREGAME,
	FIRST_HALF,
	HALF_TIME,
	PRESECONDHALF,
	SECOND_HALF,
	PREOVERTIME1,
	OVER_TIME1,
	PREOVERTIME2,
	OVER_TIME2,
	PENALTY_SHOOTOUT
} GameStage;

/** Game restart enum
 * 
 * 0 - Neutral
 * 1 - Direct
 * 2 - Indirect
 * 3 - Penalty
 * 4 - Kickoff */
typedef enum
{
	NEUTRAL,
	DIRECT,
	INDIRECT,
	PENALTY,
	KICKOFF
} GameRestart;

/** Team enum
 * 
 * 0 - Blue
 * 1 - Yellow */
typedef enum
{
	Blue,
	Yellow
} Team;

/** Current time function */
static double getCurrentTime() {

    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);
    return (currentTime.tv_sec + currentTime.tv_usec / 1000000.0);
}


class GameInfo {
	public:
		typedef struct {
			char teamNames[NUM_TEAMS][64];
			GameRestart restart;
			GameState state;
			GameStage stage;
			GameState lastState; 
	    
			/** time game started */
			time_t start;
	    
			/** time of the game */
			double time;
		
			double timeTaken;
			double timeLimits[NR_GAME_STAGES];		
			double timeouts[NUM_TEAMS];
			int nrTimeouts[NUM_TEAMS];
			int timeoutTeam;
			double timeoutStartTime;	        
			int goals[NUM_TEAMS];
			int penaltyGoals[NUM_TEAMS];
			int yellowCards[NUM_TEAMS];
			double timePenalty[NUM_TEAMS];
			double yellowCardTime;
			int redCards[NUM_TEAMS];
			int penalties[NUM_TEAMS];
			int freeKicks[NUM_TEAMS];
			int restarts;
		} GameData; 

		GameData game;
		FILE *logfile;

		
		GameInfo() {
			printf("Gameinfo\n");
			memset(&game, 0, sizeof(game));
			game.start = time(NULL);
			game.time = getCurrentTime();
		} 
		~GameInfo();

		bool openLog(const char *fileName) {
			if ((logfile = fopen(fileName, "w")) == NULL)
				return (false);
			fprintf(logfile, "%f ", game.time);
			fprintf(logfile, "Game log for %s\n", ctime(&game.start));
			return (true); 
		}

		void closeLog() {
			fclose(logfile);
		}

		bool save(char *fileName) {
			FILE *file;
			if ((file = fopen(fileName, "w")) == NULL)
				return (false);
			bool returnVal = (fwrite(&game, sizeof(game), 1, file) == sizeof(game));
			fclose(file);
			return (returnVal);
		}

		bool load(char *fileName) {
			FILE *file;
			if ((file = fopen(fileName, "r")) == NULL)
				return (false);
			bool returnVal = (fread(&game, sizeof(game), 1, file) == sizeof(game));
			game.state = HALTED;
			fclose(file);
			return (returnVal);
		}

		void writeLog(const char *format, ...) {
			va_list varg;
			va_start(varg, format);
			fprintf(logfile, "%f ", game.time);
			fprintf(logfile, "\t%s\t%s\t", getStageString(), getStateString());
			fprintf(logfile, "%f ", game.timeTaken);
			vfprintf(logfile, format, varg);
			fprintf(logfile, "\n");
			va_end(varg);
		}

		const  char *getStateString() const {
			switch (game.state) {
			case HALTED:  
				return ("Halted"); 
				break;
			case STOPPED: 
				return ("Stopped"); 
				break;
			case TIMEOUT: 
				return ("Timeout"); 
				break;
			case PRESTART:   
				return ("Prestart"); 
				break;
			case RUNNING: 
				return ("Running"); 
				break;
			default: 
				return ("Unknown state!!!!\n"); 
				break;
			}
		}

		const char *getStageString() const {
			switch (game.stage) {
			case PREGAME:  
				return ("Pre-game"); 
				break;
			case PRESECONDHALF:  
				return ("Pre-second half"); 
				break;
			case PREOVERTIME1:  
				return ("Pre-overtime first half"); 
				break;
			case PREOVERTIME2:  
				return ("Pre-overtime second half"); 
				break;
			case FIRST_HALF:  
				return ("First Half"); 
				break;
			case HALF_TIME:   
				return ("Half Time"); 
				break;
			case SECOND_HALF: 
				return ("Second Half"); 
				break;
			case OVER_TIME1:   
				return ("Overtime first half"); 
				break;
			case OVER_TIME2:   
				return ("Overtime second half"); 
				break;
			case PENALTY_SHOOTOUT: 
				return ("Penalty Shootout"); 
				break;
			default: 
				return ("Unknown stage!!!!\n"); 
				break;
			}
		}
		
		const char *getRestartString() const {
			switch (game.stage) {
			case NEUTRAL:  
				return ("Neutral"); 
				break;
			case DIRECT:   
				return ("Direct"); 
				break;
			case INDIRECT: 
				return ("Indirect"); 
				break;
			case PENALTY:  
				return ("Penalty"); 
				break;
			case KICKOFF:  
				return ("Kickoff"); 
				break;
			default: 
				return ("Unknown restart!!!!\n"); 
				break;
			}
		}

		bool isTimeComplete() {
			return (game.timeTaken >= game.timeLimits[(int) game.stage]);
		}

		double timeRemaining() {
			return (MAX(0, game.timeLimits[(int) game.stage] - game.timeTaken));
		}
    
		double timeTaken() {
			return (game.timeTaken);
		}

		bool isTimeout() {
			return (game.state == TIMEOUT);
		}
    
		bool isHalted() {
			return (game.state == HALTED);
		}
    
		bool isStopped() {
			return (game.state == STOPPED);
		}
    
		bool isPrestart() {
			return (game.state == PRESTART);
		}
    
		bool isRunning() {
			return (game.state == RUNNING);
		}

		bool isGeneralPlay() {
			return (game.stage == FIRST_HALF || game.stage == SECOND_HALF 
					|| game.stage == OVER_TIME1 || game.stage == OVER_TIME2);
		}

		bool isGameTied() {
			return (game.goals[Blue] == game.goals[Yellow]);
		}

		double timeoutRemaining(int team = -1) {
			return (game.timeouts[((team < 0) ? game.timeoutTeam : team)]);
		}

		int nrTimeouts(int team = -1) {
			return (game.nrTimeouts[((team < 0) ? game.timeoutTeam : team)]);
		}

		bool isTimeoutComplete() {
			return (game.timeouts[game.timeoutTeam] <= 0);
		}

		void resetTimer() {
			game.timeTaken = 0;
		}

		bool canRestart() {
			return (game.state == STOPPED);
		}

		void setRunning() {
			game.state = RUNNING;
		}
    
		void setPrestart() {
			game.state = PRESTART;
		}
    
		void setStopped() {
			game.state = STOPPED;
		}

		void setTimelimits(double tlim[], double touts[], int ntouts) {
			memcpy(game.timeLimits, tlim, NR_GAME_STAGES * sizeof(double));
			memcpy(game.timeouts, touts, NUM_TEAMS * sizeof(double));
			game.nrTimeouts[0] = ntouts;
			game.nrTimeouts[1] = ntouts;
		}
        
		double penaltyTimeRemaining(int team = 0) {
			return (game.timePenalty[team]);
		}
};


std::ostream &operator<<(std::ostream &s, const GameInfo &info);
