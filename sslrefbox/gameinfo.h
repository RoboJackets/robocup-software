/*
 * TITLE: gameinfo.h
 *
 * PURPOSE: Stores the status of the game
 *
 * WRITTEN BY: Brett Browning 
 */
/* LICENSE:  =========================================================================
    RoboCup F180 Referee Box Source Code Release
  -------------------------------------------------------------------------
    Copyright (C) 2003 RoboCup Federation
  -------------------------------------------------------------------------
    This software is distributed under the GNU General Public License,
    version 2.  If you do not have a copy of this licence, visit
    www.gnu.org, or write: Free Software Foundation, 59 Temple Place,
    Suite 330 Boston, MA 02111-1307 USA.  This program is distributed
    in the hope that it will be useful, but WITHOUT ANY WARRANTY,
    including MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  ------------------------------------------------------------------------- 

 */

#ifndef __GAME_INFO_H__
#define __GAME_INFO_H__

#include <string.h>
#include <sstream>
#include <cstdarg>  //va_list
#include <time.h>
#ifdef WIN32
#include <sys/timeb.h>
#else
#include <sys/time.h>
#endif
#include <math.h>

#define MAX(a, b)  (((a) > (b)) ? (a) : (b))


#define MIN2SEC(t)					((t) * 60.0)
#define SEC2MIN(t)					(t / 60.0)

#define DISP_MIN(t)  (((t) <= 0) ? 0 : (int) floor((t) / 60.0))
#define DISP_SEC(t)  (((t) <= 0) ? 0 : fmod((t), 60.0))

// time function
static double getTime() {
#ifdef WIN32
    struct timeb curr;
    ftime(&curr);
    return (curr.time + curr.millitm / 1000.0);
#else
    struct timeval curr;
    gettimeofday(&curr, NULL);
    return (curr.tv_sec + curr.tv_usec / 1000000.0);
#endif
}



#define NUM_TEAMS 2
enum Team {Blue = 0, Yellow};

/* game stage and status enums */
enum GameState {HALTED = 0, STOPPED, TIMEOUT, PRESTART, RUNNING};	
enum GameStage {PREGAME = 0, FIRST_HALF, HALF_TIME, PRESECONDHALF, SECOND_HALF, 
                PREOVERTIME1, OVER_TIME1, PREOVERTIME2, OVER_TIME2, PENALTY_SHOOTOUT};	
#define NR_GAME_STAGES (((int) PENALTY_SHOOTOUT) + 1)
enum GameRestart {NEUTRAL = 0, DIRECT, INDIRECT, PENALTY, KICKOFF};

class GameInfo {
public:
    struct Data {
        char teamnames[NUM_TEAMS][64];
        GameRestart restart;
        GameState state;
        GameStage stage;
        GameState laststate;
            
        time_t gamestart; // time game started
        double gametime; // time of the game
		
        double time_taken;

        double timelimits[NR_GAME_STAGES];
		
        double timeouts[NUM_TEAMS];
        int    nrtimeouts[NUM_TEAMS];
        int    timeoutteam;
        double timeoutstarttime;
            
        int goals[NUM_TEAMS];
        int penaltygoals[NUM_TEAMS];
        int yellowcards[NUM_TEAMS];
        double timepenalty[NUM_TEAMS];
        double yellowcard_time;
        int redcards[NUM_TEAMS];
        int penalties[NUM_TEAMS];
        int freekicks[NUM_TEAMS];
        int restarts;
    };        

    Data data;
    FILE *logfile;

    GameInfo() {
        printf("Gameinfo\n");
        memset(&data, 0, sizeof(data));
        data.gamestart = time(NULL);
        data.gametime = getTime();
    }

    bool openLog(const char *fname) {
        if ((logfile = fopen(fname, "w")) == NULL)
            return (false);
        fprintf(logfile, "%f ", data.gametime);
        fprintf(logfile, "Game log for %s\n", ctime(&data.gamestart));
        return (true); 
    }

    void closeLog() {
        fclose(logfile);
    }

    bool save(char *fname) {
        FILE *f;
        if ((f = fopen(fname, "w")) == NULL)
            return (false);
        bool rval = (fwrite(&data, sizeof(data), 1, f) == sizeof(data));
        fclose(f);
        return (rval);
    }

    bool load(char *fname) {
        FILE *f;
        if ((f = fopen(fname, "r")) == NULL)
            return (false);
        bool rval = (fread(&data, sizeof(data), 1, f) == sizeof(data));
        data.state = HALTED;
        fclose(f);
        return (rval);
    }

    void writeLog(char *fmt, ...) {
        va_list varg;
        va_start(varg, fmt);
        fprintf(logfile, "%f ", data.gametime);
        fprintf(logfile, "\t%s\t%s\t", getStageString(), getStateString());
        fprintf(logfile, "%f ", data.time_taken);
        vfprintf(logfile, fmt, varg);
        fprintf(logfile, "\n");
        va_end(varg);
    }

    const char *getStateString() const {
        switch (data.state) {
        case HALTED:  return ("Halted"); break;
        case STOPPED: return ("Stopped"); break;
        case TIMEOUT: return ("Timeout"); break;
        case PRESTART:   return ("Prestart"); break;
        case RUNNING: return ("Running"); break;
        default: return ("Unknown state!!!!\n"); break;
        }
    }

    const char *getStageString() const {
        switch (data.stage) {
        case PREGAME:  return ("Pre-game"); break;
        case PRESECONDHALF:  return ("Pre-second half"); break;
        case PREOVERTIME1:  return ("Pre-overtime first half"); break;
        case PREOVERTIME2:  return ("Pre-overtime second half"); break;
        case FIRST_HALF:  return ("First Half"); break;
        case HALF_TIME:   return ("Half Time"); break;
        case SECOND_HALF: return ("Second Half"); break;
        case OVER_TIME1:   return ("Overtime first half"); break;
        case OVER_TIME2:   return ("Overtime second half"); break;
        case PENALTY_SHOOTOUT: return ("Penalty Shootout"); break;
        default: return ("Unknown stage!!!!\n"); break;
        }
    }
		
    const char *getRestartString() const {
        switch (data.stage) {
        case NEUTRAL:  return ("Neutral"); break;
        case DIRECT:   return ("Direct"); break;
        case INDIRECT: return ("Indirect"); break;
        case PENALTY:  return ("Penalty"); break;
        case KICKOFF:  return ("Kickoff"); break;
        default: return ("Unknown restart!!!!\n"); break;
        }
    }

    bool isTimeComplete() {
        return (data.time_taken >= data.timelimits[(int) data.stage]);
    }

    double timeRemaining() {
        return (MAX(0, data.timelimits[(int) data.stage] - data.time_taken));
    }
    double timeTaken() {
        return (data.time_taken);
    }

    bool isTimeout() {
        return (data.state == TIMEOUT);
    }
    bool isHalted() {
        return (data.state == HALTED);
    }
    bool isStopped() {
        return (data.state == STOPPED);
    }
    bool isPrestart() {
        return (data.state == PRESTART);
    }
    bool isRunning() {
        return (data.state == RUNNING);
    }

    bool isGeneralPlay() {
        return (data.stage == FIRST_HALF || data.stage == SECOND_HALF 
                || data.stage == OVER_TIME1 || data.stage == OVER_TIME2);
    }

    bool isGameTied() {
        return (data.goals[Blue] == data.goals[Yellow]);
    }

    double timeoutRemaining(int team = -1) {
        return (data.timeouts[((team < 0) ? data.timeoutteam : team)]);
    }

    int nrTimeouts(int team = -1) {
        return (data.nrtimeouts[((team < 0) ? data.timeoutteam : team)]);
    }

    bool isTimeoutComplete() {
        return (data.timeouts[data.timeoutteam] <= 0);
    }

    void resetTimer() {
        data.time_taken = 0;
    }

    bool canRestart() {
        return (data.state == STOPPED);
    }

    void setRunning() {
        data.state = RUNNING;
    }
    void setPrestart() {
        data.state = PRESTART;
    }
    void setStopped() {
        data.state = STOPPED;
    }

    void setTimelimits(double tlim[], double touts[], int ntouts) {
        memcpy(data.timelimits, tlim, NR_GAME_STAGES * sizeof(double));
        memcpy(data.timeouts, touts, NUM_TEAMS * sizeof(double));
        data.nrtimeouts[0] = ntouts;
        data.nrtimeouts[1] = ntouts;
    }
        
    double penaltyTimeRemaining(int team = 0) {
        return (data.timepenalty[team]);
    }
};


// std::ostream& operator<<(std::ostream& s, const GameInfo& info);

#endif



