/*
 * TITLE: gamecontrol.cc
 *
 * PURPOSE: Wraps the game control functionality for keeping track of time, and other 
 * game info.
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

#include <cstdio>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <sstream>
#include <iostream>
#include <arpa/inet.h> //for htonl()

#include "commands.h"
#include "gamecontrol.h"
#include "serial.h"
#include "udp_broadcast.h"



#define MAX_LINE 256

#define CHOOSETEAM(t, blue, yel) (((t) == Blue) ? (blue) : (yel))

// initializes sets up everything
bool GameControl::init(const char *confname, const char *logfname, bool restart) 
{
    lastCommand = 'H'; //HALT
    lastCommandCounter = 0;

    enabled = true;
    has_serial = false; //will be set to true if found.
    // default serial device
#ifdef WIN32
    serdev = "COM1:";
#else
    serdev = "/dev/ttyS0";
#endif

    // default multicast address
    mc_addr = "224.5.29.1";
    mc_port = 10001;
  
    printf("checking file\n");
    if (!readFile(confname)) {
                                 printf("filename bad.\n");
        fprintf(stderr, "ERROR: Cannot read config file %s\n", confname);
        return (false);
    }
    
    printf("filename ok\n");

    try {
        broadcast.set_destination(mc_addr, mc_port);
    }
    catch (UDP_Broadcast::IOError& e)
    {
        std::cerr << "Broadcast: " << e.what() << std::endl;
    }

    // a little user output
    print();

#if 0
    /* open the serial port */
    fprintf(stderr, "Opening Serial Connection on device %s ...\n", serdev);
    if (!serial.Open(serdev, COMM_BAUD_RATE)) {
        fprintf(stderr, "ERROR: Cannot open serial connection..\n");
        //return (false);
    } else {
        has_serial = true;
    }
#endif

    // intialize the timer
    gameinfo.resetTimer();
    tlast = getTime();

    if (!gameinfo.openLog(logfname)) {
        fprintf(stderr, "ERROR: Cannot open log file %s..\n", logfname);
        return (false);
    }

    // restart from saved state
    if (restart) {
        gameinfo.load(savename);
    }

    return (true);
}
		

void GameControl::close() 
{
    gameinfo.closeLog();
    serial.Close();
}

void GameControl::print(FILE *f) 
{
    fprintf(f, "Game Settings\n");
    fprintf(f, "\ttimelimits : First Half %i:%02i\n",
            DISP_MIN(gameinfo.data.timelimits[FIRST_HALF]), 
            (int)DISP_SEC(gameinfo.data.timelimits[FIRST_HALF]));
    fprintf(f, "\t\tHalf time %i:%02i\n",
            DISP_MIN(gameinfo.data.timelimits[HALF_TIME]), 
            (int)DISP_SEC(gameinfo.data.timelimits[HALF_TIME]));
    fprintf(f, "\t\tSecond half %i:%02i\n",
            DISP_MIN(gameinfo.data.timelimits[SECOND_HALF]), 
            (int)DISP_SEC(gameinfo.data.timelimits[SECOND_HALF]));
    fprintf(f, "\t\tOvertime %i:%02i\n",
            DISP_MIN(gameinfo.data.timelimits[OVER_TIME1]),
            (int)DISP_SEC(gameinfo.data.timelimits[OVER_TIME1]));
    fprintf(f, "\t\tOvertime %i:%02i\n",
            DISP_MIN(gameinfo.data.timelimits[OVER_TIME2]),
            (int)DISP_SEC(gameinfo.data.timelimits[OVER_TIME2]));

    fprintf(f, "\ttimeouts : number %i, total time %f\n",
            gameinfo.data.nrtimeouts[0], SEC2MIN(gameinfo.data.timeouts[0]));
}

/////////////////////////////
// send commands
// log commands, send them over serial and change game state
// increment command counter
void GameControl::sendCommand(char cmd, const char *msg) 
{
    lastCommand = cmd;
    lastCommandCounter++;

    ethernetSendCommand(cmd, lastCommandCounter);
    
    if (has_serial)
    {
        gameinfo.writeLog("Sending %c: %s", cmd, msg);
        serial.WriteByte(cmd);
    }
}


/////////////////////////////
// send command to ethernet clients.
void GameControl::ethernetSendCommand(const char cmd, const unsigned int counter)
{
    GameStatePacket p;
    p.cmd          = cmd;
    p.cmd_counter  = lastCommandCounter & 0xFF;
    p.goals_blue   = gameinfo.data.goals[Blue  ] & 0xFF;
    p.goals_yellow = gameinfo.data.goals[Yellow] & 0xFF;
    p.time_remaining = htons((int)floor(gameinfo.timeRemaining()));

    try
    {
        broadcast.send(&p,sizeof(p));
    }
    catch (UDP_Broadcast::IOError& e)
    {
        std::cerr << "!! UDP_Broadcast: " << e.what() << std::endl;
    }
    
}
    
void GameControl::stepTime() 
{
    double tnew = getTime();
    double dt = tnew - tlast;
    tlast = tnew;

    //  printf("game state %i\n", gameinfo.data.state);

    // save restore file
    gameinfo.save(savename);

    // update game time
    gameinfo.data.gametime += dt;
    
    if (gameinfo.isTimeout()) {
        gameinfo.data.timeouts[gameinfo.data.timeoutteam] -= dt;
        if (gameinfo.isTimeoutComplete()) {
            stopTimeout();
        }
    } else {
        if ((gameinfo.data.stage == HALF_TIME) ||
            !gameinfo.isHalted()) {
            gameinfo.data.time_taken += dt;
            for (int x = 0; x < NUM_TEAMS; ++x) {
                if (gameinfo.data.timepenalty[x] > 0) {
                    gameinfo.data.timepenalty[x] -= dt;
                } else {
                    gameinfo.data.timepenalty[x] = 0;
                }
            }
      
        }
        if (gameinfo.isTimeComplete()) {
            switch (gameinfo.data.stage) {
            case PREGAME: 
            case PRESECONDHALF: 
            case PREOVERTIME1: 
            case PREOVERTIME2: 
                break;
            case FIRST_HALF:  beginHalfTime(); break;
            case HALF_TIME:   beginSecondHalf(); break;
            case SECOND_HALF: 
                if (gameinfo.isGameTied())
                    beginOvertime1(); 
                break;
            case OVER_TIME1:  beginOvertime2(); break;
            case OVER_TIME2:
                if (gameinfo.isGameTied())
                    beginPenaltyShootout(); 
                break;
            case PENALTY_SHOOTOUT:
                break;
            }
        }
    }

    // repeat last command (if someone missed it)
    ethernetSendCommand(lastCommand, lastCommandCounter);
}


////////////////////////////////////////
// configuration
// read a config file to fill in parameters
bool GameControl::readFile(const char *fname) 
{
    FILE *f;
    char line[MAX_LINE], dname[MAX_LINE], data[MAX_LINE];
    double d;
    int i;
    // open the file
    if ((f = fopen(fname, "rt")) == NULL) {
        fprintf(stderr, "ERROR: Readfile: cannot open file %s\n", fname);
        return (false);
    }

    while (fgets(line, MAX_LINE, f) != NULL) 
    {
        if ((line[0] != '#') && (strchr(line, '=') != NULL)) 
        {
            if (sscanf(line, " %[a-z_A-Z0-9] = %s", dname, data) == 2) 
            {
                if (strcmp(dname, "SAVENAME") == 0) {
                    savename = new char[MAX_LINE];
                    strncpy(savename, data, MAX_LINE - 1);
                } else if (strcmp(dname, "SERIALDEVICE") == 0) {
                    serdev = new char[MAX_LINE];
                    strncpy(serdev, data, MAX_LINE - 1);
                } else if (strcmp(dname, "MULTICASTADDRESS") == 0) {
                    mc_addr = data;
                } else if (strcmp(dname, "MULTICASTPORT") == 0) {
                    sscanf(data, " %hd", &mc_port);
                } else if (strcmp(dname, "TIMEOUT_LIMIT") == 0) {
                    sscanf(data, "%lf", &d);
                    gameinfo.data.timeouts[(int) Yellow] = MIN2SEC(d);
                    gameinfo.data.timeouts[(int) Blue] = MIN2SEC(d);
                } else if (strcmp(dname, "NR_TIMEOUTS") == 0) {
                    sscanf(data, " %d", &i);
                    gameinfo.data.nrtimeouts[(int) Blue] = i;
                    gameinfo.data.nrtimeouts[(int) Yellow] = i;
                } else if (strcmp(dname, "FIRST_HALF") == 0) {
                    sscanf(data, " %lf", &d);
                    gameinfo.data.timelimits[FIRST_HALF] = MIN2SEC(d);
                } else if (strcmp(dname, "HALF_TIME") == 0) {
                    sscanf(data, " %lf", &d);
                    gameinfo.data.timelimits[HALF_TIME] = MIN2SEC(d);
                } else if (strcmp(dname, "SECOND_HALF") == 0) {
                    sscanf(data, " %lf", &d);
                    gameinfo.data.timelimits[SECOND_HALF] = MIN2SEC(d);
                } else if (strcmp(dname, "OVER_TIME") == 0) {
                    sscanf(data, " %lf", &d);
                    gameinfo.data.timelimits[OVER_TIME1] = MIN2SEC(d);
                    gameinfo.data.timelimits[OVER_TIME2] = MIN2SEC(d);
                } else if (strcmp(dname, "YELLOWCARD_TIME") == 0) {
                    sscanf(data, " %lf", &d);
                    gameinfo.data.yellowcard_time = MIN2SEC(d);
                } else {
                    fprintf(stderr, "Unrecognized parameter %s, will be ignored\n", dname);
                }
            }
        }
    }

    // all done
    fclose(f);
    return (true);
}




/////////////////////////////
// game stage commands
bool GameControl::beginFirstHalf()
{
    if (enabled) {
        if (gameinfo.data.stage != PREGAME) 
            return (false);

        // send the first half signal but we do not oficially begin
        // until start signal is sent
        setHalt();
    }
    sendCommand(COMM_FIRST_HALF, "Begin first half");
    return (true);
}

bool GameControl::beginHalfTime() 
{ 
    if (enabled) {
        if (gameinfo.data.stage != FIRST_HALF) 
            return (false);

        gameinfo.data.stage = HALF_TIME;
        setHalt();
        gameinfo.resetTimer();
    }
    sendCommand(COMM_HALF_TIME, "Begin half time");
    return (true);
}

bool GameControl::beginSecondHalf()
{ 
    if (enabled) {
        if (gameinfo.data.stage != HALF_TIME) 
            return (false);

        // again we send signal, but do not officially begin until
        // start is sent
        setHalt();
        gameinfo.data.stage = PRESECONDHALF;
    }
    sendCommand(COMM_SECOND_HALF, "Begin second half");
    return (true);
}

bool GameControl::beginOvertime1()
{ 
    if (enabled) {
        if (gameinfo.data.stage != SECOND_HALF) 
            return (false);

        gameinfo.data.stage = PREOVERTIME1;
        setHalt();
        gameinfo.resetTimer();
    }
    sendCommand(COMM_OVER_TIME1, "Begin overtime");
    return (true);
}

bool GameControl::beginOvertime2()
{ 
    if (enabled) {
        if (gameinfo.data.stage != OVER_TIME1) 
            return (false);

        gameinfo.data.stage = PREOVERTIME2;
        setHalt();
        gameinfo.resetTimer();
    }
    sendCommand(COMM_OVER_TIME2, "Begin overtime second half");
    return (true);
}

bool GameControl::beginPenaltyShootout()
{ 
    if (enabled) {
        if ((gameinfo.data.stage != OVER_TIME2) && 
            (gameinfo.data.stage != SECOND_HALF)) {
            return (false);
        }

        gameinfo.data.stage = PENALTY_SHOOTOUT;
        setHalt();
    
        gameinfo.resetTimer();
    }
    sendCommand(COMM_FIRST_HALF, "Begin Penalty shootout");
    return (true);
}

/////////////////////////////////
// game control commands
bool GameControl::setHalt()
{ 
    if (enabled) {
        gameinfo.data.state = HALTED;
    }
    sendCommand(COMM_HALT, "Halting robots");
    return (true);
}

bool GameControl::setReady()
{ 
    if (!enabled) {
        sendCommand(COMM_READY, "STarting robots");
    } else {
        if (!gameinfo.isTimeout() && gameinfo.isPrestart()) {
            sendCommand(COMM_READY, "STarting robots");
            gameinfo.setRunning();
      
            // progress into teh first half upon the start signal
            switch (gameinfo.data.stage) {
            case PREGAME:
                gameinfo.data.stage = FIRST_HALF;
                gameinfo.resetTimer();
                break;
            case PRESECONDHALF:
                gameinfo.data.stage = SECOND_HALF;
                gameinfo.resetTimer();
                break;
            case PREOVERTIME1:
                gameinfo.data.stage = OVER_TIME1;
                gameinfo.resetTimer();
                break;
            case PREOVERTIME2:
                gameinfo.data.stage = OVER_TIME2;
                gameinfo.resetTimer();
                break;
            default:
                break;
            }
        }
    }
    return (true);
}

bool GameControl::setStart()
{ 
    if (!enabled) {
        sendCommand(COMM_START, "STarting robots");
    } else {
        //    if (!gameinfo.isTimeout() && gameinfo.isStopped()) {
        if (!gameinfo.isTimeout()) {
            sendCommand(COMM_START, "STarting robots");
            gameinfo.setRunning();
      
            // progress into teh first half upon the start signal
            switch (gameinfo.data.stage) {
            case PREGAME:
                gameinfo.data.stage = FIRST_HALF;
                gameinfo.resetTimer();
                break;
            case PRESECONDHALF:
                gameinfo.data.stage = SECOND_HALF;
                gameinfo.resetTimer();
                break;
            case PREOVERTIME1:
                gameinfo.data.stage = OVER_TIME1;
                gameinfo.resetTimer();
                break;
            case PREOVERTIME2:
                gameinfo.data.stage = OVER_TIME2;
                gameinfo.resetTimer();
                break;
            default:
                break;
            }
        }
    }
    return true;
}

bool GameControl::setStop()
{ 
    sendCommand(COMM_STOP, "Stopping robots");

    if (enabled) {
        // progress out of half time if we hit stop
        if (gameinfo.data.stage == HALF_TIME) {
            beginSecondHalf();
            gameinfo.setStopped();
        } else {
            gameinfo.setStopped();
        }
    }
    return (true);
}

// maybe deprecate
bool GameControl::setCancel()
{ 
    // reset timeout if it is canceled
    if (gameinfo.data.state == TIMEOUT) {
        gameinfo.data.nrtimeouts[gameinfo.data.timeoutteam]++;
        gameinfo.data.timeouts[gameinfo.data.timeoutteam] = gameinfo.data.timeoutstarttime;
        gameinfo.data.state = gameinfo.data.laststate;
    }
    else 
    {
        // reset yellow card if it is canceled
        for (int x = 1; x < NUM_TEAMS; ++x)
            if (gameinfo.data.timepenalty[x] > 0 && gameinfo.data.timepenalty[x] > gameinfo.data.timepenalty[x-1])
                gameinfo.data.timepenalty[x] = 0.0;
            else if (gameinfo.data.timepenalty[x-1] > 0)
                gameinfo.data.timepenalty[x-1] = 0.0;
    }
    sendCommand(COMM_CANCEL, "Sending cancel");
    return (true);
}

///////////////////
// timeout control
bool GameControl::beginTimeout(Team team)
{
    if (enabled) {
        if ((gameinfo.nrTimeouts(team) <= 0) || (gameinfo.timeoutRemaining(team) <= 0)) {
            return (false);
        }
        if (!gameinfo.isStopped() && !gameinfo.isHalted())
            return (false);
    
        gameinfo.data.laststate = gameinfo.data.state;
        gameinfo.data.state = TIMEOUT;
        gameinfo.data.nrtimeouts[(int)team]--;
        gameinfo.data.timeoutteam = team;
        gameinfo.data.timeoutstarttime = gameinfo.timeoutRemaining(team);
    }
    char msg[256];
    sendCommand(CHOOSETEAM(team, COMM_TIMEOUT_BLUE, COMM_TIMEOUT_YELLOW), 
                concatTeam(msg, "Timeout", team));

    return (true);
}

bool GameControl::stopTimeout()
{ 
    if (enabled) {
        if (gameinfo.data.state != TIMEOUT)
            return (false);
	
        // necessary since we ignore halts for timeouts
        gameinfo.data.state = HALTED;
        setHalt();
    }
    sendCommand(COMM_TIMEOUT_END, "End Timeout");
    return (true);
}


// status commands
bool GameControl::goalScored(Team team)
{ 
    if (enabled) {
        if (!gameinfo.isStopped() && !gameinfo.isHalted() ||
            (gameinfo.data.stage == PREGAME))
            return (false);
    
        if (gameinfo.data.stage == PENALTY_SHOOTOUT) {
            gameinfo.data.penaltygoals[(int) team]++;
        } else {
            gameinfo.data.goals[(int) team]++;
        }
    }
    
    char msg[256];
    sendCommand(CHOOSETEAM(team, COMM_GOAL_BLUE, COMM_GOAL_YELLOW), 
                concatTeam(msg, "Goal scored", team));
    return (true);
}

bool GameControl::removeGoal(Team team)
{ 
    if (enabled) {
        if (!gameinfo.isStopped() && !gameinfo.isHalted() ||
            (gameinfo.data.stage == PREGAME))
            return (false);
    
        if (gameinfo.data.stage == PENALTY_SHOOTOUT) {
            if (gameinfo.data.penaltygoals[(int) team] > 0)
                gameinfo.data.penaltygoals[(int) team]--;
        } else if (gameinfo.data.goals[team] > 0) {
            gameinfo.data.goals[(int) team]--;
        }
    }
	
    char msg[256];
    sendCommand(CHOOSETEAM(team, COMM_SUBGOAL_BLUE, COMM_SUBGOAL_YELLOW), 
                concatTeam(msg, "Goal removed", team));
    return (true);
}

bool GameControl::awardYellowCard(Team team)
{ 
    if (enabled) {
        if (!gameinfo.isStopped())
            return (false);
    }
  
    gameinfo.data.timepenalty[team] = gameinfo.data.yellowcard_time;
  
    char msg[256];
    sendCommand(CHOOSETEAM(team, COMM_YELLOWCARD_BLUE, COMM_YELLOWCARD_YELLOW), 
                concatTeam(msg, "Yellow card awarded", team));
    return (true);
}

bool GameControl::awardRedCard(Team team)
{
    if (enabled) {
        if (!gameinfo.isStopped())
            return (false);
    }
	
    gameinfo.data.timepenalty[team] = 0.0;
  
    ++gameinfo.data.redcards[team];
  
    char msg[256];
    sendCommand(CHOOSETEAM(team, COMM_REDCARD_BLUE, COMM_REDCARD_YELLOW), 
                concatTeam(msg, "Yellow card awarded", team));
    return (true);
}


// game restart commands
bool GameControl::setKickoff(Team team)
{
    if (enabled) {
        if (!gameinfo.isStopped() || !gameinfo.canRestart())
            return (false);
        gameinfo.setPrestart();
    }
	
    char msg[256];
    sendCommand(CHOOSETEAM(team, COMM_KICKOFF_BLUE, COMM_KICKOFF_YELLOW), 
                concatTeam(msg, "Kickoff", team));
    return (true);
}

bool GameControl::setPenalty(Team team)
{ 
    if (enabled) {
        if (!gameinfo.isStopped() || !gameinfo.canRestart())
            return (false);
	
        gameinfo.setPrestart();
    }
    char msg[256];
    sendCommand(CHOOSETEAM(team, COMM_PENALTY_BLUE, COMM_PENALTY_YELLOW), 
                concatTeam(msg, "Penalty kick", team));
    return (true);
}

bool GameControl::setDirect(Team team)
{ 
    if (enabled) {
        if (!gameinfo.isStopped() || !gameinfo.canRestart())
            return (false);
	
        gameinfo.setPrestart();
    }
    char msg[256];
    sendCommand(CHOOSETEAM(team, COMM_DIRECT_BLUE, COMM_DIRECT_YELLOW), 
                concatTeam( msg, "Direct freekick", team));
    return (true);
}

bool GameControl::setIndirect(Team team)
{ 
    if (enabled) {
        if (!gameinfo.isStopped() || !gameinfo.canRestart())
            return (false);
	
        gameinfo.setPrestart();
    }
    char msg[256];
    sendCommand(CHOOSETEAM(team, COMM_INDIRECT_BLUE, COMM_INDIRECT_YELLOW), 
                concatTeam(msg, "Indirect freekick", team));
    return (true);
}


char *GameControl::concatTeam(char *msg, const char *msgpart, Team team)
{
    strcpy(msg, msgpart);
    if (team == Blue)
        strcat(msg, " Blue");
    else
        strcat(msg, " Yellow");
    return (msg);
}
 
