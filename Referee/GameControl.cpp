#include <cstdio>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <sstream>
#include <iostream>
#include <arpa/inet.h>
#include "Commands.hpp"
#include "GameControl.hpp"
#include "Serial.hpp"
#include "UDP_Broadcast.hpp"


/** initialization */
bool GameControl::init(const char *configFileName, const char *logFileName, bool restart) 
{
	/** HALT */
    _lastCommand = 'H';
    _lastCommandCounter = 0;

    /** set enabled to true */
    _enabled = true;
    
    /** will be set to true if found */
    _hasSerial = false;
    
    /** default serial device */
    _serialDevice = (char *)"/dev/ttyS0";


    /** default multicast address */
    _multicastAddress = "224.5.29.1";
    
    /** default multicast port */
    _multicastPort = 10001;
  
    printf("checking file\n");
    
    if (!readFile(configFileName)) {
        printf("filename bad.\n");
        fprintf(stderr, "ERROR: Cannot read config file %s\n", configFileName);
        return (false);
    }
    
    printf("filename ok\n");

    try {
        _broadcast.setDestination(_multicastAddress, _multicastPort);
    }
    catch (UDP_Broadcast::IOError& e)
    {
        std::cerr << "Broadcast: " << e.what() << std::endl;
    }

    /** for testing */
    print();

    /** open the serial port */
    fprintf(stderr, "Opening Serial Connection on device %s ...\n", _serialDevice);
    if (!_serial.open(_serialDevice, COMM_BAUD_RATE)) {
        fprintf(stderr, "ERROR: Cannot open serial connection..\n");
        //return (false);
    } else {
        _hasSerial = true;
    }
    
    /** intialize the timer */
    _gameInfo.resetTimer();
    _tLast = getCurrentTime();

    if (!_gameInfo.openLog(logFileName)) {
        fprintf(stderr, "ERROR: Cannot open log file %s..\n", logFileName);
        return (false);
    }

    /** restart from saved state */
    if (restart) {
        _gameInfo.load(_saveName);
    }

    return (true);
}
		

void GameControl::close() 
{
    _gameInfo.closeLog();
    _serial.close();
}

void GameControl::print(FILE *f) 
{
    fprintf(f, "Game Settings\n");
    fprintf(f, "\t\timeLimits : First Half %i:%02i\n", 
    		DISP_MIN(_gameInfo.game.timeLimits[FIRST_HALF]), 
    		(int)DISP_SEC(_gameInfo.game.timeLimits[FIRST_HALF]));
    fprintf(f, "\t\tHalf time %i:%02i\n",
            DISP_MIN(_gameInfo.game.timeLimits[HALF_TIME]), 
            (int)DISP_SEC(_gameInfo.game.timeLimits[HALF_TIME]));
    fprintf(f, "\t\tSecond half %i:%02i\n",
            DISP_MIN(_gameInfo.game.timeLimits[SECOND_HALF]), 
            (int)DISP_SEC(_gameInfo.game.timeLimits[SECOND_HALF]));
    fprintf(f, "\t\tOvertime %i:%02i\n",
            DISP_MIN(_gameInfo.game.timeLimits[OVER_TIME1]),
            (int)DISP_SEC(_gameInfo.game.timeLimits[OVER_TIME1]));
    fprintf(f, "\t\tOvertime %i:%02i\n",
            DISP_MIN(_gameInfo.game.timeLimits[OVER_TIME2]),
            (int)DISP_SEC(_gameInfo.game.timeLimits[OVER_TIME2]));

    fprintf(f, "\ttimeouts : number %i, total time %f\n",
            _gameInfo.game.nrTimeouts[0], SEC2MIN(_gameInfo.game.timeouts[0]));
}


/** Send Commands
 *  log commands, send them over serial and change game state
 *  increment command counter */
void GameControl::sendCommand(const char refereeCommand, const char *msg)
{
    _lastCommand = refereeCommand;
    _lastCommandCounter++;

    ethernetSendCommand(refereeCommand, _lastCommandCounter);
    
    if (_hasSerial)
    {
    	_gameInfo.writeLog("Sending %c: %s", refereeCommand, msg);
        _serial.writeByte(refereeCommand);
    }
}


/** send command to ethernet clients */
void GameControl::ethernetSendCommand(const char refereeCommand, const unsigned int counter)
{
    GameStatePacket packet;
    packet.refereeCommand = refereeCommand;
    packet.commandCounter = _lastCommandCounter & 0xFF;
    packet.blueTeamScore = _gameInfo.game.goals[Blue  ] & 0xFF;
    packet.yellowTeamScore = _gameInfo.game.goals[Yellow] & 0xFF;
    packet.timeRemaining = htons((int)floor(_gameInfo.timeRemaining()));

    try
    {
    	_broadcast.sendPacket(&packet, sizeof(packet));
    }
    catch (UDP_Broadcast::IOError& e)
    {
        std::cerr << "!! UDP_Broadcast: " << e.what() << std::endl;
    }
    
}
    
void GameControl::stepTime() 
{
    double tNew = getCurrentTime();
    double dt = tNew - _tLast;
    _tLast = tNew;

    /**  printf("game state %i\n", _gameInfo.game.state); */

    /** save restore file */
    _gameInfo.save(_saveName);

    /** update game time */
    _gameInfo.game.time += dt;
    
    if (_gameInfo.isTimeout()) {
        _gameInfo.game.timeouts[_gameInfo.game.timeoutTeam] -= dt;
        if (_gameInfo.isTimeoutComplete()) {
            stopTimeout();
        }
    } else {
        if ((_gameInfo.game.stage == HALF_TIME) ||
            !_gameInfo.isHalted()) {
            _gameInfo.game.timeTaken += dt;
            for (int x = 0; x < NUM_TEAMS; ++x) {
                if (_gameInfo.game.timePenalty[x] > 0) {
                    _gameInfo.game.timePenalty[x] -= dt;
                } else {
                    _gameInfo.game.timePenalty[x] = 0;
                }
            }
      
        }
        if (_gameInfo.isTimeComplete()) {
            switch (_gameInfo.game.stage) {
            	case PREGAME: 
            	case PRESECONDHALF: 
            	case PREOVERTIME1: 
            	case PREOVERTIME2: 
            		break;
            	case FIRST_HALF:  
            		beginHalfTime(); 
            		break;
            	case HALF_TIME:   
            		beginSecondHalf(); 
            		break;
            	case SECOND_HALF: 
            		if (_gameInfo.isGameTied())
            			beginOvertime1(); 
            		break;
            	case OVER_TIME1:  
            		beginOvertime2(); 
            		break;
            	case OVER_TIME2:
            		if (_gameInfo.isGameTied())
            			beginPenaltyShootout(); 
            		break;
            	case PENALTY_SHOOTOUT:
            		break;
            }
        }
    }

    /** repeat last command (if someone missed it) */
    ethernetSendCommand(_lastCommand, _lastCommandCounter);
}



/** configuration
 * read a config file to fill in parameters */
bool GameControl::readFile(const char *fileName) 
{
    FILE *f;
    char line[MAX_LINE], dname[MAX_LINE], data[MAX_LINE];
    double d;
    int i;
    /** open the file */
    if ((f = fopen(fileName, "rt")) == NULL) {
        fprintf(stderr, "ERROR: Readfile: cannot open file %s\n", fileName);
        return (false);
    }

    while (fgets(line, MAX_LINE, f) != NULL) 
    {
        if ((line[0] != '#') && (strchr(line, '=') != NULL)) 
        {
            if (sscanf(line, " %[a-z_A-Z0-9] = %s", dname, data) == 2) 
            {
                if (strcmp(dname, "SAVENAME") == 0) {
                    _saveName = new char[MAX_LINE];
                    strncpy(_saveName, data, MAX_LINE - 1);
                } else if (strcmp(dname, "SERIALDEVICE") == 0) {
                    _serialDevice = new char[MAX_LINE];
                    strncpy(_serialDevice, data, MAX_LINE - 1);
                } else if (strcmp(dname, "MULTICASTADDRESS") == 0) {
                    _multicastAddress = data;
                } else if (strcmp(dname, "MULTICASTPORT") == 0) {
                    sscanf(data, " %hd", &_multicastPort);
                } else if (strcmp(dname, "TIMEOUT_LIMIT") == 0) {
                    sscanf(data, "%lf", &d);
                    _gameInfo.game.timeouts[(int) Yellow] = MIN2SEC(d);
                    _gameInfo.game.timeouts[(int) Blue] = MIN2SEC(d);
                } else if (strcmp(dname, "NR_TIMEOUTS") == 0) {
                    sscanf(data, " %d", &i);
                    _gameInfo.game.nrTimeouts[(int) Blue] = i;
                    _gameInfo.game.nrTimeouts[(int) Yellow] = i;
                } else if (strcmp(dname, "FIRST_HALF") == 0) {
                    sscanf(data, " %lf", &d);
                    _gameInfo.game.timeLimits[FIRST_HALF] = MIN2SEC(d);
                } else if (strcmp(dname, "HALF_TIME") == 0) {
                    sscanf(data, " %lf", &d);
                    _gameInfo.game.timeLimits[HALF_TIME] = MIN2SEC(d);
                } else if (strcmp(dname, "SECOND_HALF") == 0) {
                    sscanf(data, " %lf", &d);
                    _gameInfo.game.timeLimits[SECOND_HALF] = MIN2SEC(d);
                } else if (strcmp(dname, "OVER_TIME") == 0) {
                    sscanf(data, " %lf", &d);
                    _gameInfo.game.timeLimits[OVER_TIME1] = MIN2SEC(d);
                    _gameInfo.game.timeLimits[OVER_TIME2] = MIN2SEC(d);
                } else if (strcmp(dname, "YELLOWCARD_TIME") == 0) {
                    sscanf(data, " %lf", &d);
                    _gameInfo.game.yellowCardTime = MIN2SEC(d);
                } else {
                    fprintf(stderr, "Unrecognized parameter %s, will be ignored\n", dname);
                }
            }
        }
    }

    /** all done */
    fclose(f);
    return (true);
}


/** game stage commands */
bool GameControl::beginFirstHalf()
{
    if (_enabled) {
        if (_gameInfo.game.stage != PREGAME) 
            return (false);

        /** send the first half signal but we do not oficially begin
         * until start signal is sent */
        setHalt();
    }
    sendCommand(COMM_FIRST_HALF, "Begin first half");
    return (true);
}

bool GameControl::beginHalfTime() 
{ 
    if (_enabled) {
        if (_gameInfo.game.stage != FIRST_HALF) 
            return (false);

        _gameInfo.game.stage = HALF_TIME;
        setHalt();
        _gameInfo.resetTimer();
    }
    sendCommand(COMM_HALF_TIME, "Begin half time");
    return (true);
}

bool GameControl::beginSecondHalf()
{ 
    if (_enabled) {
        if (_gameInfo.game.stage != HALF_TIME) 
            return (false);

        /** again we send signal, but do not officially begin until
         * start is sent */
        setHalt();
        _gameInfo.game.stage = PRESECONDHALF;
    }
    sendCommand(COMM_SECOND_HALF, "Begin second half");
    return (true);
}

bool GameControl::beginOvertime1()
{ 
    if (_enabled) {
        if (_gameInfo.game.stage != SECOND_HALF) 
            return (false);

        _gameInfo.game.stage = PREOVERTIME1;
        setHalt();
        _gameInfo.resetTimer();
    }
    sendCommand(COMM_OVER_TIME1, "Begin overtime");
    return (true);
}

bool GameControl::beginOvertime2()
{ 
    if (_enabled) {
        if (_gameInfo.game.stage != OVER_TIME1) 
            return (false);

        _gameInfo.game.stage = PREOVERTIME2;
        setHalt();
        _gameInfo.resetTimer();
    }
    sendCommand(COMM_OVER_TIME2, "Begin overtime second half");
    return (true);
}

bool GameControl::beginPenaltyShootout()
{ 
    if (_enabled) {
        if ((_gameInfo.game.stage != OVER_TIME2) && 
            (_gameInfo.game.stage != SECOND_HALF)) {
            return (false);
        }

        _gameInfo.game.stage = PENALTY_SHOOTOUT;
        setHalt();
    
        _gameInfo.resetTimer();
    }
    sendCommand(COMM_FIRST_HALF, "Begin Penalty shootout");
    return (true);
}


/** game control commands */
bool GameControl::setHalt()
{ 
    if (_enabled) {
        _gameInfo.game.state = HALTED;
    }
    sendCommand(COMM_HALT, "Halting robots");
    return (true);
}

bool GameControl::setReady()
{ 
    if (!_enabled) {
        sendCommand(COMM_READY, "Starting robots");
    } else {
        if (!_gameInfo.isTimeout() && _gameInfo.isPrestart()) {
            sendCommand(COMM_READY, "Starting robots");
            _gameInfo.setRunning();
      
            /** progress into the first half upon the start signal */
            switch (_gameInfo.game.stage) {
            	case PREGAME:
            		_gameInfo.game.stage = FIRST_HALF;
            		_gameInfo.resetTimer();
            		break;
            	case PRESECONDHALF:
            		_gameInfo.game.stage = SECOND_HALF;
            		_gameInfo.resetTimer();
            		break;
            	case PREOVERTIME1:
            		_gameInfo.game.stage = OVER_TIME1;
            		_gameInfo.resetTimer();
            		break;
            	case PREOVERTIME2:
            		_gameInfo.game.stage = OVER_TIME2;
            		_gameInfo.resetTimer();
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
    if (!_enabled) {
        sendCommand(COMM_START, "Starting robots");
    } else {
        /**    if (!_gameInfo.isTimeout() && _gameInfo.isStopped()) { */
        if (!_gameInfo.isTimeout()) {
            sendCommand(COMM_START, "STarting robots");
            _gameInfo.setRunning();
      
            /** progress into the first half upon the start signal */
            switch (_gameInfo.game.stage) {
            case PREGAME:
                _gameInfo.game.stage = FIRST_HALF;
                _gameInfo.resetTimer();
                break;
            case PRESECONDHALF:
                _gameInfo.game.stage = SECOND_HALF;
                _gameInfo.resetTimer();
                break;
            case PREOVERTIME1:
                _gameInfo.game.stage = OVER_TIME1;
                _gameInfo.resetTimer();
                break;
            case PREOVERTIME2:
                _gameInfo.game.stage = OVER_TIME2;
                _gameInfo.resetTimer();
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

    if (_enabled) {
        /** progress out of half time if we hit stop */
        if (_gameInfo.game.stage == HALF_TIME) {
            beginSecondHalf();
            _gameInfo.setStopped();
        } else {
            _gameInfo.setStopped();
        }
    }
    return (true);
}

/** maybe deprecate */
bool GameControl::setCancel()
{ 
    /** reset timeout if it is canceled */
    if (_gameInfo.game.state == TIMEOUT) {
        _gameInfo.game.nrTimeouts[_gameInfo.game.timeoutTeam]++;
        _gameInfo.game.timeouts[_gameInfo.game.timeoutTeam] = _gameInfo.game.timeoutStartTime;
        _gameInfo.game.state = _gameInfo.game.lastState;
    }
    else 
    {
        /** reset yellow card if it is canceled */
        for (int x = 1; x < NUM_TEAMS; ++x)
            if (_gameInfo.game.timePenalty[x] > 0 && _gameInfo.game.timePenalty[x] > _gameInfo.game.timePenalty[x-1])
                _gameInfo.game.timePenalty[x] = 0.0;
            else if (_gameInfo.game.timePenalty[x-1] > 0)
                _gameInfo.game.timePenalty[x-1] = 0.0;
    }
    sendCommand(COMM_CANCEL, "Sending cancel");
    return (true);
}


/**  timeout control */
bool GameControl::beginTimeout(Team team)
{
    if (_enabled) {
        if ((_gameInfo.nrTimeouts(team) <= 0) || (_gameInfo.timeoutRemaining(team) <= 0)) {
            return (false);
        }
        if (!_gameInfo.isStopped() && !_gameInfo.isHalted())
            return (false);
    
        _gameInfo.game.lastState = _gameInfo.game.state;
        _gameInfo.game.state = TIMEOUT;
        _gameInfo.game.nrTimeouts[(int)team]--;
        _gameInfo.game.timeoutTeam = team;
        _gameInfo.game.timeoutStartTime = _gameInfo.timeoutRemaining(team);
    }
    char msg[256];
    sendCommand(CHOOSETEAM(team, COMM_TIMEOUT_BLUE, COMM_TIMEOUT_YELLOW), 
                concatTeam(msg, "Timeout", team));

    return (true);
}

bool GameControl::stopTimeout()
{ 
    if (_enabled) {
        if (_gameInfo.game.state != TIMEOUT)
            return (false);
	
        /** necessary since we ignore halts for timeouts */
        _gameInfo.game.state = HALTED;
        setHalt();
    }
    sendCommand(COMM_TIMEOUT_END, "End Timeout");
    return (true);
}


/** status commands */
bool GameControl::goalScored(Team team)
{ 
    if (_enabled) {
        if (!_gameInfo.isStopped() && !_gameInfo.isHalted() ||
            (_gameInfo.game.stage == PREGAME))
            return (false);
    
        if (_gameInfo.game.stage == PENALTY_SHOOTOUT) {
            _gameInfo.game.penaltyGoals[(int) team]++;
        } else {
            _gameInfo.game.goals[(int) team]++;
        }
    }
    
    char msg[256];
    sendCommand(CHOOSETEAM(team, COMM_GOAL_BLUE, COMM_GOAL_YELLOW), 
                concatTeam(msg, "Goal scored", team));
    return (true);
}

bool GameControl::removeGoal(Team team)
{ 
    if (_enabled) {
        if (!_gameInfo.isStopped() && !_gameInfo.isHalted() ||
            (_gameInfo.game.stage == PREGAME))
            return (false);
    
        if (_gameInfo.game.stage == PENALTY_SHOOTOUT) {
            if (_gameInfo.game.penaltyGoals[(int) team] > 0)
                _gameInfo.game.penaltyGoals[(int) team]--;
        } else if (_gameInfo.game.goals[team] > 0) {
            _gameInfo.game.goals[(int) team]--;
        }
    }
	
    char msg[256];
    sendCommand(CHOOSETEAM(team, COMM_SUBGOAL_BLUE, COMM_SUBGOAL_YELLOW), 
                concatTeam(msg, "Goal removed", team));
    return (true);
}

bool GameControl::awardYellowCard(Team team)
{ 
    if (_enabled) {
        if (!_gameInfo.isStopped())
            return (false);
    }
  
    _gameInfo.game.timePenalty[team] = _gameInfo.game.yellowCardTime;
  
    char msg[256];
    sendCommand(CHOOSETEAM(team, COMM_YELLOWCARD_BLUE, COMM_YELLOWCARD_YELLOW), 
                concatTeam(msg, "Yellow card awarded", team));
    return (true);
}

bool GameControl::awardRedCard(Team team)
{
    if (_enabled) {
        if (!_gameInfo.isStopped())
            return (false);
    }
	
    _gameInfo.game.timePenalty[team] = 0.0;
  
    ++_gameInfo.game.redCards[team];
  
    char msg[256];
    sendCommand(CHOOSETEAM(team, COMM_REDCARD_BLUE, COMM_REDCARD_YELLOW), 
                concatTeam(msg, "Yellow card awarded", team));
    return (true);
}


/** game restart commands */
bool GameControl::setKickoff(Team team)
{
    if (_enabled) {
        if (!_gameInfo.isStopped() || !_gameInfo.canRestart())
            return (false);
        _gameInfo.setPrestart();
    }
	
    char msg[256];
    sendCommand(CHOOSETEAM(team, COMM_KICKOFF_BLUE, COMM_KICKOFF_YELLOW), 
                concatTeam(msg, "Kickoff", team));
    return (true);
}

bool GameControl::setPenalty(Team team)
{ 
    if (_enabled) {
        if (!_gameInfo.isStopped() || !_gameInfo.canRestart())
            return (false);
	
        _gameInfo.setPrestart();
    }
    char msg[256];
    sendCommand(CHOOSETEAM(team, COMM_PENALTY_BLUE, COMM_PENALTY_YELLOW), 
                concatTeam(msg, "Penalty kick", team));
    return (true);
}

bool GameControl::setDirect(Team team)
{ 
    if (_enabled) {
        if (!_gameInfo.isStopped() || !_gameInfo.canRestart())
            return (false);
	
        _gameInfo.setPrestart();
    }
    char msg[256];
    sendCommand(CHOOSETEAM(team, COMM_DIRECT_BLUE, COMM_DIRECT_YELLOW), 
                concatTeam( msg, "Direct freekick", team));
    return (true);
}

bool GameControl::setIndirect(Team team)
{ 
    if (_enabled) {
        if (!_gameInfo.isStopped() || !_gameInfo.canRestart())
            return (false);
	
        _gameInfo.setPrestart();
    }
    char msg[256];
    sendCommand(CHOOSETEAM(team, COMM_INDIRECT_BLUE, COMM_INDIRECT_YELLOW), 
                concatTeam(msg, "Indirect freekick", team));
    return (true);
}


char *GameControl::concatTeam(char *msg, const char *msgPart, Team team)
{
    strcpy(msg, msgPart);
    if (team == Blue)
        strcat(msg, " Blue");
    else
        strcat(msg, " Yellow");
    return (msg);
}
