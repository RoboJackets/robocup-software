/*
 * TITLE: gamecontrol.h
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

#ifndef __GAME_CONTROL_H__
#define __GAME_CONTROL_H__

#include "gameinfo.h"
#include "serial.h"
#include "udp_broadcast.h"
#include "commands.h"


// structure for determining what buttons are useable
struct EnableState {
  bool enable;
  bool halt;
  bool ready;
  bool stop;
  bool start;
  bool cancel;
  
  bool timeout[NUM_TEAMS];
  bool endtimeout;

  bool goal[NUM_TEAMS];
  bool subgoal[NUM_TEAMS];
  bool kickoff[NUM_TEAMS];
  bool penalty[NUM_TEAMS];
  bool direct[NUM_TEAMS];
  bool indirect[NUM_TEAMS];
  bool cards;

  EnableState() {
    enable = halt = ready = stop = start = cancel = true;
    endtimeout = true;
    cards = true;
    for (int t = 0; t < NUM_TEAMS; t++) {
      timeout[t] = goal[t] = subgoal[t] = true;
      kickoff[t] = penalty[t] = direct[t] = indirect[t] = true;
    }
  }
    
  EnableState(GameState state = HALTED, GameStage stage = PREGAME, 
              bool enabled = true) {

    if (!enabled) {
      enable = halt = ready = stop = start = cancel = true;
      endtimeout = true;
      cards = true;
      for (int t = 0; t < NUM_TEAMS; t++) {
        timeout[t] = true;
        goal[t] = true;
        subgoal[t] = true;
        kickoff[t] = true;
        penalty[t] = true;
        direct[t] = true;
        indirect[t] = true;
      }
    } else {
      halt = stop = cancel = true;
      enable = enabled;
      bool isstopped = (state == STOPPED);
      bool isprestart = (state == PRESTART);
      cards = isstopped;
      halt = isstopped;
      switch (stage) {
      case PREGAME: 
      case PRESECONDHALF: 
      case PREOVERTIME1: 
      case PREOVERTIME2: 
      case HALF_TIME:
        //        start = !halt; //isstopped;
        start = (state==STOPPED || state==PRESTART || state==RUNNING);
        ready = isprestart;
        setRestarts(isstopped, true);
        setTimeouts(isstopped);
        setGoals(false);
        break;
      case FIRST_HALF:  
      case SECOND_HALF: 
      case OVER_TIME1:
      case OVER_TIME2:
        start = (state==STOPPED || state==PRESTART || state==RUNNING);
        //        start = !halt; //isstopped;
        ready = isprestart;
        setTimeouts(isstopped);
        setGoals(isstopped);
        setRestarts(isstopped);
        break;
      case PENALTY_SHOOTOUT:
        setTimeouts(isstopped);
        setGoals(isstopped);
        setRestarts(isstopped);
        break;
      default:
        setTimeouts(false);
        setGoals(false);
        setRestarts(false);
      }
    }
  }

  void setRestarts(bool en = true, bool kickoffonly = false) {

    for (int t = 0; t < NUM_TEAMS; t++) {
      kickoff[t] = en;
      if (!kickoffonly) {
        penalty[t] = direct[t] = indirect[t] = en;
      } else {
        penalty[t] = direct[t] = indirect[t] = false;
      }
    }
  }
  void setGoals(bool en = true) {
    for (int t = 0; t < NUM_TEAMS; t++) {
      goal[t] = subgoal[t] = en;
    }
  }
  void setTimeouts(bool en = true) {
    timeout[0] = timeout[1] = endtimeout = en;
  }
};


/* structure for encapsulating all the game control data */
class  GameControl {
public:
  struct GameStatePacket{
    char cmd;                      // current referee command
    unsigned char cmd_counter;     // increments each time new command is set
    unsigned char goals_blue;      // current score for blue team
    unsigned char goals_yellow;    // current score for yellow team
    unsigned short time_remaining; // seconds remaining for current game stage
  };

private:
	char *savename;
	Serial serial;
        UDP_Broadcast broadcast;
	char *serdev;
        std::string mc_addr;
        uint16_t mc_port;
	GameInfo gameinfo;
	double tlast;

	// configuration
	// read a config file to fill in parameters
	bool readFile(const char *fname);

  // last Command sent.
  char         lastCommand;

  // incremented if a new command was sent.
  unsigned int lastCommandCounter;    

  // log commands, send them over serial and change game state    
  void sendCommand(const char cmd, const char *msg);
  void ethernetSendCommand(const char cmd, const unsigned int counter);
  char *concatTeam(char *msg, const char *msgpart, Team team);

  bool enabled;
  bool has_serial;


public:

	// initializes sets up everything
	bool init(const char *confname,
              const char *logfname,
              bool restart = false);

	void close();

	// debugging printout
	void print(FILE *f = stdout);

	// get the info to display
	GameInfo getGameInfo() {
		return (gameinfo);
	}

  bool isEnabled() {
    return (enabled);
  }

  void toggleEnable() {
    enabled = !enabled;
    printf("enabled %i\n", enabled);
  }
  void setEnable(bool en = true) {
    enabled = en;
    printf("setting enabled %i\n", enabled);
  }

  EnableState getEnableState() {
    EnableState es(gameinfo.data.state, gameinfo.data.stage, enabled);
    return (es);
  }


	/////////////////////////////
	// send commands
	bool beginFirstHalf();
	bool beginHalfTime();
	bool beginSecondHalf();
	bool beginOvertime1();
	bool beginOvertime2();
	bool beginPenaltyShootout();

	bool beginTimeout(Team team);
	bool stopTimeout();

	bool goalScored(Team team);
	bool removeGoal(Team team);

	bool awardYellowCard(Team team);
	bool awardRedCard(Team team);

	bool setKickoff(Team team);
	bool setPenalty(Team team);
	bool setDirect(Team team);
	bool setIndirect(Team team);

        void setTeamName(Team team, const std::string& name)
        {
            //max. 63 chars + null
            strncpy(gameinfo.data.teamnames[team], name.c_str(), 63);
            gameinfo.data.teamnames[team][63] = '\0';
        }  

	bool setHalt();
	bool setReady();
	bool setStart();
	bool setStop();

	// maybe deprecate
	bool setCancel();

	// update times etc
	void stepTime();

};



#endif
