#pragma once

#include "GameInfo.hpp"
#include "Serial.hpp"
#include "UDP_Broadcast.hpp"
#include "Commands.hpp"

#define MAX_LINE 256
#define CHOOSETEAM(t, blue, yel) (((t) == Blue) ? (blue) : (yel))

/** structure for determining what buttons are useable */
struct EnableState {
	bool enable;
	bool halt;
	bool ready;
	bool stop;
	bool start;
	bool cancel;
  
	bool timeout[NUM_TEAMS];
	bool endTimeout;

	bool goal[NUM_TEAMS];
	bool subgoal[NUM_TEAMS];
	bool kickoff[NUM_TEAMS];
	bool penalty[NUM_TEAMS];
	bool direct[NUM_TEAMS];
	bool indirect[NUM_TEAMS];
	bool cards;

	EnableState() {
		enable = true;
		halt = true;
		ready = true;
		stop = true;
		start = true;
		cancel = true;
		endTimeout = true;
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
	}
    
	EnableState(GameState state = HALTED, GameStage stage = PREGAME, bool _enabled = true) {
		if (!_enabled) {
			enable = true;
			halt = true;
			ready = true;
			stop = true;
			start = true;
			cancel = true;
			endTimeout = true;
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
			halt = true;
			stop = true;
			cancel = true;
			enable = _enabled;
			bool isStopped = (state == STOPPED);
			bool isPrestart = (state == PRESTART);
			cards = isStopped;
			halt = isStopped;
      
			switch (stage) {
				case PREGAME: 
				case PRESECONDHALF: 
				case PREOVERTIME1: 
				case PREOVERTIME2: 
				case HALF_TIME:
					start = (state==STOPPED || state==PRESTART || state==RUNNING);
					ready = isPrestart;
					setRestarts(isStopped, true);
					setTimeouts(isStopped);
					setGoals(false);
					break;
				case FIRST_HALF:  
				case SECOND_HALF: 
				case OVER_TIME1:
				case OVER_TIME2:
					start = (state==STOPPED || state==PRESTART || state==RUNNING);
					ready = isPrestart;
					setTimeouts(isStopped);
					setGoals(isStopped);
					setRestarts(isStopped);
					break;
				case PENALTY_SHOOTOUT:
					setTimeouts(isStopped);
					setGoals(isStopped);
					setRestarts(isStopped);
					break;
				default:
					setTimeouts(false);
					setGoals(false);
					setRestarts(false);
			}
		}
	}

	void setRestarts(bool en = true, bool kickoffOnly = false) {

		for (int t = 0; t < NUM_TEAMS; t++) {
			kickoff[t] = en;
			if (!kickoffOnly) {
				penalty[t] = en;
				direct[t] = en;
				indirect[t] = en;
			} else {
				penalty[t] = false;
				direct[t] = false;
				indirect[t] = false;
			}
		}
	}
  
	void setGoals(bool en = true) {
		for (int t = 0; t < NUM_TEAMS; t++) {
			goal[t] = en;
			subgoal[t] = en;
		}
	}
	
	void setTimeouts(bool en = true) {
		timeout[0] = en;
		timeout[1] = en;
		endTimeout = en;
	}
};


/** structure for encapsulating all the game control data */
class  GameControl {
	
	/// types ///
	public:
		struct GameStatePacket{
			/** current referee command */
			char refereeCommand;
			/** increments each time new command is set */
			unsigned char commandCounter;  
			/** current score for blue team */
			unsigned char blueTeamScore; 
			/** current score for yellow team */
			unsigned char yellowTeamScore;
			/** seconds remaining for current game stage */
			unsigned short timeRemaining; 
		};

	/// methods ///
	public:
		GameControl();
		~GameControl();
		
		/** initializes and sets up everything */
		bool init(const char *configFileName, const char *logFileName, bool restart = false);

		void close();

		/** debugging printout */
		void print(FILE *f = stdout);

		/** get the info to display */
		GameInfo getGameInfo() {
			return (_gameInfo);
		}

		bool isEnabled() {
			return (_enabled);
		}

		void toggleEnable() {
			_enabled = !_enabled;
			printf("enabled %i\n", _enabled);
		}
		
		void setEnable(bool en = true) {
			_enabled = en;
			printf("setting enabled %i\n", _enabled);
		}

		EnableState getEnableState() {
			EnableState es(_gameInfo.game.state, _gameInfo.game.stage, _enabled);
			return (es);
		}

		/** const std::string
		  * send commands */
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

        void setTeamName(Team team, const std::string &name)
        {
            /** max. 63 chars + null */
            strncpy(_gameInfo.game.teamNames[team], name.c_str(), 63);
            _gameInfo.game.teamNames[team][63] = '\0';
        }  

        bool setHalt();
        bool setReady();
        bool setStart();
        bool setStop();

        /** maybe deprecate */
        bool setCancel();

        /** update times etc */
        void stepTime();
        
        
        /// methods ///
	private:
        /** read a config file to fill in parameters */
        bool readFile(const char *fileName);
        
        /** log commands, send them over serial and change game state */    
        void sendCommand(const char refereeCommand, const char *msg);
        void ethernetSendCommand(const char refereeCommand, const unsigned int counter);
        char *concatTeam(char *msg, const char *msgPart, Team team);
		
		/// members ///
	private:
		char *_saveName;
		Serial _serial;
        UDP_Broadcast _broadcast;
        char *_serialDevice;
        std::string _multicastAddress;
        uint16_t _multicastPort;
        GameInfo _gameInfo;
        double _tLast;
        bool _enabled;
        bool _hasSerial;
        
        /** last Command sent */
        char _lastCommand;
        
        /** incremented if a new command was sent */
        unsigned int _lastCommandCounter;    
};
