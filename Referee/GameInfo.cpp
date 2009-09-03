#include <ostream>
#include "GameInfo.hpp"

GameInfo::~GameInfo(){

}

const char* strGameState[] = {
    "HALTED",
    "STOPPED",
    "TIMEOUT",
    "PRESTART",
    "RUNNING"
};

const char* strGameStage[] = {
    "PREGAME",
    "FIRST_HALF", 
    "HALF_TIME", 
    "PRESECONDHALF", 
    "SECOND_HALF",
    "PREOVERTIME1", 
    "OVER_TIME1", 
    "PREOVERTIME2", 
    "OVER_TIME2",
    "PENALTY_SHOOTOUT"
};  

const char* strGameRestart[] = {
    "NEUTRAL", 
    "DIRECT", 
    "INDIRECT", 
    "PENALTY", 
    "KICKOFF"
};

/** Create human and machine-readable string for ethernet transmission */
std::ostream &operator<<(std::ostream &s, const GameInfo &info)
{
         const GameInfo::GameData &game = info.game;
         static const std::string crlf("\r\n");
    
         Team t0 = Blue;
         Team t1 = Yellow;
         s << "restart: "    		<< strGameRestart[ game.restart ] 	<< crlf;
         s << "state: "      		<< strGameState[ game.state ]     	<< crlf;
         s << "lastState: "  		<< strGameState[ game.lastState ] 	<< crlf;
         s << "stage: "      		<< strGameStage[ game.stage ]     	<< crlf;
         s << "time: "   			<< game.time                  		<< crlf;
         s << "timeTaken: "			<< game.timeTaken                 	<< crlf;
         s << "restarts: "   		<< game.restarts                   	<< crlf;
         s << "timeoutTeam: "      	<< game.timeoutTeam 				<< crlf;
         s << "timeoutStartTime: " 	<< game.timeoutStartTime 			<< crlf;
        
        

         s << "colors: BLUE, YELLOW\r\n";

         s << "teamNames: "
           << game.teamNames[t0] << ", " << game.teamNames[t1] << crlf;

         s << "goals: " 
           << game.goals[t0] << ", " << game.goals[t1] << "\r\n";

         s << "penaltyGoals: "
           << game.penaltyGoals[t0] << ", " << game.penaltyGoals[t1] << "\r\n";

         s << "yellowCards: "
           << game.yellowCards[t0] << ", " << game.yellowCards[t1] << crlf;

         s << "timePenalty: "
           << game.timePenalty[t0] << ", " << game.timePenalty[t1] << crlf;

         s << "redCards: "
           << game.redCards[t0] << ", " << game.redCards[t1] << crlf;

         s << "penalties: "
           << game.penalties[t0] << ", "<< game.penalties[t1] << crlf;

         s << "freeKicks: "
           << game.freeKicks[t0] << ", " << game.freeKicks[t1] << crlf;

         s << "timeouts: "
           << game.timeouts[t0] << ", " << game.timeouts[t1] << crlf;

         s << "nrTimeouts: "
           << game.nrTimeouts[t0] << ", " << game.nrTimeouts[t1] << crlf;

         return s;
}
