#include <ostream>
#include "gameinfo.h"


const char* str_GameState[] = {
    "HALTED", "STOPPED", "TIMEOUT", "PRESTART", "RUNNING"
};

const char* str_GameStage[] = {
    "PREGAME",
    "FIRST_HALF", "HALF_TIME", "PRESECONDHALF", "SECOND_HALF",
    "PREOVERTIME1", "OVER_TIME1", "PREOVERTIME2", "OVER_TIME2",
    "PENALTY_SHOOTOUT"
};  

const char* str_GameRestart[] = {
    "NEUTRAL", "DIRECT", "INDIRECT", "PENALTY", "KICKOFF"
};


// /** create human- and machine-readable string for ethernet transmission */
// std::ostream& operator<<(std::ostream& s, const GameInfo& info)
// {
//         const GameInfo::Data& data = info.data;
//         static const std::string crlf("\r\n");
    
//         Team t0 = Blue;
//         Team t1 = Yellow;
//         s << "restart:"    << str_GameRestart[ data.restart ] << crlf;
//         s << "state:"      << str_GameState[ data.state ]     << crlf;
//         s << "laststate:"  << str_GameState[ data.laststate ] << crlf;
//         s << "stage:"      << str_GameStage[ data.stage ]     << crlf;
//         s << "gametime:"   << data.time_taken                 << crlf;
//         s << "time_taken:" << data.time_taken                 << crlf;
//         s << "restarts:"   << data.restarts                   << crlf;
//         s << "timeoutteam:"      << data.timeoutteam << crlf;
//         s << "timeoutstarttime:" << data.timeoutteam << crlf;
        
        

//         s << "colors:BLUE,YELLOW\r\n";

//         s << "teamnames:"
//           << data.teamnames[t0] << "," << data.teamnames[t1] << crlf;

//         s << "goals:" 
//           << data.goals[t0] << "," << data.goals[t1] << "\r\n";

//         s << "penaltygoals:"
//           << data.penaltygoals[t0] << ","<< data.penaltygoals[t1] << "\r\n";

//         s << "yellowcards:"
//           << data.yellowcards[t0] << ","<< data.yellowcards[t1] << crlf;

//         s << "timepenalty:"
//           << data.timepenalty[t0] << ","<< data.timepenalty[t1] << crlf;

//         s << "redcards:"
//           << data.redcards[t0] << ","<< data.redcards[t1] << crlf;

//         s << "penalties:"
//           << data.penalties[t0] << ","<< data.penalties[t1] << crlf;

//         s << "freekicks:"
//           << data.freekicks[t0] << ","<< data.freekicks[t1] << crlf;

//         s << "timeouts:"
//           << data.timeouts[t0] << ","<< data.timeouts[t1] << crlf;

//         s << "nrtimeouts:"
//           << data.nrtimeouts[t0] << ","<< data.nrtimeouts[t1] << crlf;

//         return s;
// }
