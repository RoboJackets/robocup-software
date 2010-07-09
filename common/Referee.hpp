#pragma once

namespace Packet
{
    class Referee
    {
        public:
            uint8_t command;
            uint8_t counter;
            uint8_t scoreBlue;
            uint8_t scoreYellow;
            uint8_t timeHigh;
            uint8_t timeLow;
            
            Referee()
            {
                command = 0;
                counter = 0;
                scoreBlue = 0;
                scoreYellow = 0;
                timeHigh = 0;
                timeLow = 0;
            }
    };
    
}

