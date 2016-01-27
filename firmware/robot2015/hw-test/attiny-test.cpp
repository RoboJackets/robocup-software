#include <mbed.h>
#include "kicker-board.hpp"

Ticker lifeLight;
DigitalOut ledOne(LED1);
DigitalOut ledTwo(LED2);

LocalFileSystem fs("local");

/**
 * timer interrupt based light flicker
 */
void imAlive()
{
    ledOne = !ledOne;
}

/**
 * system entry point
 */
int main() 
{
    lifeLight.attach(&imAlive, 0.25);

    //  initialize kicker board and flash it with new firmware if necessary
    KickerBoard kickerBoard(p11, p12, p13, p27, string("/local/kickerFW"));
    bool kickerSuccess = kickerBoard.flash(true, true) == 0;
    
    while (true)
    {
        //main loop heartbeat
        wait(0.3);
        ledTwo = !ledTwo;
    }

    //clear light for main loop (shows its complete)
    ledTwo = false;
}
